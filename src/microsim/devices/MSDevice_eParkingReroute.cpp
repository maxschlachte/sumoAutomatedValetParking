/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2021 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSDevice_eParkingReroute.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    11.06.2013
///
// A device which stands as an implementation parkinginfo and which outputs movereminder calls
/****************************************************************************/
#include <config.h>

#include <mutex>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSStop.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_eParkingReroute.h"
#include "MSDevice_Battery.h"

//#define DEBUG_COND (getHolder().getID() == "f_eAuto_1.327")
//#define DEBUG_COND (getHolder().getID() == "f_eAuto.6" || getHolder().getID() == "f_eAuto.2")
#define DEBUG_COND (false)
//#define DEBUG_COND (isSelected())
#define VERBOSE (false)

std::vector<std::pair<MSVehicle*,MSDevice_Battery*>> MSDevice_eParkingReroute::myRegisteredVehicles;

// ===========================================================================
// (cre): method definitions for eParkingReroute device
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_eParkingReroute::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("eParkingReroute Device");
    insertDefaultAssignmentOptions("eparkingreroute", "eParkingReroute Device", oc);

    oc.doRegister("device.eparkingreroute.maxCharge", new Option_Float(1.0f));
    oc.doRegister("device.eparkingreroute.prioritizeLowCharges", new Option_Bool(true));
    oc.doRegister("device.eparkingreroute.begEdges", new Option_String());
    oc.doRegister("device.eparkingreroute.endEdges", new Option_String());
    oc.addDescription("device.eparkingreroute.maxCharge", "eParkingReroute Device", "The percentile maximum charge at which a vehicle is still moved to a charging space (default = 1.0).");
    oc.addDescription("device.eparkingreroute.prioritizeLowCharges", "eParkingReroute Device", "Whether the vehicles with the lowest charge should be prioritized (default = true).");
    oc.addDescription("device.eparkingreroute.begEdges", "eParkingReroute Device", "Beginning edges.");
    oc.addDescription("device.eparkingreroute.endEdges", "eParkingReroute Device", "End edges.");
}


void
MSDevice_eParkingReroute::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    // Check if vehicle should get a eParkingReroute
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "eParkingReroute", v, false)) {
        MSVehicle* MSv = dynamic_cast<MSVehicle*>(&v);
        if (!MSv->hasDevice("battery")) {
            WRITE_ERROR("MSDevice_eParkingReroute defined for a vehicle without MSDevice_Battery. MSDevice_Battery has to be defined before MSDevice_eParkingReroute.")
        } else {
            MSDevice_Battery* vBattery = dynamic_cast<MSDevice_Battery*>(MSv->getDevice(typeid(MSDevice_Battery)));
            bool callOnlyMode = false;
            if (v.getVehicleType().getParameter().knowsParameter("callOnly")) {
                try {
                    callOnlyMode = StringUtils::toBool(v.getVehicleType().getParameter().getParameter("callOnly", "false"));
                } catch (...) {
                    WRITE_WARNING("Invalid value '" + v.getVehicleType().getParameter().getParameter("callOnly", "false") + "'for vType parameter 'callOnly'");
                }
            }
            if ((vBattery->getActualBatteryCapacity() / vBattery->getMaximumBatteryCapacity()) > oc.getFloat("device.eparkingreroute.maxCharge")) {
                callOnlyMode = true;
            }
            // get custom vehicle parameter
            std::vector<MSParkingArea*> targetAreas;
            if (v.getVehicleType().getParameter().knowsParameter("eParkingReroute.targetAreas")) {
                try {
                    std::string targetAreaIDs = v.getVehicleType().getParameter().getParameter("eParkingReroute.targetAreas", "");
                    size_t pos = 0;
                    // parse the target areas
                    StringTokenizer st(targetAreaIDs, " ", true);
                    while (st.hasNext()) {
                        MSParkingArea* targetArea = static_cast<MSParkingArea*>(MSNet::getInstance()->getStoppingPlace(StringUtils::prune(st.next()), SUMO_TAG_PARKING_AREA));
                        if (targetArea != nullptr) {
                            targetAreas.push_back(targetArea);
                        }
                    }
                } catch (...) {
                    WRITE_WARNING("Invalid value '" + v.getParameter().getParameter("eParkingReroute.targetArea", "") + "'for vehicle parameter 'eParkingReroute.targetArea'");
                }
            } else {
                WRITE_WARNING("vehicle '" + v.getID() + "' does not supply vehicle parameter 'targetArea'!");
            }
            std::vector<std::string> begIDs;
            std::vector<std::string> endIDs;
            std::string begIDstr = oc.getString("device.eparkingreroute.begEdges");
            StringTokenizer stBeg(begIDstr, " ", true);
            while (stBeg.hasNext()) {
                begIDs.push_back(StringUtils::prune(stBeg.next()));
            }
            std::string endIDstr = oc.getString("device.eparkingreroute.endEdges");
            StringTokenizer stEnd(endIDstr, " ", true);
            while (stEnd.hasNext()) {
                endIDs.push_back(StringUtils::prune(stEnd.next()));
            }
            MSDevice_eParkingReroute* device = new MSDevice_eParkingReroute(v, "eparkingreroute_" + v.getID(),
                                                       oc.getFloat("device.eparkingreroute.maxCharge"),
                                                       oc.getBool("device.eparkingreroute.prioritizeLowCharges"),
                                                       begIDs,
                                                       endIDs,
                                                       targetAreas,
                                                       callOnlyMode);
            into.push_back(device);
        }
    }
}

void
MSDevice_eParkingReroute::cleanup() {
    // clearing myRegisteredVehicles in order to avoid problems when reseting the simulation
    myRegisteredVehicles.clear();
}

// ---------------------------------------------------------------------------
// MSDevice_eParkingReroute-methods
// ---------------------------------------------------------------------------
MSDevice_eParkingReroute::MSDevice_eParkingReroute(SUMOVehicle& holder, const std::string& id,
                                   float maxCharge, bool prioLowCharges, std::vector<std::string> begIDs, std::vector<std::string> endIDs, std::vector<MSParkingArea*> targetAreas, bool callOnlyMode) :
    MSVehicleDevice(holder, id),
    myMaxCharge(maxCharge),
    myPrioLowCharges(prioLowCharges),
    myTargetAreas(targetAreas),
    myCallOnlyMode(callOnlyMode),
    myBegIDs(begIDs),
    myEndIDs(endIDs),
    myBaseTime(0),
    myWasRegistered(false),
    myCalledNextCandidate(false) {
        myMSVehicleHolder = dynamic_cast<MSVehicle*>(&holder);
        myBattery = dynamic_cast<MSDevice_Battery*>(myMSVehicleHolder->getDevice(typeid(MSDevice_Battery)));
}

MSDevice_eParkingReroute::~MSDevice_eParkingReroute() {
}

bool
MSDevice_eParkingReroute::compareCurrentCharges(const std::pair<MSVehicle*,MSDevice_Battery*> regVeh1, const std::pair<MSVehicle*,MSDevice_Battery*> regVeh2) {
    return (regVeh1.second->getActualBatteryCapacity() / regVeh1.second->getMaximumBatteryCapacity()) < (regVeh2.second->getActualBatteryCapacity() / regVeh2.second->getMaximumBatteryCapacity());
}

bool
MSDevice_eParkingReroute::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    if (!myWasRegistered) {
        for (auto id : myBegIDs) {
            if (tObject.getEdge()->getID() == id) {
                // do not add the vehicle if it's relative battery capacity is exceeding the max charge parameter
                if ((myBattery->getActualBatteryCapacity() / myBattery->getMaximumBatteryCapacity()) <= myMaxCharge) {
                    myRegisteredVehicles.push_back(std::make_pair(myMSVehicleHolder,myBattery));
                }
                myWasRegistered = true;
            }
        }
    }
    if (myWasRegistered) {
        for (auto id : myEndIDs) {
            if (tObject.getEdge()->getID() == id) {
                unregister(myMSVehicleHolder);
            }
        }
    }
    return true;
}

bool
MSDevice_eParkingReroute::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    // when entering the network add vehicle and battery to myRegisteredVehicles
    if (!myCallOnlyMode && !myWasRegistered && myBegIDs.size() == 0) {
        // do not add the vehicle if it's relative battery capacity is exceeding the max charge
        if ((myBattery->getActualBatteryCapacity() / myBattery->getMaximumBatteryCapacity()) <= myMaxCharge) {
            myRegisteredVehicles.push_back(std::make_pair(myMSVehicleHolder,myBattery));
        }
        myWasRegistered = true;
        //std::cout << "veh=" << myMSVehicleHolder->getID() << " was registered!" << std::endl;
    }
    return true; // keep the device
}


bool
MSDevice_eParkingReroute::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason == MSMoveReminder::NOTIFICATION_TELEPORT || reason == MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED || reason == MSMoveReminder::NOTIFICATION_ARRIVED) {
        unregister(myMSVehicleHolder);
    }
    return true; // keep the device
}


void
MSDevice_eParkingReroute::generateOutput(OutputDevice* tripinfoOut) const {

}

std::string
MSDevice_eParkingReroute::getParameter(const std::string& key) const {
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}

void
MSDevice_eParkingReroute::notifyParking() {
    if (myMSVehicleHolder->getStops().front().parkingarea->getChargingSpace(myHolder) != nullptr) {
        unregister(myMSVehicleHolder);
        if (DEBUG_COND && VERBOSE) {
            for (auto s : myMSVehicleHolder->getStops()) {
                std::cout << "  " << s.getDescription() << " at edge=" << s.lane->getEdge().getID() << std::endl;
            }
        }
        // if the vehicle is fully charged or the duration is below 1, we call a new vehicle
        if ((myMSVehicleHolder->getStops().front().duration <= 1 || myBattery->getActualBatteryCapacity() >= myBattery->getMaximumBatteryCapacity()) && !myCalledNextCandidate) {
            if (DEBUG_COND) {
                if (myMSVehicleHolder->getStops().front().duration <= 1) {
                    std::cout << SIMTIME << " veh=" << myMSVehicleHolder->getID() << " is not fully charged, but parking time expired! (current charge=" << myBattery->getActualBatteryCapacity() << ", capacity=" << myBattery->getMaximumBatteryCapacity() << std::endl;
                } else {
                    std::cout << SIMTIME << " veh=" << myMSVehicleHolder->getID() << " is fully charged! (current charge=" << myBattery->getActualBatteryCapacity() << ", capacity=" << myBattery->getMaximumBatteryCapacity() << std::endl;
                }
            }
            // get current parking area
            MSParkingArea* currentParkingArea = myMSVehicleHolder->getStops().front().parkingarea;
            // get remaining duration
            SUMOTime remainingDuration = myMSVehicleHolder->getStops().front().duration;

            // delete all empty vehicles
            for (std::vector<std::pair<MSVehicle*,MSDevice_Battery*>>::iterator v = myRegisteredVehicles.begin(); v != myRegisteredVehicles.end(); ++v) {
                if ((*v).first == nullptr || (*v).first->getID() == myMSVehicleHolder->getID()) {
                    myRegisteredVehicles.erase(v);
                    --v;
                }
            }

            // either prioritize by current charge, or it is first come first serve
            if (myPrioLowCharges) {
                std::sort(myRegisteredVehicles.begin(), myRegisteredVehicles.end(), compareCurrentCharges);
            }

            // find candidate from vector of registered vehicles
            MSVehicle* candidate = nullptr;
            for (std::vector<std::pair<MSVehicle*,MSDevice_Battery*>>::iterator v = myRegisteredVehicles.begin(); v != myRegisteredVehicles.end(); ++v) {
                // get the router of the vehicle
                SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = (*v).first->getInfluencer().getRouterTT((*v).first->getRNGIndex(), (*v).first->getVClass());
                // Compute the route edges from the current edge to the parking area edge
                ConstMSEdgeVector edgesToPark;
                router.compute(
                    (*v).first->getEdge(),
                    (*v).first->getPositionOnLane(),
                    myMSVehicleHolder->getEdge(),
                    currentParkingArea->getLastFreePos(*(*v).first),
                    (*v).first, MSNet::getInstance()->getCurrentTimeStep(), edgesToPark, true);
                // choose only a vehicle that was not called yet, that is on a normal edge and that can reach the stop edge
                if (!(*v).first->wasCalledForCharging() && !(*v).first->getLane()->getEdge().isInternal() && edgesToPark.size() > 0
                      // we only use vehicles that are parking or are en route to a parking area
                      && ((*v).first->isParking() || (*v).first->getNextParkingArea() != nullptr)) {
                    candidate = (*v).first;
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << myMSVehicleHolder->getID() << " found a candidate to call in veh=" << candidate->getID() << std::endl;
                    }
                    break;
                }
            }

            // call the next vehicle to the charging area
            std::string errorMsg = "";
            if (candidate != nullptr) {
                // get the remaining duration of the called vehicle
                SUMOTime remainingDurationRegVeh = candidate->getStops().front().duration;
                // make called vehicle leave its current stop
                candidate->setChargeCallFlag();
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << myMSVehicleHolder->getID() << " leaves, call " << candidate->getID() << std::endl;
                }
                if (DEBUG_COND && VERBOSE) {
                    std::cout << "veh=" << candidate->getID() << " edges before call:" << std::endl;
                    for (auto it = candidate->getRoute().begin(); it != candidate->getRoute().end(); ++it) {
                        if (candidate->getCurrentRouteEdge() == it) std::cout << " >";
                        else std::cout << "  ";
                        std::cout << (*it)->getID() << std::endl;
                    }
                    std::cout << "veh=" << candidate->getID() << " stops before call:" << std::endl;
                    for (auto s : candidate->getStops()) {
                        std::cout << "  " << s.getDescription() << " at edge=" << s.lane->getEdge().getID() << std::endl;
                    }
                }
                // generate parameter for new stop
                SUMOVehicleParameter::Stop parsRegVeh;
                parsRegVeh = myMSVehicleHolder->getStops().front().pars;
                parsRegVeh.index = 0;
                parsRegVeh.duration = remainingDurationRegVeh;
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << myMSVehicleHolder->getID() << " reroute candidate " << candidate->getID() << " to edge " << parsRegVeh.edge << std::endl;
                }

                if (candidate->isParking()) {
                    if (candidate->rerouteNewStop(parsRegVeh, errorMsg)) {
                        candidate->setReparkTimeStamp();
                        candidate->setCurrentStopDuration(0);
                    } else {
                        WRITE_ERROR("rerouteNewStop: " + errorMsg);
                    }
                } else if (candidate->getNextParkingArea() != nullptr) {
                    if (candidate->getNextParkingArea()->getID() != parsRegVeh.parkingarea) {
                        if (!candidate->rerouteParkingArea(parsRegVeh.parkingarea, errorMsg)) {
                            WRITE_ERROR("rerouteParkingArea: " + errorMsg);
                        }
                    } else {
                        if (DEBUG_COND) {
                            std::cout << "veh=" << candidate->getID() << " is already going to parking area " << parsRegVeh.parkingarea << std::endl;
                        }
                    }
                } else {
                    WRITE_WARNING(toString(SIMTIME) + " Cannot reroute veh=" + myMSVehicleHolder->getID() + " since it is not parking nor en route to a parking area! " + errorMsg);
                }

                // debug print
                if (DEBUG_COND && VERBOSE) {
                    std::cout << "veh=" << candidate->getID() << " edges after call:" << std::endl;
                    for (auto it = candidate->getRoute().begin(); it != candidate->getRoute().end(); ++it) {
                        if (candidate->getCurrentRouteEdge() == it) std::cout << " >";
                        else std::cout << "  ";
                        std::cout << (*it)->getID() << std::endl;
                    }
                    std::cout << "veh=" << candidate->getID() << " stops after call:" << std::endl;
                    for (auto s : candidate->getStops()) {
                        std::cout << "  " << s.getDescription() << " at edge=" << s.lane->getEdge().getID() << std::endl;
                    }
                }
            }

            if(!myCallOnlyMode) {
                // if the remaining duration is more than 0 search for the nearest target area
                // if the parking duration of the original parking area is already expired, route to the next stop as per normal!
                if (remainingDuration > 0) {
                    if (myTargetAreas.size() == 0) {
                        WRITE_ERROR("No target areas for eParkingReroute device of vehicle '" + myMSVehicleHolder->getID() + "' were given!");
                        return ;
                    }

                    // find the nearest target area
                    int nearestID = -1;
                    double minDist = libsumo::INVALID_DOUBLE_VALUE;
                    for (int i = 0; i < myTargetAreas.size(); ++i) {
                        if (minDist == libsumo::INVALID_DOUBLE_VALUE || (*myMSVehicleHolder->getStops().front().edge)->getDistanceTo(&myTargetAreas.at(i)->getLane().getEdge()) < minDist) {
                            nearestID = i;
                            minDist = (*myMSVehicleHolder->getStops().front().edge)->getDistanceTo(&myTargetAreas.at(i)->getLane().getEdge());
                        }
                    }

                    // make vehicle leave parking area
                    SUMOVehicleParameter::Stop pars;
                    pars.busstop = "";
                    pars.containerstop = "";
                    pars.parkingarea = myTargetAreas.at(nearestID)->getID();
                    pars.chargingStation = "";
                    pars.overheadWireSegment = "";
                    pars.triggered = myMSVehicleHolder->getStops().front().triggered;
                    pars.containerTriggered = false;
                    pars.joinTriggered = false;
                    pars.parking = true;
                    pars.lane = myTargetAreas.at(nearestID)->getLane().getID();
                    pars.edge = myTargetAreas.at(nearestID)->getLane().getEdge().getID();
                    pars.duration = remainingDuration;
                    errorMsg = "";
                    if (myMSVehicleHolder->rerouteNewStop(pars, errorMsg)) {
                        myMSVehicleHolder->setReparkTimeStamp();
                        myMSVehicleHolder->setCurrentStopDuration(0);
                    } else {
                        WRITE_ERROR(errorMsg);
                    }

                    // debug print
                    if (DEBUG_COND && VERBOSE) {
                        std::cout << "veh=" << myMSVehicleHolder->getID() << " edges after reroute:" << std::endl;
                        for (auto it = myMSVehicleHolder->getRoute().begin(); it != myMSVehicleHolder->getRoute().end(); ++it) {
                            if (myMSVehicleHolder->getCurrentRouteEdge() == it) std::cout << " >";
                            else std::cout << "  ";
                            std::cout << (*it)->getID() << std::endl;
                        }
                        std::cout << "veh=" << myMSVehicleHolder->getID() << " stops after reroute:" << std::endl;
                        for (auto s : myMSVehicleHolder->getStops()) {
                            std::cout << "  " << s.getDescription() << " at edge=" << s.lane->getEdge().getID() << std::endl;
                        }
                    }
                }
            }

            myCalledNextCandidate = true;
        }
    }
}

void
MSDevice_eParkingReroute::unregister(MSVehicle* veh) {
    for (std::vector<std::pair<MSVehicle*,MSDevice_Battery*>>::iterator v = myRegisteredVehicles.begin(); v != myRegisteredVehicles.end(); ++v) {
        if ((*v).first == veh) {
            myRegisteredVehicles.erase(v);
            break;
        }
    }
}

/****************************************************************************/
