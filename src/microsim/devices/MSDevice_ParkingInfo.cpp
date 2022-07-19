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
/// @file    MSDevice_ParkingInfo.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    11.06.2013
///
// A device which stands as an implementation parkinginfo and which outputs movereminder calls
/****************************************************************************/
#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSStop.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Battery.h"
#include "MSDevice_ParkingInfo.h"

//#define DEBUG_PI
//#define DEBUG_COND(obj) (obj->getID() == "id of holder")
#define DEBUG_COND(obj) (false);

// ===========================================================================
// (pki): method definitions for parking info device
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_ParkingInfo::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("ParkingInfo Device");
    insertDefaultAssignmentOptions("parkinginfo", "ParkingInfo Device", oc);

    oc.doRegister("device.parkinginfo.begEdge", new Option_String());
    oc.doRegister("device.parkinginfo.endEdge", new Option_String());
    oc.addDescription("device.parkinginfo.begEdge", "ParkingInfo Device", "Edge at which measuring starts.");
    oc.addDescription("device.parkinginfo.endEdge", "ParkingInfo Device", "Edge at which measuring ends.");
}


void
MSDevice_ParkingInfo::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    std::vector<std::string> begIDs;
    std::vector<std::string> endIDs;
    std::string begIDstr = oc.getString("device.parkinginfo.begEdge");
    StringTokenizer stBeg(begIDstr, " ", true);
    while (stBeg.hasNext()) {
        begIDs.push_back(StringUtils::prune(stBeg.next()));
    }
    std::string endIDstr = oc.getString("device.parkinginfo.endEdge");
    StringTokenizer stEnd(endIDstr, " ", true);
    while (stEnd.hasNext()) {
        endIDs.push_back(StringUtils::prune(stEnd.next()));
    }
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "ParkingInfo", v, false)) {
        MSDevice_ParkingInfo* device = new MSDevice_ParkingInfo(v, "parkinginfo_" + v.getID(), begIDs, endIDs);
        into.push_back(device);
    }
}

void
MSDevice_ParkingInfo::cleanup() {
    // cleaning up global state (if any)
}

// ---------------------------------------------------------------------------
// MSDevice_ParkingInfo-methods
// ---------------------------------------------------------------------------
MSDevice_ParkingInfo::MSDevice_ParkingInfo(SUMOVehicle& holder, const std::string& id,
                                   std::vector<std::string> begIDs, std::vector<std::string> endIDs) :
    MSVehicleDevice(holder, id),
    myBegIDs(begIDs),
    myEndIDs(endIDs),
    myBegID(""),
    myEndID(""),
    myEnteringTime(-1),
    myLeavingTime(0),
    myParkingDuration(0),
    myReparkingTime(0),
    myBaseTime(0),
    myIsWithinBounds(false),
    myReachedParkingArea(false) {
      myMSHolder = dynamic_cast<MSVehicle*>(&myHolder);
}

MSDevice_ParkingInfo::~MSDevice_ParkingInfo() {
}


bool
MSDevice_ParkingInfo::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */,
                             double /* newPos */, double newSpeed) {
    // start measurements when reaching a beginning edge
    for (auto id : myBegIDs) {
        if (!myIsWithinBounds && tObject.getEdge()->getID() == id) {
            myBegID = id;
            myIsWithinBounds = true;
            myBaseTime = MSNet::getInstance()->getCurrentTimeStep();
            if (myMSHolder->hasDevice("battery")) {
                MSDevice_Battery* battery = static_cast<MSDevice_Battery*>(myMSHolder->getDevice(typeid(MSDevice_Battery)));
                myBaseTotalEnergyCharged = battery->getTotalEnergyCharged();
                myBaseTotalChargingTime = battery->getTotalChargingTime();
                myBaseTotalBlockingTime = battery->getTotalBlockingTime();
            }
#ifdef DEBUG_PI
            if (DEBUG_COND(myHolder.getID())) {
                    std::cout << SIMTIME << " veh=" << toString(myHolder.getID()) << " reaches start edge=" << myBegID << " at time=" << toString(myBaseTime) << std::endl;
            }
#endif
        }
    }
    if (myIsWithinBounds) {
        // MEASURE
        SUMOVehicle& veh = static_cast<SUMOVehicle&>(tObject);
        if (!veh.isParking() && myReachedParkingArea) {
            if (noParkingStopsBeforeEndEdge(tObject)) {
                myLeavingTime += DELTA_T;
            } else {
                myReparkingTime += DELTA_T;
            }
        }
        // stop measurements when reaching an end edge
        for (auto id : myEndIDs) {
            if (tObject.getEdge()->getID() == id) {
                myEndID = id;
                myIsWithinBounds = false;
                myEndTime = MSNet::getInstance()->getCurrentTimeStep();
                if (myMSHolder->hasDevice("battery")) {
                    MSDevice_Battery* battery = static_cast<MSDevice_Battery*>(myMSHolder->getDevice(typeid(MSDevice_Battery)));
                    myTotalEnergyCharged = battery->getTotalEnergyCharged() - myBaseTotalEnergyCharged;
                    myTotalChargingTime = battery->getTotalChargingTime() - myBaseTotalChargingTime;
                    myTotalBlockingTime = battery->getTotalBlockingTime() - myBaseTotalBlockingTime;
                }
#ifdef DEBUG_PI
                if (DEBUG_COND(myHolder.getID())) {
                        std::cout << SIMTIME << " veh=" << toString(myHolder.getID()) << " reaches end edge=" << myEndID << " at time=" << toString(myEndTime) << std::endl;
                }
#endif
            }
        }
        if (myEndIDs.size() == 0 && std::prev(myMSHolder->getRoute().end()) == myMSHolder->getCurrentRouteEdge()) {
            myEndID = (*myMSHolder->getCurrentRouteEdge())->getID();
            myEndTime = MSNet::getInstance()->getCurrentTimeStep();
            if (myMSHolder->hasDevice("battery")) {
                MSDevice_Battery* battery = static_cast<MSDevice_Battery*>(myMSHolder->getDevice(typeid(MSDevice_Battery)));
                myTotalEnergyCharged = battery->getTotalEnergyCharged() - myBaseTotalEnergyCharged;
                myTotalChargingTime = battery->getTotalChargingTime() - myBaseTotalChargingTime;
                myTotalBlockingTime = battery->getTotalBlockingTime() - myBaseTotalBlockingTime;
            }
        }
    }
    return true; // keep the device
}

bool
MSDevice_ParkingInfo::noParkingStopsBeforeEndEdge(SUMOTrafficObject& tObject) const {
    MSVehicle& veh = static_cast<MSVehicle&>(tObject);
    auto nextStop = veh.getStops().begin();
    // get iterator of the next end edge
    MSRouteIterator endEdgeIt = veh.getCurrentRouteEdge();
    while (endEdgeIt != veh.getRoute().end()) {
        for (auto id : myEndIDs) {
            if ((*endEdgeIt)->getID() == id) {
                goto endEdgeItFound;
            }
        }
        endEdgeIt++;
    }
    endEdgeItFound:;
    // check if we should check stops beginning from the next stop or the stop after
    if (veh.isParking() && veh.getStops().front().duration <= 0) {
        nextStop++;
    }
    // search stops until we are past the endEdge or until we find a parkingarea
    while (nextStop != veh.getStops().end()) {
        if ((*nextStop).edge >= endEdgeIt) {
            //std::cout << "veh=" << veh.getID() << " stop edge " << (*(*nextStop).edge)->getID() << " lies beyond or equals endEdge " << (*endEdgeIt)->getID() << std::endl;
            return true;
        }
        if ((*nextStop).parkingarea != nullptr) {
            //std::cout << "veh=" << veh.getID() << " stop is a parking area!" << std::endl;
            return false;
        }
        nextStop++;
    }
    return true;
}

bool
MSDevice_ParkingInfo::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (!myIsWithinBounds && myBegIDs.size() == 0) {
        myBegID = (*myMSHolder->getCurrentRouteEdge())->getID();
        myIsWithinBounds = true;
    }
    return true; // keep the device
}


bool
MSDevice_ParkingInfo::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    return true; // keep the device
}


void
MSDevice_ParkingInfo::generateOutput(OutputDevice* tripinfoOut) const {
    if (tripinfoOut != nullptr) {
        tripinfoOut->openTag("parkinginfo");
        if (myBegID != "") {
            tripinfoOut->writeAttr("from", myBegID);
        }
        if (myEndID != "") {
            tripinfoOut->writeAttr("to", myEndID);
        }
        tripinfoOut->writeAttr("begTime", time2string(myBaseTime));
        tripinfoOut->writeAttr("endTime", time2string(myEndTime));
        tripinfoOut->writeAttr("enteringTime", time2string(myEnteringTime));
        tripinfoOut->writeAttr("leavingTime", time2string(myLeavingTime));
        tripinfoOut->writeAttr("parkingDuration", time2string(myParkingDuration));
        tripinfoOut->writeAttr("reparkingTime", time2string(myReparkingTime));
        if (myMSHolder->hasDevice("battery")) {
            tripinfoOut->writeAttr("totalEnergyCharged", toString(myTotalEnergyCharged));
            tripinfoOut->writeAttr("totalChargingTime", time2string(myTotalChargingTime));
            tripinfoOut->writeAttr("totalBlockingTime", time2string(myTotalBlockingTime));
        }
        tripinfoOut->closeTag();
    }
}

std::string
MSDevice_ParkingInfo::getParameter(const std::string& key) const {
    if (key == "enteringTime") {
        return time2string(myEnteringTime);
    } else if (key == "leavingTime") {
        return time2string(myLeavingTime);
    } else if (key == "parkingDuration") {
        return time2string(myParkingDuration);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}

void
MSDevice_ParkingInfo::notifyParking() {
    if (myIsWithinBounds) {
        // increase parking duration
        myParkingDuration += DELTA_T;
        if (myEnteringTime < 0) {
            // ... set entering time and flag that the parking area was reached
            myEnteringTime = MSNet::getInstance()->getCurrentTimeStep() - myBaseTime;
            myReachedParkingArea = true;
        }
        MSVehicle& veh = static_cast<MSVehicle&>(myHolder);
        if (veh.getStops().front().duration <= 0 && myReachedParkingArea && noParkingStopsBeforeEndEdge(myHolder)) {
            myLeavingTime += DELTA_T;
        }
    }
}

/****************************************************************************/
