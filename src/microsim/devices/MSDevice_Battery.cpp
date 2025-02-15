/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2013-2022 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Battery.cpp
/// @author  Tamas Kurczveil
/// @author  Pablo Alvarez Lopez
/// @date    20.12.2013
///
// The Battery parameters for the vehicle
/****************************************************************************/
#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/SUMOTime.h>
#include <utils/geom/GeomHelper.h>
#include <utils/emissions/HelpersEnergy.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
// (chs): add MSStop and MSParkingArea in order to interact with the stop the charging space is on
#include <microsim/MSStop.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Battery.h"

#define DEFAULT_MAX_CAPACITY 35000
#define DEFAULT_CHARGE_RATIO 0.5


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Battery::insertOptions(OptionsCont& oc) {
    insertDefaultAssignmentOptions("battery", "Battery", oc);
    // custom options
    oc.doRegister("device.battery.track-fuel", new Option_Bool(false));
    oc.addDescription("device.battery.track-fuel", "Battery", "Track fuel consumption for non-electric vehicles");
}


void
MSDevice_Battery::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    // Check if vehicle should get a battery
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "battery", v, false)) {
        const SUMOVTypeParameter& typeParams = v.getVehicleType().getParameter();
        // obtain maximumBatteryCapacity
        const double maximumBatteryCapacity = typeParams.getDouble(toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY), DEFAULT_MAX_CAPACITY);

        // obtain actualBatteryCapacity
        double actualBatteryCapacity = 0;
        if (v.getParameter().getParameter(toString(SUMO_ATTR_ACTUALBATTERYCAPACITY), "-") == "-") {
            actualBatteryCapacity = typeParams.getDouble(toString(SUMO_ATTR_ACTUALBATTERYCAPACITY),
                                    maximumBatteryCapacity * DEFAULT_CHARGE_RATIO);
        } else {
            actualBatteryCapacity = StringUtils::toDouble(v.getParameter().getParameter(toString(SUMO_ATTR_ACTUALBATTERYCAPACITY), "0"));
        }

        const double powerMax = typeParams.getDouble(toString(SUMO_ATTR_MAXIMUMPOWER), 100.);
        const double stoppingTreshold = typeParams.getDouble(toString(SUMO_ATTR_STOPPINGTRESHOLD), 0.1);
        EnergyParams params(&v.getVehicleType().getParameter());

        // battery constructor
        MSDevice_Battery* device = new MSDevice_Battery(v, "battery_" + v.getID(),
                actualBatteryCapacity, maximumBatteryCapacity, powerMax, stoppingTreshold, params);

        // Add device to vehicle
        into.push_back(device);
    }
}


bool MSDevice_Battery::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */, double /* newPos */, double /* newSpeed */) {
    if (!tObject.isVehicle()) {
        return false;
    }
    SUMOVehicle& veh = static_cast<SUMOVehicle&>(tObject);
    // Start vehicleStoppedTimer if the vehicle is stopped. In other case reset timer
    if (veh.getSpeed() < myStoppingTreshold) {
        // Increase vehicle stopped timer
        increaseVehicleStoppedTimer();
    } else {
        // Reset vehicle Stopped
        resetVehicleStoppedTimer();
    }

    // Update Energy from the battery
    if (getMaximumBatteryCapacity() != 0) {
        myParam.setDouble(SUMO_ATTR_ANGLE, myLastAngle == std::numeric_limits<double>::infinity() ? 0. : GeomHelper::angleDiff(myLastAngle, veh.getAngle()));
        if (myTrackFuel) {
            // [ml]
            myConsum = PollutantsInterface::compute(veh.getVehicleType().getEmissionClass(), PollutantsInterface::FUEL, veh.getSpeed(), veh.getAcceleration(), veh.getSlope(), &myParam) * TS;
        } else {
            // [Wh]
            myConsum = PollutantsInterface::getEnergyHelper().compute(0, PollutantsInterface::ELEC, veh.getSpeed(), veh.getAcceleration(), veh.getSlope(), &myParam) * TS;
        }
        if (veh.isParking()) {
            // recuperation from last braking step is ok but further consumption should cease
            myConsum = MIN2(myConsum, 0.0);
        }

        // Energy lost/gained from vehicle movement (via vehicle energy model) [Wh]
        setActualBatteryCapacity(getActualBatteryCapacity() - myConsum);

        // Track total energy consumption and regeneration
        if (myConsum > 0.0) {
            myTotalConsumption += myConsum;
        } else {
            myTotalRegenerated -= myConsum;
        }

        // saturate between 0 and myMaximumBatteryCapacity [Wh]
        if (getActualBatteryCapacity() < 0) {
            setActualBatteryCapacity(0);
            if (getMaximumBatteryCapacity() > 0) {
                WRITE_WARNING("Battery of vehicle '" + veh.getID() + "' is depleted.")
            }
        } else if (getActualBatteryCapacity() > getMaximumBatteryCapacity()) {
            setActualBatteryCapacity(getMaximumBatteryCapacity());
        }
        myLastAngle = veh.getAngle();
    }

    // Check if vehicle has under their position one charge Station
    const std::string chargingStationID = MSNet::getInstance()->getStoppingPlaceID(veh.getLane(), veh.getPositionOnLane(), SUMO_TAG_CHARGING_STATION);
    // (chs): Check if vehicle is parking on a charging space
    // is vehicle parking AND the veh is currently on a charging space
    bool isOnChargingSpace = veh.isParking() && veh.getStops().front().parkingarea->getChargingSpace(veh) != nullptr;

    // If vehicle is over a charging station
    if (chargingStationID != "") {
        // if the vehicle is almost stopped, or charge in transit is enabled, then charge vehicle
        MSChargingStation* const cs = static_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(chargingStationID, SUMO_TAG_CHARGING_STATION));
        if ((veh.getSpeed() < myStoppingTreshold) || cs->getChargeInTransit()) {
            // Set Flags Stopped/intransit to
            if (veh.getSpeed() < myStoppingTreshold) {
                // vehicle ist almost stopped, then is charging stopped
                myChargingStopped = true;

                // therefore isn't charging in transit
                myChargingInTransit = false;
            } else {
                // vehicle is moving, and the Charging station allow charge in transit
                myChargingStopped = false;

                // Therefore charge in transit
                myChargingInTransit = true;
            }

            // get pointer to charging station
            myActChargingStation = cs;

            // Only update charging start time if vehicle allow charge in transit, or in other case
            // if the vehicle not allow charge in transit but it's stopped.
            if ((myActChargingStation->getChargeInTransit()) || (veh.getSpeed() < myStoppingTreshold)) {
                // Update Charging start time
                increaseChargingStartTime();
            }

            // time it takes the vehicle at the station < charging station time delay?
            if (getChargingStartTime() > myActChargingStation->getChargeDelay()) {
                // Enable charging vehicle
                myActChargingStation->setChargingVehicle(true);

                // Calulate energy charged
                myEnergyCharged = myActChargingStation->getChargingPower(myTrackFuel) * myActChargingStation->getEfficency() * TS;

                if (getActualBatteryCapacity() < getMaximumBatteryCapacity()) {
                    // (pki): increase metric for total charging time, blocking time is not considered for charging lanes
                    myTotalChargingTime += DELTA_T;
                }

                // Update Battery charge
                if ((myEnergyCharged + getActualBatteryCapacity()) > getMaximumBatteryCapacity()) {
                    // (pki): increase metric for total charged energy
                    myTotalEnergyCharged += getMaximumBatteryCapacity() - getActualBatteryCapacity();
                    setActualBatteryCapacity(getMaximumBatteryCapacity());
                } else {
                    // (pki): increase metric for total charged energy
                    myTotalEnergyCharged += myEnergyCharged;
                    setActualBatteryCapacity(getActualBatteryCapacity() + myEnergyCharged);
                }
            }
            // add charge value for output to myActChargingStation
            myActChargingStation->addChargeValueForOutput(myEnergyCharged, this);
        }
        // else disable charging vehicle
        else {
            cs->setChargingVehicle(false);
        }
        // disable charging vehicle from previous (not current) ChargingStation (reason: if there is no gap between two different chargingStations = the vehicle switches from used charging station to other one in a single timestap)
        if (myPreviousNeighbouringChargingStation != nullptr && myPreviousNeighbouringChargingStation != cs) {
            myPreviousNeighbouringChargingStation->setChargingVehicle(false);
        }
        myPreviousNeighbouringChargingStation = cs;
    }
    // (chs): if the vehicle is on a charging space
    else if(isOnChargingSpace) {
        // (chs): get values from space
        myActChargingSpace = veh.getStops().front().parkingarea->getChargingSpace(veh);
        myActChargingSpaceID = myActChargingSpace->getID();
        // (chs): check if charging time exceeds the charge delay
        if (getChargingStartTime() > myActChargingSpace->getChargeDelay()) {
            //power = myTrackFuel ? power : power / 3600
            // (chs): Calulate energy charged
            myEnergyCharged = myActChargingSpace->getChargingPower(myTrackFuel) * myActChargingSpace->getEfficiency() * TS;
#ifdef DEBUG_CHARGINGSPACES
            // (chs) (dbg): debug print for charging space
            std::cout << "veh=" << veh.getID() << " is on charging space:\n"
                    << "time on space=" << time2string(getChargingStartTime()) << "\n"
                    << "myEnergyCharged=" << myEnergyCharged << "\n"
                    << "maximum capacity=" << getMaximumBatteryCapacity() << "\n"
                    << "current charge=" << getActualBatteryCapacity() << "\n"
                    << "steps to full charge=" << ceil(((getMaximumBatteryCapacity() - getActualBatteryCapacity()) / myEnergyCharged)) << "\n";
#endif
            if (getActualBatteryCapacity() < getMaximumBatteryCapacity()) {
                // (pki): increase metric for total charging time
                myTotalChargingTime += DELTA_T;
            } else {
                // (pki): if the vehicle is on a charging space but is not charging, it blocks the space for other vehicles
                myTotalBlockingTime += DELTA_T;
            }

            if ((myEnergyCharged + getActualBatteryCapacity()) > getMaximumBatteryCapacity()) {
                // (pki): increase metric for total charged energy
                myTotalEnergyCharged += getMaximumBatteryCapacity() - getActualBatteryCapacity();
                setActualBatteryCapacity(getMaximumBatteryCapacity());
            } else {
                // (pki): increase metric for total charged energy
                myTotalEnergyCharged += myEnergyCharged;
                setActualBatteryCapacity(getActualBatteryCapacity() + myEnergyCharged);
            }
        }
        // (chs): add to charging start time
        increaseChargingStartTime();
        myActChargingSpace->addChargeValueForOutput(myEnergyCharged, this);
    }
    // In other case, vehicle will be not charged
    else {
        // Disable flags
        myChargingInTransit = false;
        myChargingStopped = false;

        // Disable charging vehicle
        if (myActChargingStation != nullptr) {
            myActChargingStation->setChargingVehicle(false);
        }

        // Set charging station pointer to NULL
        myActChargingStation = nullptr;

        // Set energy charged to 0
        myEnergyCharged = 0.00;

        // Reset timer
        resetChargingStartTime();
    }
    // Always return true.
    return true;
}


// ---------------------------------------------------------------------------
// MSDevice_Battery-methods
// ---------------------------------------------------------------------------
MSDevice_Battery::MSDevice_Battery(SUMOVehicle& holder, const std::string& id, const double actualBatteryCapacity, const double maximumBatteryCapacity,
                                   const double powerMax, const double stoppingTreshold, const EnergyParams& param) :
    MSVehicleDevice(holder, id),
    myActualBatteryCapacity(0),         // [actualBatteryCapacity <= maximumBatteryCapacity]
    myMaximumBatteryCapacity(0),        // [maximumBatteryCapacity >= 0]
    myPowerMax(0),                      // [maximumPower >= 0]
    myStoppingTreshold(0),              // [stoppingTreshold >= 0]
    myParam(param),
    myLastAngle(std::numeric_limits<double>::infinity()),
    myChargingStopped(false),           // Initially vehicle don't charge stopped
    myChargingInTransit(false),         // Initially vehicle don't charge in transit
    myChargingStartTime(0),             // Initially charging start time (must be if the vehicle was launched at the charging station)
    myConsum(0),                        // Initially the vehicle is stopped and therefore the consum is zero.
    myTotalConsumption(0.0),
    myTotalRegenerated(0.0),
    myActChargingStation(nullptr),         // Initially the vehicle isn't over a Charging Station
    myPreviousNeighbouringChargingStation(nullptr),    // Initially the vehicle wasn't over a Charging Station
    myEnergyCharged(0),                 // Initially the energy charged is zero
    // (pki): init new parameters for parking info
    myTotalChargingTime(0),
    myTotalEnergyCharged(0),
    myTotalBlockingTime(0),
    myVehicleStopped(0) {  // Initially the vehicle is stopped and the corresponding variable is 0

    if (maximumBatteryCapacity < 0) {
        WRITE_WARNING("Battery builder: Vehicle '" + getID() + "' doesn't have a valid value for parameter " + toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY) + " (" + toString(maximumBatteryCapacity) + ").")
    } else {
        myMaximumBatteryCapacity = maximumBatteryCapacity;
    }

    if (actualBatteryCapacity > maximumBatteryCapacity) {
        WRITE_WARNING("Battery builder: Vehicle '" + getID() + "' has a " + toString(SUMO_ATTR_ACTUALBATTERYCAPACITY) + " ("  + toString(actualBatteryCapacity) + ") greater than it's " + toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY) + " (" + toString(maximumBatteryCapacity) + "). A max battery capacity value will be asigned");
        myActualBatteryCapacity = myMaximumBatteryCapacity;
    } else {
        myActualBatteryCapacity = actualBatteryCapacity;
    }

    if (powerMax < 0) {
        WRITE_WARNING("Battery builder: Vehicle '" + getID() + "' doesn't have a valid value for parameter " + toString(SUMO_ATTR_MAXIMUMPOWER) + " (" + toString(powerMax) + ").")
    } else {
        myPowerMax = powerMax;
    }

    if (stoppingTreshold < 0) {
        WRITE_WARNING("Battery builder: Vehicle '" + getID() + "' doesn't have a valid value for parameter " + toString(SUMO_ATTR_STOPPINGTRESHOLD) + " (" + toString(stoppingTreshold) + ").")
    } else {
        myStoppingTreshold = stoppingTreshold;
    }

    checkParam(SUMO_ATTR_VEHICLEMASS);
    checkParam(SUMO_ATTR_FRONTSURFACEAREA);
    checkParam(SUMO_ATTR_AIRDRAGCOEFFICIENT);
    checkParam(SUMO_ATTR_INTERNALMOMENTOFINERTIA);
    checkParam(SUMO_ATTR_RADIALDRAGCOEFFICIENT);
    checkParam(SUMO_ATTR_ROLLDRAGCOEFFICIENT);
    checkParam(SUMO_ATTR_CONSTANTPOWERINTAKE);
    checkParam(SUMO_ATTR_PROPULSIONEFFICIENCY);
    checkParam(SUMO_ATTR_RECUPERATIONEFFICIENCY);
    checkParam(SUMO_ATTR_RECUPERATIONEFFICIENCY_BY_DECELERATION);

    myTrackFuel = !PollutantsInterface::getEnergyHelper().includesClass(holder.getVehicleType().getEmissionClass()) && OptionsCont::getOptions().getBool("device.battery.track-fuel");
}


MSDevice_Battery::~MSDevice_Battery() {
}


void
MSDevice_Battery::checkParam(const SumoXMLAttr paramKey, const double lower, const double upper) {
    if (!myParam.knowsParameter(paramKey)
            || myParam.getDouble(paramKey) < lower
            || myParam.getDouble(paramKey) > upper) {
        WRITE_WARNING("Battery builder: Vehicle '" + getID() + "' doesn't have a valid value for parameter " + toString(paramKey) + " (" + toString(myParam.getDouble(paramKey)) + ").");
        myParam.setDouble(paramKey, PollutantsInterface::getEnergyHelper().getDefaultParam(paramKey));
    }
}


void
MSDevice_Battery::setActualBatteryCapacity(const double actualBatteryCapacity) {
    if (actualBatteryCapacity < 0) {
        myActualBatteryCapacity = 0;
    } else if (actualBatteryCapacity > myMaximumBatteryCapacity) {
        myActualBatteryCapacity = myMaximumBatteryCapacity;
    } else {
        myActualBatteryCapacity = actualBatteryCapacity;
    }
}


void
MSDevice_Battery::setMaximumBatteryCapacity(const double maximumBatteryCapacity) {
    if (myMaximumBatteryCapacity < 0) {
        WRITE_WARNING("Trying to set into the battery device of vehicle '" + getID() + "' an invalid " + toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY) + " (" + toString(maximumBatteryCapacity) + ").")
    } else {
        myMaximumBatteryCapacity = maximumBatteryCapacity;
    }
}


void
MSDevice_Battery::setPowerMax(const double powerMax) {
    if (myPowerMax < 0) {
        WRITE_WARNING("Trying to set into the battery device of vehicle '" + getID() + "' an invalid " + toString(SUMO_ATTR_MAXIMUMPOWER) + " (" + toString(powerMax) + ").")
    } else {
        myPowerMax = powerMax;
    }
}


void
MSDevice_Battery::setStoppingTreshold(const double stoppingTreshold) {
    if (stoppingTreshold < 0) {
        WRITE_WARNING("Trying to set into the battery device of vehicle '" + getID() + "' an invalid " + toString(SUMO_ATTR_STOPPINGTRESHOLD) + " (" + toString(stoppingTreshold) + ").")
    } else {
        myStoppingTreshold = stoppingTreshold;
    }
}


void
MSDevice_Battery::resetChargingStartTime() {
    myChargingStartTime = 0;
}


void
MSDevice_Battery::increaseChargingStartTime() {
    myChargingStartTime += DELTA_T;
}


void
MSDevice_Battery::resetVehicleStoppedTimer() {
    myVehicleStopped = 0;
}


void
MSDevice_Battery::increaseVehicleStoppedTimer() {
    myVehicleStopped++;
}


double
MSDevice_Battery::getActualBatteryCapacity() const {
    return myActualBatteryCapacity;
}


double
MSDevice_Battery::getMaximumBatteryCapacity() const {
    return myMaximumBatteryCapacity;
}


double
MSDevice_Battery::getMaximumPower() const {
    return myPowerMax;
}


double
MSDevice_Battery::getConsum() const {
    return myConsum;
}

double
MSDevice_Battery::getTotalConsumption() const {
    return myTotalConsumption;
}


double
MSDevice_Battery::getTotalRegenerated() const {
    return myTotalRegenerated;
}


bool
MSDevice_Battery::isChargingStopped() const {
    return myChargingStopped;
}


bool
MSDevice_Battery::isChargingInTransit() const {
    return myChargingInTransit;
}


SUMOTime
MSDevice_Battery::getChargingStartTime() const {
    return myChargingStartTime;
}


std::string
MSDevice_Battery::getChargingStationID() const {
    // (chs): return charging space id
    if(myActChargingSpace != nullptr) {
        return myActChargingSpaceID;
    } else if (myActChargingStation != nullptr) {
        return myActChargingStation->getID();
    } else {
        return "NULL";
    }
}

double
MSDevice_Battery::getEnergyCharged() const {
    return myEnergyCharged;
}


int
MSDevice_Battery::getVehicleStopped() const {
    return myVehicleStopped;
}


double
MSDevice_Battery::getStoppingTreshold() const {
    return myStoppingTreshold;
}


std::string
MSDevice_Battery::getParameter(const std::string& key) const {
    if (key == toString(SUMO_ATTR_ACTUALBATTERYCAPACITY)) {
        return toString(getActualBatteryCapacity());
    } else if (key == toString(SUMO_ATTR_ENERGYCONSUMED)) {
        return toString(getConsum());
    } else if (key == toString(SUMO_ATTR_TOTALENERGYCONSUMED)) {
        return toString(getTotalConsumption());
    } else if (key == toString(SUMO_ATTR_TOTALENERGYREGENERATED)) {
        return toString(getTotalRegenerated());
    } else if (key == toString(SUMO_ATTR_ENERGYCHARGED)) {
        return toString(getEnergyCharged());
    } else if (key == toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY)) {
        return toString(getMaximumBatteryCapacity());
    } else if (key == toString(SUMO_ATTR_CHARGINGSTATIONID)) {
        return getChargingStationID();
    } else if (key == toString(SUMO_ATTR_VEHICLEMASS)) {
        return toString(myParam.getDouble(SUMO_ATTR_VEHICLEMASS));
    // (pki): add new metrics to parameter getter
    } else if (key == toString(SUMO_ATTR_TOTALCHARGINGTIME)) {
        return time2string(myTotalChargingTime);
    } else if (key == toString(SUMO_ATTR_TOTALENERGYCHARGED)) {
        return toString(myTotalEnergyCharged);
    } else if (key == toString(SUMO_ATTR_TOTALBLOCKINGTIME)) {
        return time2string(myTotalBlockingTime);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_Battery::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == toString(SUMO_ATTR_ACTUALBATTERYCAPACITY)) {
        setActualBatteryCapacity(doubleValue);
    } else if (key == toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY)) {
        setMaximumBatteryCapacity(doubleValue);
    } else if (key == toString(SUMO_ATTR_VEHICLEMASS)) {
        myParam.setDouble(SUMO_ATTR_VEHICLEMASS, doubleValue);
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


void
MSDevice_Battery::notifyParking() {
    // @note: only charing is performed but no energy is consumed
    notifyMove(myHolder, myHolder.getPositionOnLane(), myHolder.getPositionOnLane(), myHolder.getSpeed());
    myConsum = 0;
}

// (pki): getter for myTotalChargingTime
SUMOTime
MSDevice_Battery::getTotalChargingTime() const {
    return myTotalChargingTime;
}

// (pki): getter for myTotalBlockingTime
SUMOTime
MSDevice_Battery::getTotalBlockingTime() const {
    return myTotalBlockingTime;
}

// (pki): getter for myTotalEnergyCharged
double
MSDevice_Battery::getTotalEnergyCharged() const {
    return myTotalEnergyCharged;
}
/****************************************************************************/
