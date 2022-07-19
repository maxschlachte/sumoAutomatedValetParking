/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2021 German Aerospace Center (DLR) and others.
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
/// @file    MSChargingSpace.cpp
/// @author  Daniel Krajzewicz
/// @author  Tamas Kurczveil
/// @author  Pablo Alvarez Lopez
/// @date    22-02-02
///
// Charging Space for Electric vehicles
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utils/common/StringUtils.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSVehicleType.h>
#include <microsim/devices/MSDevice_Battery.h>
#include <microsim/MSNet.h>
#include "MSChargingSpace.h"
#include "MSTrigger.h"


// ===========================================================================
// member method definitions
// ===========================================================================

MSChargingSpace::MSChargingSpace(const std::string id, double chargingPower, double efficiency, SUMOTime chargeDelay) :
    Named(id),
    myChargingPower(0),
    myEfficiency(0),
    myChargeDelay(0),
    myChargingVehicle(false),
    myTotalCharge(0) {

    if (chargingPower < 0) {
        WRITE_WARNING("Parameter " + toString(SUMO_ATTR_CHARGINGPOWER) + " for " + toString(SUMO_TAG_CHARGING_STATION) + " is invalid (" + toString(chargingPower) + ").");
    } else {
        myChargingPower = chargingPower;
    }

    if (efficiency < 0 || efficiency > 1) {
        WRITE_WARNING("Parameter " + toString(SUMO_ATTR_EFFICIENCY) + " for " + toString(SUMO_TAG_CHARGING_STATION) + " is invalid (" + toString(getEfficiency()) + ").");
    } else {
        myEfficiency = efficiency;
    }

    if (chargeDelay < 0) {
        WRITE_WARNING("Parameter " + toString(SUMO_ATTR_CHARGEDELAY) + " for " + toString(SUMO_TAG_CHARGING_STATION) + " is invalid (" + toString(getChargeDelay()) + ").");
    } else {
        myChargeDelay = chargeDelay;
    }
}


MSChargingSpace::~MSChargingSpace() {
}


double
MSChargingSpace::getChargingPower(bool usingFuel) const {
    if (usingFuel) {
        return myChargingPower;
    } else {
        // Convert from [Ws] to [Wh] (3600s / 1h):
        return myChargingPower / 3600;
    }
}


double
MSChargingSpace::getEfficiency() const {
    return myEfficiency;
}


SUMOTime
MSChargingSpace::getChargeDelay() const {
    return myChargeDelay;
}


void
MSChargingSpace::setChargingVehicle(bool value) {
    myChargingVehicle = value;
}


bool
MSChargingSpace::isCharging() const {
    return myChargingVehicle;
}


void
MSChargingSpace::addChargeValueForOutput(double WCharged, MSDevice_Battery* battery) {
    std::string status = "";
    if (battery->getChargingStartTime() > myChargeDelay) {
        if (battery->getHolder().getSpeed() < battery->getStoppingTreshold()) {
            status = "chargingStopped";
        } else {
            status = "noCharging";
        }
    } else {
        if (battery->getHolder().getSpeed() < battery->getStoppingTreshold()) {
            status = "waitingChargeStopped";
        } else {
            status = "noWaitingCharge";
        }
    }
    // update total charge
    myTotalCharge += WCharged;
    // create charge row and insert it in myChargeValues
    const std::string vehID = battery->getHolder().getID();
    if (myChargeValues.count(vehID) == 0) {
        myChargedVehicles.push_back(vehID);
    }
    Charge C(MSNet::getInstance()->getCurrentTimeStep(), vehID, battery->getHolder().getVehicleType().getID(),
             status, WCharged, battery->getActualBatteryCapacity(), battery->getMaximumBatteryCapacity(),
             myChargingPower, myEfficiency, myTotalCharge);
    myChargeValues[vehID].push_back(C);
}


void
MSChargingSpace::writeChargingStationOutput(OutputDevice& output) {
    int chargingSteps = 0;
    for (const auto& item : myChargeValues) {
        chargingSteps += (int)item.second.size();
    }
    output.openTag(SUMO_TAG_CHARGING_STATION);
    output.writeAttr(SUMO_ATTR_ID, myID);
    output.writeAttr(SUMO_ATTR_TOTALENERGYCHARGED, myTotalCharge);
    output.writeAttr(SUMO_ATTR_CHARGINGSTEPS, chargingSteps);
    // start writting
    if (myChargeValues.size() > 0) {
        for (const std::string& vehID : myChargedVehicles) {
            int iStart = 0;
            const auto& chargeSteps = myChargeValues[vehID];
            while (iStart < (int)chargeSteps.size()) {
                int iEnd = iStart + 1;
                double charged = chargeSteps[iStart].WCharged;
                while (iEnd < (int)chargeSteps.size() && chargeSteps[iEnd].timeStep == chargeSteps[iEnd - 1].timeStep + DELTA_T) {
                    charged += chargeSteps[iEnd].WCharged;
                    iEnd++;
                }
                writeVehicle(output, chargeSteps, iStart, iEnd, charged);
                iStart = iEnd;
            }
        }
    }
    // close charging station tag
    output.closeTag();
}

void
MSChargingSpace::writeVehicle(OutputDevice& out, const std::vector<Charge>& chargeSteps, int iStart, int iEnd, double charged) {
    const Charge& first = chargeSteps[iStart];
    out.openTag(SUMO_TAG_VEHICLE);
    out.writeAttr(SUMO_ATTR_ID, first.vehicleID);
    out.writeAttr(SUMO_ATTR_TYPE, first.vehicleType);
    out.writeAttr(SUMO_ATTR_TOTALENERGYCHARGED_VEHICLE, charged);
    out.writeAttr(SUMO_ATTR_CHARGINGBEGIN, time2string(first.timeStep));
    out.writeAttr(SUMO_ATTR_CHARGINGEND, time2string(chargeSteps[iEnd - 1].timeStep));
    for (int i = iStart; i < iEnd; i++) {
        const Charge& c = chargeSteps[i];
        out.openTag(SUMO_TAG_STEP);
        out.writeAttr(SUMO_ATTR_TIME, time2string(c.timeStep));
        // charge values
        out.writeAttr(SUMO_ATTR_CHARGING_STATUS, c.status);
        out.writeAttr(SUMO_ATTR_ENERGYCHARGED, c.WCharged);
        out.writeAttr(SUMO_ATTR_PARTIALCHARGE, c.totalEnergyCharged);
        // charging values of charging station in this timestep
        out.writeAttr(SUMO_ATTR_CHARGINGPOWER, c.chargingPower);
        out.writeAttr(SUMO_ATTR_EFFICIENCY, c.chargingEfficiency);
        // battery status of vehicle
        out.writeAttr(SUMO_ATTR_ACTUALBATTERYCAPACITY, c.actualBatteryCapacity);
        out.writeAttr(SUMO_ATTR_MAXIMUMBATTERYCAPACITY, c.maxBatteryCapacity);
        // close tag timestep
        out.closeTag();
    }
    out.closeTag();
}


/****************************************************************************/
