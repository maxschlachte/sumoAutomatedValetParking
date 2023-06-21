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
/// @file    MSDevice_ParkingInfo.h
/// @author  Maximilian Schlachte
/// @date    04.05.2022
///
// (pki): A device which records parking related data.
// Based on MSDevice_Example by Daniel Krajzewicz and Jakob Erdmann
/****************************************************************************/
#pragma once
#include <config.h>

#include "MSVehicleDevice.h"
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOTrafficObject;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_ParkingInfo
 * @brief A device which collects info on the vehicle trip concerning parking
 *
 * @see MSDevice
 */
class MSDevice_ParkingInfo : public MSVehicleDevice {
public:
    /** @brief Inserts MSDevice_ParkingInfo-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief resets counters
    static void cleanup();

public:
    /// @brief Destructor.
    ~MSDevice_ParkingInfo();



    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     *
     * @return True (always).
     */
    bool notifyMove(SUMOTrafficObject& veh, double oldPos,
                    double newPos, double newSpeed);


    /** @brief Saves departure info on insertion
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return Always true
     * @see MSMoveReminder::notifyEnter
     * @see MSMoveReminder::Notification
     */
    bool notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);


    /** @brief Saves arrival info
     *
     * @param[in] veh The leaving vehicle.
     * @param[in] lastPos Position on the lane when leaving.
     * @param[in] isArrival whether the vehicle arrived at its destination
     * @param[in] isLaneChange whether the vehicle changed from the lane
     * @return True if it did not leave the net.
     */
    bool notifyLeave(SUMOTrafficObject& veh, double lastPos,
                     MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);
    /// @}

    void notifyParking();

    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "ParkingInfo";
    }

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    //void setParameter(const std::string& key, const std::string& value);

    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::generateOutput
     */
    void generateOutput(OutputDevice* tripinfoOut) const;

    /** @brief Checkers whether a parking area is coming up for a given vehicle before the end edge
     *
     * @param[in] tObject The checked vehicle.
     * @return True if there is no parking area stop before the end edge
     */
    bool noParkingStopsBeforeEndEdge(SUMOTrafficObject& tObject) const;

private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_ParkingInfo(SUMOVehicle& holder, const std::string& id, std::vector<std::string> begIDs, std::vector<std::string> endIDs);



private:
    // private state members of the Example device

    /// @brief first edges for measurements
    std::vector<std::string> myBegIDs;

    /// @brief last edges for measurements
    std::vector<std::string> myEndIDs;

    /// @brief begID for this vehicle
    std::string myBegID;

    /// @brief endID for this vehicle
    std::string myEndID;

    bool myIsWithinBounds;
    bool myReachedParkingArea;

    /// @brief measured parameters
    SUMOTime myParkingDuration;
    SUMOTime myEnteringTime;
    SUMOTime myLeavingTime;
    SUMOTime myReparkingTime;

    /// @brief parameter time frame
    SUMOTime myBaseTime;
    SUMOTime myEndTime;

    /// @brief parameter base values for electric vehicles
    SUMOTime myBaseTotalChargingTime;
    SUMOTime myBaseTotalBlockingTime;
    double myBaseTotalEnergyCharged;

    /// @brief parameter values for electric vehicles
    SUMOTime myTotalChargingTime;
    SUMOTime myTotalBlockingTime;
    double myTotalEnergyCharged;

    /// @brief Holder cast to MSVehicle*
    MSVehicle* myMSHolder;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_ParkingInfo(const MSDevice_ParkingInfo&);

    /// @brief Invalidated assignment operator.
    MSDevice_ParkingInfo& operator=(const MSDevice_ParkingInfo&);


};
