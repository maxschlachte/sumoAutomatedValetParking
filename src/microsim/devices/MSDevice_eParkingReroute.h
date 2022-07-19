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
/// @file    MSDevice_eParkingReroute.h
/// @author  Maximilian Schlachte
/// @date    04.05.2022
///
// (cre): A device which reroutes fully charged vehicles from their charging space
// Based on MSDevice_Example by Daniel Krajzewicz and Jakob Erdmann
/****************************************************************************/
#pragma once
#include <config.h>

#include <mutex>
#include "MSVehicleDevice.h"
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOTrafficObject;
class MSDevice_Battery;
class MSVehicle;
class MSParkingArea;
class MSStop;

// ===========================================================================
// (cre): class definitions for eParkingReroute device
// ===========================================================================
/**
 * @class MSDevice_eParkingReroute
 * @brief A device which reroutes vehicles after being fully charged.
 *
 * @see MSDevice
 */
class MSDevice_eParkingReroute : public MSVehicleDevice {
public:

    /** @brief Inserts MSDevice_eParkingReroute-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a example-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief resets counters
    static void cleanup();

public:
    /// @brief Destructor.
    ~MSDevice_eParkingReroute();

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

    /** @brief Registers and unregisters vehicles for rerouting when moving into one of the begEdges and endEdges
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return Always true
     * @see MSMoveReminder::notifyEnter
     * @see MSMoveReminder::Notification
     */
    bool notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);


    /** @brief Unregisters vehicle if not already done
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
        return "eParkingReroute";
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

    /// @brief getter for maximum charge to which vehicles are considered candidates for calling
    float getMaxCharge() const {
        return myMaxCharge;
    }

    /// @brief getter for priorization flag
    bool prioritizeLowCharges() const {
        return myPrioLowCharges;
    }

    /// @brief unregisters a given vehicle from the List of possible call candidates
    void unregister(MSVehicle* veh);


private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     * @param[in] maxCharge The maximum charge to which vehicles are considered candidates for calling
     * @param[in] prioLowCharges Sets whether call candidates are sorted by their current charge in ascending order
     * @param[in] begIDs The IDs of the beginning Edges for registering call candidates
     * @param[in] endIDs The IDs of the end Edges for unregistering call candidates
     * @param[in] targetAreas The vector of parking areas the charged vehicles can be rerouted to
     * @param[in] callOnlyMode Whether a leaving vehicles should issue a call but not reroute anywhere else
     */
    MSDevice_eParkingReroute(SUMOVehicle& holder, const std::string& id, float maxCharge, bool prioLowCharges, std::vector<std::string> begIDs, std::vector<std::string> endIDs, std::vector<MSParkingArea*> targetAreas, bool callOnlyMode);



private:

    /// @brief sorting condition for charge values of candidates
    static bool compareCurrentCharges(const std::pair<MSVehicle*,MSDevice_Battery*> regVeh1, const std::pair<MSVehicle*,MSDevice_Battery*> regVeh2);

    /// @brief the holder as an MSVehicle reference (in order to avoid type casting at every step)
    MSVehicle* myMSVehicleHolder;

    /// @brief the holders battery
    MSDevice_Battery* myBattery;

    /// @brief measured parameters
    double myMaxCharge;

    /// @brief Holds time when entering net
    bool myPrioLowCharges;

    /// @brief Holds time when entering net
    SUMOTime myBaseTime;

    /// @brief Flag for indicating if the vehicles was registered
    bool myWasRegistered;

    /// @brief Flag for indicating that the vehicle called a candidate
    bool myCalledNextCandidate;

    /// @brief Flag for indicating that the vehicle should only issue a call when normally leaving and should not be rerouted itself
    bool myCallOnlyMode;

    /// @brief The vector of registered vehicles and their battery
    static std::vector<std::pair<MSVehicle*,MSDevice_Battery*>> myRegisteredVehicles;

    /// @brief the parking area the vehicle should be rerouted to
    std::vector<MSParkingArea*> myTargetAreas;

    /// @brief first edges for measurements
    const std::vector<std::string> myBegIDs;

    /// @brief last edges for measurements
    const std::vector<std::string> myEndIDs;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_eParkingReroute(const MSDevice_eParkingReroute&);

    /// @brief Invalidated assignment operator.
    MSDevice_eParkingReroute& operator=(const MSDevice_eParkingReroute&);


};
