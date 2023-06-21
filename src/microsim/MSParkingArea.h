/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2015-2022 German Aerospace Center (DLR) and others.
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
/// @file    MSParkingArea.h
/// @author  Mirco Sturari
/// @date    Tue, 19.01.2016
///
// A area where vehicles can park next to the road
/****************************************************************************/
#pragma once
#include <config.h>

#include <vector>
#include <algorithm>
#include <map>
#include <string>
#include <utils/geom/PositionVector.h>
#include <utils/common/Named.h>
#include <utils/vehicle/SUMOVehicleParameter.h>
// (chs): include charging space header
#include <microsim/trigger/MSChargingSpace.h>
#include "MSStoppingPlace.h"


// ===========================================================================
// class declarations
// ===========================================================================
class MSLane;
class SUMOVehicle;
class MSTransportable;
class Position;
class Command;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSParkingArea
 * @brief A lane area vehicles can halt at
 *
 * The stop tracks the last free space a vehicle may halt at by being
 *  informed about a vehicle's entering and depart. It keeps the information
 *  about entered vehicles' begin and end position within an internal
 *  container ("myEndPositions") and is so able to compute the last free space.
 *
 * Please note that using the last free space disallows vehicles to enter a
 *  free space in between other vehicles.
 */
class MSParkingArea : public MSStoppingPlace {
public:

    /** @brief Constructor
     *
     * @param[in] id The id of the stop
     * @param[in] net The net the stop belongs to
     * @param[in] lines Names of the lines that halt on this stop
     * @param[in] lane The lane the stop is placed on
     * @param[in] begPos Begin position of the stop on the lane
     * @param[in] endPos End position of the stop on the lane
     * @param[in] capacity Capacity of the stop
     * @param[in] width Width of the default lot rectangle
     * @param[in] length Length of the default lot rectangle
     * @param[in] angle Angle of the default lot rectangle
     */
    MSParkingArea(const std::string& id,
                  const std::vector<std::string>& lines, MSLane& lane,
                  double begPos, double endPos, int capacity,
                  double width, double length, double angle, const std::string& name,
                  bool onRoad,
                  const std::string& departPos,
                  // (qpk): add parameter for exit lane
                  MSLane* exitLane,
                  // (chs): add parameters for charging space (power, efficiency and charge delay)
                  double power, double efficiency, SUMOTime chargeDelay);

    /// @brief Destructor
    virtual ~MSParkingArea();

    /// @brief Returns the area capacity
    int getCapacity() const;

    /// @brief whether vehicles park on the road
    bool parkOnRoad() const;

    /// @brief compute lot for this vehicle
    int getLotIndex(const SUMOVehicle* veh) const;

    /** @brief Returns the area occupancy
     *
     * @return The occupancy computed as number of vehicles in myEndPositions
     * (reduced by 1 if at least one vehicle has finished parking but is blocked
     * from entering the road)
     */
    int getOccupancy() const;

    /// @brief Returns the area occupancy
    int getOccupancyIncludingBlocked() const;

    /// @brief Returns the area occupancy at the end of the last simulation step
    int getLastStepOccupancy() const;

    /** @brief Called if a vehicle enters this stop
     *
     * Stores the position of the entering vehicle in myEndPositions.
     *
     * Recomputes the free space using "computeLastFreePos" then.
     *
     * @param[in] what The vehicle that enters the parking area
     * @see computeLastFreePos
     */
    void enter(SUMOVehicle* veh);

    /** @brief Called if a vehicle leaves this stop
     *
     * Removes the position of the vehicle from myEndPositions.
     *
     * Recomputes the free space using "computeLastFreePos" then.
     *
     * @param[in] what The vehicle that leaves the parking area
     * @see computeLastFreePos
     */
    void leaveFrom(SUMOVehicle* what);

    /** @brief Called at the end of the time step
     *
     * Stores the current occupancy.
     *
     * @param[in] currentTime The current simulation time (unused)
     * @return Always 0 (the event is not rescheduled)
     */
    SUMOTime updateOccupancy(SUMOTime currentTime);

    // (utl): just return myLastFreePos
    double getMyLastFreePos() const {
        return myLastFreePos;
    }

    /// @brief Returns the last free position on this stop
    double getLastFreePos(const SUMOVehicle& forVehicle, double brakePos = 0) const;

    /** @brief Returns the last free position on this stop including
     * reservatiosn from the current lane and time step
     *
     * @return The last free position of this bus stop
     */
    double getLastFreePosWithReservation(SUMOTime t, const SUMOVehicle& forVehicle, double brakePos);

    /// @brief Returns the position of parked vehicle
    Position getVehiclePosition(const SUMOVehicle& forVehicle) const;

    /// @brief Returns the insertion position of a parked vehicle
    double getInsertionPosition(const SUMOVehicle& forVehicle) const;

    /// @brief Returns the angle of parked vehicle
    double getVehicleAngle(const SUMOVehicle& forVehicle) const;

    /// @brief Returns the slope of parked vehicle
    double getVehicleSlope(const SUMOVehicle& forVehicle) const;

    /** @brief Return the angle of myLastFreeLot - the next parking lot
     *         only expected to be called after we have established there is space in the parking area
     *
     * @return The angle of the lot in degrees
     */
    int getLastFreeLotAngle() const;

    /** @brief Return the GUI angle of myLastFreeLot - the angle the GUI uses to rotate into the next parking lot
     *         as above, only expected to be called after we have established there is space in the parking area
     *
     * @return The GUI angle, relative to the lane, in radians
     */
    double getLastFreeLotGUIAngle() const;

    /// @brief Return the manoeuver angle of the lot where the vehicle is parked
    int getManoeuverAngle(const SUMOVehicle& forVehicle) const;

    /// @brief  Return the GUI angle of the lot where the vehicle is parked
    double getGUIAngle(const SUMOVehicle& forVehicle) const;

    /** @brief Add a lot entry to parking area
     *
     * @param[in] x X position of the lot center
     * @param[in] y Y position of the lot center
     * @param[in] z Z position of the lot center
     * @param[in] width Width of the lot rectangle
     * @param[in] length Length of the lot rectangle
     * @param[in] angle Angle of the lot rectangle
     * @param[in] slope Slope of the lot rectangle
     * @return Whether the lot entry could be added
     */
    virtual void addLotEntry(double x, double y, double z,
                             double width, double length,
                             double angle, double slope,
                             // (chs): charging space parameter in space
                             MSChargingSpace* chargingSpace);


    // MS_EDIT (qpk): method for adding subspaces to space
    /** @brief setter for adding a subspace to the most recent space
     *
     * @param[in] x X position of the lot center
     * @param[in] y Y position of the lot center
     * @param[in] z Z position of the lot center
     * @param[in] width Width of the lot rectangle
     * @param[in] length Length of the lot rectangle
     * @param[in] angle Angle of the lot rectangle
     * @param[in] slope Slope of the lot rectangle
     * @param[in] charging space object if the space is a space for charging or nullpointer
     */
    virtual void addSubspace(double x, double y, double z,
                     double width, double length,
                     double angle, double slope,
                     // (chs): charging space parameter in addSubspace
                     MSChargingSpace* chargingSpace);

    double getFirstFreePos() const;

    // (qpk): declaration for method that pushes vehicles in a queue forward
    /** @brief pushes vehicles in a queue forward
     *
     * @param[in] veh Vehicle that should be pushed forward
     */
    void pushVehicleForward(SUMOVehicle* veh);

    // (qpk): forces leaders of a given vehicle forward in the queue
    /** @brief forces leaders of a vehicle in a queue forward
     *
     * @param[in] veh Vehicle whose leaders should be forced out of the queue
     */
    void forceLeadersForward(SUMOVehicle* veh);

    // (qpk): sorts the space occupancies by endPos
    void sortSpaceOccupancies();

    // (qpk): declaration for method that checks if a vehicle is on the last space of a queue
    /** @brief checks if a given vehicle is on a valid exit space of a queue. Valid exit spaces are the last spaces in a queue.
     *
     * @param[in] veh Vehicle that should be searched for
     */
    bool vehicleIsOnValidExitSpace(const SUMOVehicle& forVehicle) const;

    // (qpk): declare getter for first row capacity
    /// @brief Returns the first row capacity
    int getFirstRowCapacity() const;

    // (qpk): declare getter for first row occupancy
    /// @brief Returns occupancy for first row of parking spaces. Holds relevancy for queue parking, otherwise it will return the same results as getOccupancy().
    int getFirstRowOccupancy(bool includingBlocked = false) const;

    // (qpk): declare and implement getter for exit lane
    /// @brief getter for opposite lane
    MSLane* getExitLane() {
        return myExitLane;
    }

    // (chs): declare and implement getter for charging power
    /// @brief getter for charging power
    double getChargingPower() const {
        return myChargingPower;
    }

    // (chs): declare and implement getter for charging efficiency
    /// @brief getter for charging efficiency
    double getChargingEfficiency() const {
        return myChargingEfficiency;
    }

    // (chs): declare method for getting the charging space a vehicle is on.
    /// @brief return the charging space the vehicle is on, if it is on no charging space it returns nullptr
    MSChargingSpace* getChargingSpace(const SUMOVehicle& forVehicle) const;

    // (chs): declare and implement getter for charge delay
    /// @brief getter for charge delay
    double getChargeDelay() const {
        return myChargeDelay;
    }

    /// @brief Returns the lot rectangle width
    double getWidth() const;

    /// @brief Returns the lot rectangle length
    double getLength() const;

    /// @brief Returns the lot rectangle angle
    double getAngle() const;

    /// @brief update state so that vehicles wishing to enter cooperate with exiting vehicles
    void notifyEgressBlocked();

    /// @brief get number alternatives
    int getNumAlternatives() const;

    /// @brief set number alternatives
    void setNumAlternatives(int alternatives);

    // (utl): returns whether the next parking space is on the right side of the lane
    /// @brief checks whether the next parking space is on the right side of the lane
    bool nextSpaceIsOnRightSide();

protected:
    /** @struct LotSpaceDefinition
     * @brief Representation of a single lot space
     */
     struct LotSpaceDefinition {
          /// @brief default constructor
          LotSpaceDefinition();

          /// @brief parameter constructor
          LotSpaceDefinition(int index, SUMOVehicle* vehicle, double x, double y, double z, double rotation, double slope, double width, double length);

          // (chs): constructor for spaces with charging ability
          /// @brief parameter constructor
          LotSpaceDefinition(int index, SUMOVehicle* vehicle, double x, double y, double z, double rotation, double slope, double width, double length, MSChargingSpace* chargingSpace);

          // (utl): remove const identifier to make sorting possible
          /// @brief the running index
          //const int index;
          int index;

          /// @brief The last parked vehicle or 0
          //const SUMOVehicle* vehicle;
          SUMOVehicle* vehicle;

          /// @brief The position of the vehicle when parking in this space
          //const Position position;
          Position position;

          /// @brief The rotation
          //const double rotation;
          double rotation;

          /// @brief The slope
          //const double slope;
          double slope;

          /// @brief The width
          //const double width;
          double width;

          /// @brief The length
          //const double length;
          double length;

          /// @brief The position along the lane that the vehicle needs to reach for entering this lot
          double endPos;

          ///@brief The angle between lane and lot through which a vehicle must manoeuver to enter the lot
          double manoeuverAngle;

          ///@brief Whether the lot is on the LHS of the lane relative to the lane direction
          bool sideIsLHS;

          // (chs): holds an charging space object if the space is a charging space
          /// @brief Charging Space if the Lot is one, otherwise nullptr
          MSChargingSpace* chargingSpace;

          // (qpk): vector list of subspaces
          /// @brief All the spaces in this parking area
          std::vector<LotSpaceDefinition> subspaces;

          /// @brief Invalidated assignment operator.
          //LotSpaceDefinition& operator=(const LotSpaceDefinition&) = delete;
      };

      // (qpk): sorting condition
      static bool compareEndPos(LotSpaceDefinition csd1, LotSpaceDefinition csd2);

    /** @brief Computes the last free position on this stop
     *
     * The last free position is the one, the last vehicle ends at.
     * It is stored in myLastFreePos. If no vehicle halts, the last free
     *  position gets the value of myEndPos.
     */
    void computeLastFreePos();

    /// @brief Last free lot number (-1 no free lot)
    int myLastFreeLot;

    /// @brief Stop area capacity
    int myCapacity;

    /// @brief Whether vehicles stay on the road
    bool myOnRoad;

    /// @brief The default width of each parking space
    double myWidth;

    /// @brief The default length of each parking space
    double myLength;

    /// @brief The default angle of each parking space
    double myAngle;

    /// @brief All the spaces in this parking area
    std::vector<LotSpaceDefinition> mySpaceOccupancies;

    /// @brief The roadside shape of this parkingArea
    PositionVector myShape;

    /// @brief whether a vehicle wants to exit but is blocked
    bool myEgressBlocked;

    /// @brief track parking reservations from the lane for the current time step
    SUMOTime myReservationTime;

    /// @brief number of reservations
    int myReservations;

    /// @brief reservation max length
    double myReservationMaxLength;

    /// @brief the number of alternative parkingAreas that are assigned to parkingAreaRerouter
    int myNumAlternatives;

    /// @brief Changes to the occupancy in the current time step
    int myLastStepOccupancy;

    /// @brief custom departPos
    double myDepartPos;
    DepartPosDefinition myDepartPosDefinition;

    /// @brief Event for updating the occupancy
    Command* myUpdateEvent;

    // (qpk): add object for storing reference to exit lane
    /// @brief exit lane to which a vehicle exits, if none is given the vehicle should exit to the lane it came from
    MSLane* myExitLane;

    // (chs): charging power variable declaration for parking area
    /// @brief the charging power of this parking areas lots, mainly used during NLTriggerBuilder when retrieving the power from the parking area last created
    double myChargingPower;

    // (chs): charging efficiency variable declaration for parking area
    /// @brief the charging efficiency of this parking areas lots
    double myChargingEfficiency;

    // (chs): charge delay variable declaration for parking area
    /// @brief the charge delay of this parking areas lots
    double myChargeDelay;

    // (qpk): declare variable for capacity of first row of a queue parking area
    /// @brief Queue parking area capacity (if the stop is no queue parking area myFirstRowCapacity == myCapacity)
    int myFirstRowCapacity;

private:
    /// @brief Invalidated copy constructor.
    MSParkingArea(const MSParkingArea&) = delete;

    /// @brief Invalidated assignment operator.
    MSParkingArea& operator=(const MSParkingArea&) = delete;
};
