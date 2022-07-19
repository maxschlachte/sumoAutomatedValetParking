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
/// @file    MSParkingArea.cpp
/// @author  Mirco Sturari
/// @author  Jakob Erdmann
/// @date    Tue, 19.01.2016
///
// A area where vehicles can park next to the road
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utils/common/WrappingCommand.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeomHelper.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicleType.h>
#include "MSLane.h"
// (utl): include edge header (needed for distance measurements when creating the lots)
#include "MSEdge.h"
#include <microsim/transportables/MSTransportable.h>
#include "MSParkingArea.h"
#include "MSGlobals.h"

//#define DEBUG_RESERVATIONS
//#define DEBUG_GET_LAST_FREE_POS
//#define DEBUG_COND2(obj) (obj.getID() == "v.3")
#define DEBUG_COND2(obj) (obj.isSelected())


// ===========================================================================
// method definitions
// ===========================================================================
MSParkingArea::MSParkingArea(const std::string& id, const std::vector<std::string>& lines,
                             MSLane& lane, double begPos, double endPos, int capacity, double width, double length,
                             double angle, const std::string& name, bool onRoad,
                             const std::string& departPos,
                             // (qpk): add parameter for exit lane
                             MSLane* exitLane,
                             // (chs): add parameters for charging space (power, efficiency and charge delay)
                             double power, double efficiency, SUMOTime chargeDelay) :
    MSStoppingPlace(id, SUMO_TAG_PARKING_AREA, lines, lane, begPos, endPos, name),
    myRoadSideCapacity(capacity),
    myCapacity(0),
    myOnRoad(onRoad),
    myWidth(width),
    myLength(length),
    myAngle(angle),
    myEgressBlocked(false),
    myReservationTime(-1),
    myReservations(0),
    myReservationMaxLength(0),
    myNumAlternatives(0),
    myLastStepOccupancy(0),
    myDepartPos(-1),
    myDepartPosDefinition(DepartPosDefinition::DEFAULT),
    myUpdateEvent(nullptr),
    // (qpk): set exit lane
    myExitLane(exitLane),
    // (chs): set variables for charging spaces in this parking area
    myChargingPower(power),
    myChargingEfficiency(efficiency),
    myChargeDelay(chargeDelay),
    // (qpk): init myFirstRowCapacity
    myFirstRowCapacity(0) {
    // initialize unspecified defaults
    if (myWidth == 0) {
        myWidth = SUMO_const_laneWidth;
    }
    const double spaceDim = capacity > 0 ? myLane.interpolateLanePosToGeometryPos((myEndPos - myBegPos) / capacity) : 7.5;
    if (myLength == 0) {
        myLength = spaceDim;
    }
    if (departPos != "") {
        std::string error;
        if (!SUMOVehicleParameter::parseDepartPos(departPos, toString(myElement), getID(), myDepartPos, myDepartPosDefinition, error)) {
            throw ProcessError(error);
        }
        if (myDepartPosDefinition != DepartPosDefinition::GIVEN) {
            // maybe allow other methods at a later time
            throw ProcessError("Only a numerical departPos is supported for " + toString(myElement) + " '" + getID() + "'");
        } else if (myDepartPos < 0 || myDepartPos > lane.getLength()) {
            throw ProcessError("Invalid departPos for " + toString(myElement) + " '" + getID() + "'");
        }
    }

    const double offset = MSGlobals::gLefthand ? -1 : 1;
    myShape = lane.getShape().getSubpart(
                  lane.interpolateLanePosToGeometryPos(begPos),
                  lane.interpolateLanePosToGeometryPos(endPos));
    if (!myOnRoad) {
        myShape.move2side((lane.getWidth() / 2. + myWidth / 2.) * offset);
    }
    // Initialize space occupancies if there is a road-side capacity
    // The overall number of lots is fixed and each lot accepts one vehicle regardless of size
    for (int i = 0; i < capacity; ++i) {
        // calculate pos, angle and slope of parking lot space
        const Position pos = GeomHelper::calculateLotSpacePosition(myShape, i, spaceDim, myAngle, myWidth, myLength);
        double spaceAngle = GeomHelper::calculateLotSpaceAngle(myShape, i, spaceDim, myAngle);
        double spaceSlope = GeomHelper::calculateLotSpaceSlope(myShape, i, spaceDim);
        // (chs): declare chargingSpace
        MSChargingSpace* chargingSpace = nullptr;
        // (chs): if the area was declared as a charging area (has a positive power value) create every lot as a charging space
        if(power > 0) {
            std::string id_cs = id + "_cs_" + toString(i) + "_0";
            chargingSpace = new MSChargingSpace(id_cs, power, efficiency, chargeDelay);
        }
        // add lotEntry
        // (chs): pass chargingSpace
        addLotEntry(pos.x(), pos.y(), pos.z(), myWidth, myLength, spaceAngle, spaceSlope, chargingSpace);
        // update endPos
        mySpaceOccupancies.back().endPos = MIN2(myEndPos, myBegPos + MAX2(POSITION_EPS, spaceDim * (i + 1)));
    }
    computeLastFreePos();
}


MSParkingArea::~MSParkingArea() {}


void
MSParkingArea::addLotEntry(double x, double y, double z, double width, double length, double angle, double slope,
    // (chs): add paramter for charging space
    MSChargingSpace* chargingSpace) {
    // create LotSpaceDefinition
    // (chs): pass charging space to new lsd
    LotSpaceDefinition lsd((int)mySpaceOccupancies.size(), nullptr, x, y, z, angle, slope, width, length, chargingSpace);
    // If we are modelling parking set the end position to the lot position relative to the lane
    // rather than the end of the parking area - this results in vehicles stopping nearer the space
    // and re-entering the lane nearer the space. (If we are not modelling parking the vehicle will usually
    // enter the space and re-enter at the end of the parking area.)
    if (MSGlobals::gModelParkingManoeuver) {
        const double offset = this->getLane().getShape().nearest_offset_to_point2D(lsd.position);
        // (utl): handle invalid offset
        if (offset == GeomHelper::INVALID_OFFSET) {
            // (utl): measures distance to lot and decides whether it is at the end or at the front of the parking area
            if (this->getLane().getEdge().getFromJunction()->getShape().getPolygonCenter().distanceTo(lsd.position) < this->getLane().getEdge().getToJunction()->getShape().getPolygonCenter().distanceTo(lsd.position)) {
                lsd.endPos = getBeginLanePosition() + POSITION_EPS;
            } else {
                lsd.endPos = this->getLane().getLength() - POSITION_EPS;
            }
        } else if (offset <  getBeginLanePosition()) {
            lsd.endPos =  getBeginLanePosition() + POSITION_EPS;
        } else {
            if (this->getLane().getLength() > offset) {
                lsd.endPos = offset;
            } else {
                lsd.endPos = this->getLane().getLength() - POSITION_EPS;
            }
        }
        // Work out the angle of the lot relative to the lane  (-90 adjusts for the way the bay is drawn )
        double relativeAngle = fmod(lsd.rotation - 90., 360) - fmod(RAD2DEG(this->getLane().getShape().rotationAtOffset(lsd.endPos)), 360) + 0.5;
        if (relativeAngle < 0.) {
            relativeAngle += 360.;
        }
        lsd.manoeuverAngle = relativeAngle;

        // if p2.y is -ve the lot is on LHS of lane relative to lane direction
        // we need to know this because it inverts the complexity of the parking manoeuver
        Position p2 = this->getLane().getShape().transformToVectorCoordinates(lsd.position);
        if (p2.y() < (0. + POSITION_EPS)) {
            lsd.sideIsLHS = true;
        } else {
            lsd.sideIsLHS = false;
        }
    } else {
        lsd.endPos = myEndPos;
        lsd.manoeuverAngle = int(angle); // unused unless gModelParkingManoeuver is true
        lsd.sideIsLHS = true;
    }
    mySpaceOccupancies.push_back(lsd);
    myCapacity++;
    // (qpk): increase first row capacity as well
    myFirstRowCapacity++;
    computeLastFreePos();
}

// (utl): sorting method for spaces
void
MSParkingArea::sortSpaceOccupancies() {
    if (MSGlobals::gModelParkingManoeuver) {
        // (utl): sort the mySpaceOccupancies by endPos - this is needed especially for custom spaces if they are not ordered correctly in the additional file
        std::sort(mySpaceOccupancies.begin(), mySpaceOccupancies.end(), compareEndPos);
        // (utl): renumerate index since we sorted the spaces
        for(auto it = mySpaceOccupancies.begin(); it != mySpaceOccupancies.end(); ++it) {
            it->index = std::distance(mySpaceOccupancies.begin(), it);
        }
        computeLastFreePos();
    }
}

// (utl): define sorting condition for rearranging the space occupancies
bool
MSParkingArea::compareEndPos(LotSpaceDefinition csd1, LotSpaceDefinition csd2) {
    return csd1.endPos < csd2.endPos;
}

// (qpk): definition for addSubspace method. The method adds a subspace with the specified parameters to the most recently created space
void
MSParkingArea::addSubspace(double x, double y, double z, double width, double length, double angle, double slope, MSChargingSpace* chargingSpace) {
    LotSpaceDefinition ssd((int)mySpaceOccupancies.back().subspaces.size(), nullptr, x, y, z, angle, slope, width, length, chargingSpace);
    if (MSGlobals::gModelParkingManoeuver) {
        // Work out the angle of the lot relative to the lane  (-90 adjusts for the way the bay is drawn )
        double relativeAngle = fmod(ssd.rotation - 90., 360) - fmod(RAD2DEG(this->getExitLane()->getShape().rotationAtOffset(ssd.endPos)), 360) + 0.5;
        if (relativeAngle < 0.) {
            relativeAngle += 360.;
        }
        ssd.manoeuverAngle = relativeAngle;

        // if p2.y is -ve the lot is on LHS of lane relative to lane direction
        // we need to know this because it inverts the complexity of the parking manoeuver
        Position p2 = this->getExitLane()->getShape().transformToVectorCoordinates(ssd.position);
        if (p2.y() < (0. + POSITION_EPS)) {
            ssd.sideIsLHS = true;
        } else {
            ssd.sideIsLHS = false;
        }
    }
    //mySpaceOccupancies.at(lsdIdx).subspaces.push_back(ssd);
    mySpaceOccupancies.back().subspaces.push_back(ssd);
    myCapacity++;
}

// (qpk): definition for method that is pushing vehicles forward in a queue
void
MSParkingArea::pushVehicleForward(SUMOVehicle* veh) {
    // iterate through all normal spaces
    for(auto& lsd : mySpaceOccupancies) {
        // if the current space has subspaces
        for(int i = 0; i < (int)lsd.subspaces.size(); i++) {
            // if passed veh equals the one from the subspace and if it is not on the last subspace
            if(lsd.subspaces.at(i).vehicle == veh && i != (int)lsd.subspaces.size()-1 && lsd.subspaces.at(i+1).vehicle == nullptr) {
                lsd.subspaces.at(i+1).vehicle = veh;
                lsd.subspaces.at(i).vehicle = nullptr;
                break;
            // if the vehicle is on the current normal space and the first subspace is empty
            } else if(lsd.vehicle == veh && lsd.subspaces.at(0).vehicle == nullptr) {
                //std::cout << "push veh " << veh.getID() << " from normal parking space " << lsd.index << " to subspace at pos 0" << std::endl;
                // push vehicle from normal space on first subspace
                lsd.subspaces.at(0).vehicle = veh;
                lsd.vehicle = nullptr;
                //std::cout << "veh " << lsd.subspaces.at(0).vehicle.getID() << " on subspace" << lsd.subspaces.at(0).index << std::endl;
                break;
            }
        }
    }
    computeLastFreePos();
}

// (qpk): forces vehicles in front of given vehicle to leave the parking area and loop back to the entry lane of the parking area
void
MSParkingArea::forceLeadersForward(SUMOVehicle* veh) {
    bool vehicleFoundInColumn = false;
    // iterate through all normal spaces
    for(auto& lsd : mySpaceOccupancies) {
        if(lsd.vehicle == veh) {
            vehicleFoundInColumn = true;
        }
        // if the current space has subspaces
        for(int i = 0; i < (int)lsd.subspaces.size(); i++) {
            if (vehicleFoundInColumn) {
                // if the next subspace holds a vehicle
                if(lsd.subspaces.at(i).vehicle != nullptr) {
                    // force the vehicle forward
                    lsd.subspaces.at(i).vehicle->forceLeave();
                }
            } else {
                if(lsd.subspaces.at(i).vehicle == veh) {
                    vehicleFoundInColumn = true;
                }
            }
        }
        if (vehicleFoundInColumn) {
            break;
        }
    }
}

int
MSParkingArea::getLastFreeLotAngle() const {
    assert(myLastFreeLot >= 0);
    assert(myLastFreeLot < (int)mySpaceOccupancies.size());

    const LotSpaceDefinition& lsd = mySpaceOccupancies[myLastFreeLot];
    if (lsd.sideIsLHS) {
        return abs(int(lsd.manoeuverAngle)) % 180;
    } else {
        return abs(abs(int(lsd.manoeuverAngle)) % 180 - 180) % 180;
    }
}

double
MSParkingArea::getLastFreeLotGUIAngle() const {
    assert(myLastFreeLot >= 0);
    assert(myLastFreeLot < (int)mySpaceOccupancies.size());

    const LotSpaceDefinition& lsd = mySpaceOccupancies[myLastFreeLot];
    if (lsd.manoeuverAngle > 180.) {
        return DEG2RAD(lsd.manoeuverAngle - 360.);
    } else {
        return DEG2RAD(lsd.manoeuverAngle);
    }
}


double
MSParkingArea::getLastFreePos(const SUMOVehicle& forVehicle, double brakePos) const {
    if (myCapacity == (int)myEndPositions.size()) {
        // keep enough space so that  parking vehicles can leave
#ifdef DEBUG_GET_LAST_FREE_POS
        if (DEBUG_COND2(forVehicle)) {
            std::cout << SIMTIME << " getLastFreePos veh=" << forVehicle.getID() << " allOccupied\n";
        }
#endif
        return myLastFreePos - forVehicle.getVehicleType().getMinGap() - POSITION_EPS;
    } else {
        const double minPos = MIN2(myEndPos, brakePos);
        if (myLastFreePos >= minPos) {
#ifdef DEBUG_GET_LAST_FREE_POS
            if (DEBUG_COND2(forVehicle)) {
                std::cout << SIMTIME << " getLastFreePos veh=" << forVehicle.getID() << " brakePos=" << brakePos << " myEndPos=" << myEndPos << " using myLastFreePos=" << myLastFreePos << "\n";
            }
#endif
            return myLastFreePos;
        } else {
            // find free pos after minPos
            for (const auto& lsd : mySpaceOccupancies) {
                if ((lsd.vehicle == nullptr && lsd.endPos >= minPos) || (lsd.subspaces.size() > 0 && lsd.subspaces.at(0).vehicle == nullptr)) {
#ifdef DEBUG_GET_LAST_FREE_POS
                    if (DEBUG_COND2(forVehicle)) {
                        std::cout << SIMTIME << " getLastFreePos veh=" << forVehicle.getID() << " brakePos=" << brakePos << " myEndPos=" << myEndPos << " nextFreePos=" << lsd.endPos << "\n";
                    }
#endif
                    return lsd.endPos;
                }
            }
            // shouldn't happen. No good solution seems possible
#ifdef DEBUG_GET_LAST_FREE_POS
            if (DEBUG_COND2(forVehicle)) {
                std::cout << SIMTIME << " getLastFreePos veh=" << forVehicle.getID() << " brakePos=" << brakePos << " myEndPos=" << myEndPos << " noGoodFreePos blockedAt=" << brakePos << "\n";
            }
#endif
            return brakePos;
        }
    }
}

Position
MSParkingArea::getVehiclePosition(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            return lsd.position;
        }
        // (qpk): return position on subspaces
        for(const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                return ssd.position;
            }
        }
    }
    return Position::INVALID;
}


double
MSParkingArea::getInsertionPosition(const SUMOVehicle& forVehicle) const {
    if (myDepartPosDefinition == DepartPosDefinition::GIVEN) {
        return myDepartPos;
    }
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            // (qpk): return the pos of a vehicle on a space extrapolated to the exit lane
            if (myExitLane != nullptr) {
                return myExitLane->getShape().nearest_offset_to_point2D(getVehiclePosition(forVehicle)) > 0 ? myExitLane->getShape().nearest_offset_to_point2D(getVehiclePosition(forVehicle)) : myExitLane->getLength();
            }
            return lsd.endPos;
        }
        // (qpk): return the pos of a vehicle on a subspace  extrapolated to the exit lane
        for (const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                if (myExitLane != nullptr) {
                    return myExitLane->getShape().nearest_offset_to_point2D(getVehiclePosition(forVehicle)) > 0 ? myExitLane->getShape().nearest_offset_to_point2D(getVehiclePosition(forVehicle)) : myExitLane->getLength();
                }
                return ssd.endPos;
            }
        }
    }
    return -1;
}


double
MSParkingArea::getVehicleAngle(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            return (lsd.rotation - 90.) * (double) M_PI / (double) 180.0;
        }
        // (qpk): return the angle if the vehicle is on a subspace
        for (const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                return (ssd.rotation - 90.) * (double) M_PI / (double) 180.0;
            }
        }
    }
    return 0;
}

double
MSParkingArea::getVehicleSlope(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            return lsd.slope;
        }
        // (qpk): return the slope if the vehicle is on a subspace
        for (const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                return ssd.slope;
            }
        }
    }
    return 0;
}

// (qpk): check if a vehicle is on the last space of a queue and if so return true
bool
MSParkingArea::vehicleIsOnValidExitSpace(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.subspaces.size() > 0) {
            if (lsd.subspaces.at(lsd.subspaces.size()-1).vehicle == &forVehicle) return true;
        } else {
            if (lsd.vehicle == &forVehicle) return true;
        }
    }
    return false;
}

// (utl): checks if the next parking space on the right side of the lane (main use is for one way roads with spaces defined on the left of the road. Do not use for opposite lane with opposite direction lane - this method is bound to the lane the parking area is on and compares the position of the defined spaces to the position of the parking area relative to the lane)
bool
MSParkingArea::nextSpaceIsOnRightSide() {
    // (utl): parking area center projected on position on lane
    double lanePosOfParkingArea = myLane.interpolateGeometryPosToLanePos(myLane.getShape().nearest_offset_to_point2D(myShape.getPolygonCenter()));
    // (utl): substract x and y coordinates of the parking area center from the respective x and y of the geometry position of the parking area center on the lane
    double xDelta = myLane.geometryPositionAtOffset(lanePosOfParkingArea).x() - myShape.getPolygonCenter().x();
    double yDelta = myLane.geometryPositionAtOffset(lanePosOfParkingArea).y() - myShape.getPolygonCenter().y();
    // (utl): If there were no spaces defined return the standard value (true, the space is on the right side)
    if (mySpaceOccupancies.size() < myLastFreeLot) {
        return true;
    }
    // (utl): substract x and y coordinates of the next parking space from the respective x and y of the geometry position of the space position on the lane
    double xSpace = myLane.geometryPositionAtOffset(myLastFreePos).x() - mySpaceOccupancies[myLastFreeLot].position.x();
    double ySpace = myLane.geometryPositionAtOffset(myLastFreePos).y() - mySpaceOccupancies[myLastFreeLot].position.y();
    // (utl): reduce calculated values to 1, 0 or -1 to make the direction of the positions comparable
    double xDeltaVal = xDelta != 0 ? xDelta / abs(xDelta) : 0;
    double yDeltaVal = yDelta != 0 ? yDelta / abs(yDelta) : 0;
    double xSpaceVal = xSpace != 0 ? xSpace / abs(xSpace) : 0;
    double ySpaceVal = ySpace != 0 ? ySpace / abs(ySpace) : 0;
    // (utl): if calculated values of space and parking area match the space must be on the side of the parking area - which means it is on the right side
    if(xDeltaVal == xSpaceVal && yDeltaVal == ySpaceVal) {
        return true;
    }
    // (utl): if the values do not match the space is on the left side
    return false;
}

double
MSParkingArea::getGUIAngle(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            if (lsd.manoeuverAngle > 180.) {
                return DEG2RAD(lsd.manoeuverAngle - 360.);
            } else {
                return DEG2RAD(lsd.manoeuverAngle);
            }
        }
        // (qpk): iterate through subspaces to get angle (GUI) of vehicle
        for (const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                if (ssd.manoeuverAngle > 180.) {
                    return DEG2RAD(ssd.manoeuverAngle - 360.);
                } else {
                    return DEG2RAD(ssd.manoeuverAngle);
                }
            }
        }
    }
    return 0.;
}

int
MSParkingArea::getManoeuverAngle(const SUMOVehicle& forVehicle) const {
    for (const auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == &forVehicle) {
            if (lsd.sideIsLHS) {
                return abs(int(lsd.manoeuverAngle)) % 180;
            } else {
                return abs(abs(int(lsd.manoeuverAngle)) % 180 - 180) % 180;
            }
        }
        // (qpk): iterate through subspaces to get angle (normal) of vehicle
        for (const auto& ssd : lsd.subspaces) {
            if (ssd.vehicle == &forVehicle) {
                if (ssd.sideIsLHS) {
                    return abs(int(ssd.manoeuverAngle)) % 180;
                } else {
                    return abs(abs(int(ssd.manoeuverAngle)) % 180 - 180) % 180;
                }
            }
        }
    }
    return 0;
}

int
MSParkingArea::getLotIndex(const SUMOVehicle* veh) const {
    if (veh->getPositionOnLane() > myLastFreePos) {
        // vehicle has gone past myLastFreePos and we need to find the actual lot
        int closestLot = 0;
        for (int i = 0; i < (int)mySpaceOccupancies.size(); i++) {
            const LotSpaceDefinition lsd = mySpaceOccupancies[i];
            if (lsd.vehicle == nullptr) {
                closestLot = i;
                if (lsd.endPos >= veh->getPositionOnLane()) {
                    return i;
                }
            }
        }
        // for on-road parking we need to be precise
        return myOnRoad ? -1 : closestLot;
    }
    if (myOnRoad && myLastFreePos - veh->getPositionOnLane() > POSITION_EPS) {
        // for on-road parking we need to be precise
        return -1;
    }
    return myLastFreeLot;
}

void
MSParkingArea::enter(SUMOVehicle* veh) {
    double beg = veh->getPositionOnLane() + veh->getVehicleType().getMinGap();
    double end = veh->getPositionOnLane() - veh->getVehicleType().getLength();
    if (myUpdateEvent == nullptr) {
        myUpdateEvent = new WrappingCommand<MSParkingArea>(this, &MSParkingArea::updateOccupancy);
        MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(myUpdateEvent);
    }
    int lotIndex = getLotIndex(veh);
    if (lotIndex < 0) {
        WRITE_WARNING("Unsuitable parking position for vehicle '" + veh->getID() + "' at parkingArea '" + getID() + "' time=" + time2string(SIMSTEP));
        lotIndex = myLastFreeLot;
    }
#ifdef DEBUG_GET_LAST_FREE_POS
    ((SUMOVehicleParameter&)veh->getParameter()).setParameter("lotIndex", toString(lotIndex));
#endif
    assert(myLastFreePos >= 0);
    assert(lotIndex < (int)mySpaceOccupancies.size());
    mySpaceOccupancies[lotIndex].vehicle = veh;
    myEndPositions[veh] = std::pair<double, double>(beg, end);
    computeLastFreePos();
    // current search ends here
    veh->setNumberParkingReroutes(0);
}


void
MSParkingArea::leaveFrom(SUMOVehicle* what) {
    assert(myEndPositions.find(what) != myEndPositions.end());
    if (myUpdateEvent == nullptr) {
        myUpdateEvent = new WrappingCommand<MSParkingArea>(this, &MSParkingArea::updateOccupancy);
        MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(myUpdateEvent);
    }
    for (auto& lsd : mySpaceOccupancies) {
        // (qpk): leave from subspace
        if(lsd.subspaces.size() > 0) {
            if(lsd.subspaces.at(lsd.subspaces.size()-1).vehicle == what) {
                lsd.subspaces.at(lsd.subspaces.size()-1).vehicle = nullptr;
                break;
            }
        }
        if (lsd.vehicle == what) {
            lsd.vehicle = nullptr;
            break;
        }
    }
    myEndPositions.erase(myEndPositions.find(what));
    computeLastFreePos();
}


SUMOTime
MSParkingArea::updateOccupancy(SUMOTime /* currentTime */) {
    myLastStepOccupancy = getOccupancy();
    myUpdateEvent = nullptr;
    return 0;
}


MSParkingArea::LotSpaceDefinition::LotSpaceDefinition() :
    index(-1),
    vehicle(nullptr),
    rotation(0),
    slope(0),
    width(0),
    length(0),
    endPos(0),
    manoeuverAngle(0),
    sideIsLHS(false) {
}


MSParkingArea::LotSpaceDefinition::LotSpaceDefinition(int index_, SUMOVehicle* vehicle_, double x, double y, double z, double rotation_, double slope_, double width_, double length_) :
    index(index_),
    vehicle(vehicle_),
    position(Position(x, y, z)),
    rotation(rotation_),
    slope(slope_),
    width(width_),
    length(length_),
    endPos(0),
    manoeuverAngle(0),
    sideIsLHS(false) {
}

// (chs): charging space constructor
MSParkingArea::LotSpaceDefinition::LotSpaceDefinition(int index_, SUMOVehicle* vehicle_, double x, double y, double z, double rotation_, double slope_, double width_, double length_, MSChargingSpace* chargingSpace_) :
    index(index_),
    vehicle(vehicle_),
    position(Position(x, y, z)),
    rotation(rotation_),
    slope(slope_),
    width(width_),
    length(length_),
    endPos(0),
    manoeuverAngle(0),
    sideIsLHS(false),
    chargingSpace(chargingSpace_) {
}


void
MSParkingArea::computeLastFreePos() {
    myLastFreeLot = -1;
    myLastFreePos = myBegPos;
    myEgressBlocked = false;
    for (auto& lsd : mySpaceOccupancies) {
        if (lsd.vehicle == nullptr
                || (//getOccupancy() == getCapacity()
                    // (qpk): check for first capacity row rather than whole capacity
                    getFirstRowOccupancy() == getFirstRowCapacity()
                    && lsd.vehicle->remainingStopDuration() <= 0
                    && !lsd.vehicle->isStoppedTriggered())) {
            if (lsd.vehicle == nullptr) {
                myLastFreeLot = lsd.index;
                myLastFreePos = lsd.endPos;
            } else {
                // vehicle wants to exit the parking area
                myLastFreeLot = lsd.index;
                myLastFreePos = lsd.endPos - lsd.vehicle->getVehicleType().getLength() - POSITION_EPS;
                myEgressBlocked = true;
            }
            break;
        } else {
            myLastFreePos = MIN2(myLastFreePos,
                                 lsd.endPos - lsd.vehicle->getVehicleType().getLength() - NUMERICAL_EPS);
            // (qpk): if there is a free lot in the queue we want to take the lsd at the start of the queue
            for (auto& ssd : lsd.subspaces) {
                if (ssd.vehicle == nullptr) {
                    myLastFreeLot = lsd.index;
                    myLastFreePos = lsd.endPos;
                    break;
                }
            }
        }
    }
}


double
MSParkingArea::getLastFreePosWithReservation(SUMOTime t, const SUMOVehicle& forVehicle, double brakePos) {
    if (forVehicle.getLane() != &myLane) {
        // for different lanes, do not consider reservations to avoid lane-order
        // dependency in parallel simulation
#ifdef DEBUG_RESERVATIONS
        if (DEBUG_COND2(forVehicle)) {
            std::cout << SIMTIME << " pa=" << getID() << " freePosRes veh=" << forVehicle.getID() << " other lane\n";
        }
#endif
        if (myNumAlternatives > 0 && getOccupancy() == getCapacity()) {
            // ensure that the vehicle reaches the rerouter lane
            return MAX2(myBegPos, MIN2(POSITION_EPS, myEndPos));
        } else {
            return getLastFreePos(forVehicle, brakePos);
        }
    }
    if (t > myReservationTime) {
#ifdef DEBUG_RESERVATIONS
        if (DEBUG_COND2(forVehicle)) {
            std::cout << SIMTIME << " pa=" << getID() << " freePosRes veh=" << forVehicle.getID() << " first reservation\n";
        }
#endif
        myReservationTime = t;
        myReservations = 1;
        myReservationMaxLength = forVehicle.getVehicleType().getLength();
        for (const auto& lsd : mySpaceOccupancies) {
            if (lsd.vehicle != nullptr) {
                myReservationMaxLength = MAX2(myReservationMaxLength, lsd.vehicle->getVehicleType().getLength());
            }
        }
        return getLastFreePos(forVehicle, brakePos);
    } else {
        if (myCapacity > getOccupancy() + myReservations) {
#ifdef DEBUG_RESERVATIONS
            if (DEBUG_COND2(forVehicle)) {
                std::cout << SIMTIME << " pa=" << getID() << " freePosRes veh=" << forVehicle.getID() << " res=" << myReservations << " enough space\n";
            }
#endif
            myReservations++;
            myReservationMaxLength = MAX2(myReservationMaxLength, forVehicle.getVehicleType().getLength());
            return getLastFreePos(forVehicle, brakePos);
        } else {
            if (myCapacity == 0) {
                return getLastFreePos(forVehicle, brakePos);
            } else {
#ifdef DEBUG_RESERVATIONS
                if (DEBUG_COND2(forVehicle)) std::cout << SIMTIME << " pa=" << getID() << " freePosRes veh=" << forVehicle.getID()
                                                           << " res=" << myReservations << " resTime=" << myReservationTime << " reserved full, maxLen=" << myReservationMaxLength << " endPos=" << mySpaceOccupancies[0].endPos << "\n";
#endif
                return (mySpaceOccupancies[0].endPos
                        - myReservationMaxLength
                        - forVehicle.getVehicleType().getMinGap()
                        - NUMERICAL_EPS);
            }
        }
    }
}


double
MSParkingArea::getWidth() const {
    return myWidth;
}


double
MSParkingArea::getLength() const {
    return myLength;
}


double
MSParkingArea::getAngle() const {
    return myAngle;
}


int
MSParkingArea::getCapacity() const {
    return myCapacity;
}


bool
MSParkingArea::parkOnRoad() const {
    return myOnRoad;
}


int
MSParkingArea::getOccupancy() const {
    return (int)myEndPositions.size() - (myEgressBlocked ? 1 : 0);
}


int
MSParkingArea::getOccupancyIncludingBlocked() const {
    return (int)myEndPositions.size();
}


int
MSParkingArea::getLastStepOccupancy() const {
    return myLastStepOccupancy;
}

void
MSParkingArea::notifyEgressBlocked() {
    computeLastFreePos();
}


int
MSParkingArea::getNumAlternatives() const {
    return myNumAlternatives;
}


void
MSParkingArea::setNumAlternatives(int alternatives) {
    myNumAlternatives = MAX2(myNumAlternatives, alternatives);
}


// (qpk): getter for myFirstRowCapacity
int
MSParkingArea::getFirstRowCapacity() const {
    return myFirstRowCapacity;
}


// (qpk): method for getting the occupancy of the first row
int
MSParkingArea::getFirstRowOccupancy(bool includingBlocked) const {
    int firstRowVehicleCount = 0;
    for(auto& lsd : mySpaceOccupancies) {
        if(lsd.vehicle != nullptr) firstRowVehicleCount++;
    }
    if (includingBlocked) {
        return firstRowVehicleCount;
    }
    return firstRowVehicleCount - (myEgressBlocked ? 1 : 0);
}

// (chs): method for getting the charging space a vehicle is on or nullptr
MSChargingSpace*
MSParkingArea::getChargingSpace(const SUMOVehicle& forVehicle) const {
    for(auto it = mySpaceOccupancies.begin(); it != mySpaceOccupancies.end(); ++it) {
        if((*it).vehicle == &forVehicle) {
            return (*it).chargingSpace;
        }
        for(auto it_sub = (*it).subspaces.begin(); it_sub != (*it).subspaces.end(); ++it_sub) {
            if((*it_sub).vehicle == &forVehicle) {
                return (*it_sub).chargingSpace;
            }
        }
    }
    return nullptr;
}

/****************************************************************************/
