/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2022 German Aerospace Center (DLR) and others.
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
/// @file    NEMAController.cpp
/// @author  Tianxin Li
/// @author  Qichao Wang
/// @date    August 2020
///
// An actuated NEMA-phase-compliant traffic light logic
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#include <microsim/MSEventControl.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/output/MSInductLoop.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include "MSTrafficLightLogic.h"
#include "NEMAController.h"
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <netload/NLDetectorBuilder.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include "microsim/output/MSE2Collector.h"
#include <sstream>
#include <iostream>
// ===========================================================================
// parameter defaults definitions
// ===========================================================================


#define INVALID_POSITION std::numeric_limits<double>::max() // tl added

// #define DEBUG_NEMA

// ===========================================================================
// method definitions
// ===========================================================================
NEMALogic::NEMALogic(MSTLLogicControl& tlcontrol,
                     const std::string& id, const std::string& programID,
                     const SUMOTime _offset,
                     const Phases& phases,
                     int /*step*/, SUMOTime /*delay*/,
                     const std::map<std::string, std::string>& parameter,
                     const std::string& basePath) :
    MSSimpleTrafficLightLogic(tlcontrol, id, programID, _offset, TrafficLightType::NEMA, phases, 0, phases.front()->minDuration + SIMSTEP, parameter),
    myPhase(phases[0]->duration, phases[0]->getState()) {
    myDetectorLength = StringUtils::toDouble(getParameter("detector-length", "20"));
    myDetectorLengthLeftTurnLane = StringUtils::toDouble(getParameter("detector-length-leftTurnLane", "20"));
    myCycleLength = (StringUtils::toDouble(getParameter("total-cycle-length", getParameter("cycle-length", getParameter(toString(SUMO_ATTR_CYCLETIME), "60")))));
    myNextCycleLength = myCycleLength;
    myDefaultCycleTime = TIME2STEPS(myCycleLength);
    myShowDetectors = StringUtils::toBool(getParameter("show-detectors", toString(OptionsCont::getOptions().getBool("tls.actuated.show-detectors"))));
    myFile = FileHelpers::checkForRelativity(getParameter("file", "NUL"), basePath);
    myFreq = TIME2STEPS(StringUtils::toDouble(getParameter("freq", "300")));
    myVehicleTypes = getParameter("vTypes", "");
    ring1 = getParameter("ring1", "");
    ring2 = getParameter("ring2", "");

    std::vector<int> VecMinRecall = readParaFromString(getParameter("minRecall", "1,2,3,4,5,6,7,8"));
    for (int i = 0; i < (int)VecMinRecall.size(); i++) {
        minRecalls[VecMinRecall[i] - 1] = true;
        recall[VecMinRecall[i] - 1] = true;
    }

    std::vector<int> VecMaxRecall = readParaFromString(getParameter("maxRecall", ""));
    for (int i = 0; i < (int)VecMaxRecall.size(); i++) {
        maxRecalls[VecMaxRecall[i] - 1] = true;
        recall[VecMaxRecall[i] - 1] = true;
    }

#ifdef DEBUG_NEMA
    std::cout << "minRecall: ";
    for (int i = 0; i < 8; i++) {
        std::cout << minRecalls[i] << '\t';
    }
    std::cout << std::endl;

    std::cout << "maxRecall: ";
    for (int i = 0; i < 8; i++) {
        std::cout << maxRecalls[i] << '\t';
    }
    std::cout << std::endl;
#endif
    barriers = getParameter("barrierPhases", "");
    coordinates = getParameter("coordinatePhases", getParameter("barrier2Phases", ""));
    fixForceOff = StringUtils::toBool(getParameter("fixForceOff", "false"));
    offset = STEPS2TIME(_offset);
    myNextOffset = offset;
    whetherOutputState = StringUtils::toBool(getParameter("whetherOutputState", "false"));
    coordinateMode = StringUtils::toBool(getParameter("coordinate-mode", "false"));
    greenTransfer = StringUtils::toBool(getParameter("greenTransfer", "true"));

    //missing parameter error
    error_handle_not_set(ring1, "ring1");
    error_handle_not_set(ring2, "ring2");
    error_handle_not_set(barriers, "barrierPhases");
    error_handle_not_set(coordinates, "barrier2Phases or coordinatePhases");

    //print to check
#ifdef DEBUG_NEMA
    std::cout << "JunctionID = " << myID << std::endl;
    std::cout << "All parameters after calling constructor are: " << std::endl;
    std::cout << "myDetectorLength = " << myDetectorLength << std::endl;
    std::cout << "cycleLength = " << myCycleLength << std::endl;
    std::cout << "ring1 = " << ring1 << std::endl;
    std::cout << "ring2 = " << ring2 << std::endl;
    std::cout << "barriers = " << barriers << std::endl;
    std::cout << "coordinates = " << coordinates << std::endl;
    std::cout << "offset = " << offset << std::endl;
    std::cout << "whetherOutputState = " << whetherOutputState << std::endl;
    std::cout << "myShowDetectors = " << myShowDetectors << std::endl;
    std::cout << "coordinateMode = " << coordinateMode << std::endl;
    std::cout << "fixForceOff = " << fixForceOff << std::endl;
    std::cout << "greenTransfer = " << greenTransfer << std::endl;
    std::cout << "You reach the end of constructor" << std::endl;
    std::cout << "****************************************\n";
#endif

    // construct the phaseDetectorMapping. In the future this could hold more parameters, such as lock in time or delay
    for (auto p : readParaFromString(ring1)) {
        // #TODO clean up this string mess
        std::string cps = "crossPhaseSwitching:";
        int crossPhase = StringUtils::toInt(getParameter(cps.append(std::to_string(p)), "0"));
        phase2DetectorMap[p] = phaseDetectorInfo(crossPhase);
    }
    for (auto p : readParaFromString(ring2)) {
        std::string cps = "crossPhaseSwitching:";
        int crossPhase = StringUtils::toInt(getParameter(cps.append(std::to_string(p)), "0"));
        phase2DetectorMap[p] = phaseDetectorInfo(crossPhase);
    }
    // Construct the Cross Mapping
    for (auto const& phaseDetectInfo : phase2DetectorMap) {
        if (phaseDetectInfo.second.cpdSource > 0) {
            phase2DetectorMap.find(phaseDetectInfo.second.cpdSource) -> second.cpdTarget = phaseDetectInfo.first;
        }
    }

}

NEMALogic::~NEMALogic() { }

void
NEMALogic::init(NLDetectorBuilder& nb) {
    //init the base path for output state
    outputStateFilePath = outputStateFilePath + "/" + myID + "_state_output";
    // std::cout << "outputStaetFilePath = " << outputStateFilePath << std::endl;
    //init cycleRefPoint
    cycleRefPoint = 0;

    //init outputStateFile
    if (whetherOutputState) {
        outputStateFile.open(outputStateFilePath);
        outputStateFile << "Output state changes:\n";
        outputStateFile.close();
    }
    //init phaseStartTime and phaseExpectedDuration
    int phaseNumber = 8;
    for (int i = 0; i < phaseNumber; i++) {
        phaseStartTime[i] = 0;
        phaseExpectedDuration[i] = 0;
    }

    //print to check
    //init minGreen, maxGreen, vehExt, red, and yellow
    for (MSPhaseDefinition* phase : myPhases) {
        int NEMAPhase = string2int(phase->getName());
        int i = NEMAPhase - 1;
        // std::string indexFromName = phase->getName();
        // std::stringstream ss(indexFromName);
        // int NEMAPhase = 0;
        // ss << NEMAPhase;
#ifdef DEBUG_NEMA
        std::cout << "NEMAIndex = " << NEMAPhase << ": ";
#endif
        minGreen[i] = STEPS2TIME(phase->minDuration);
        maxGreen[i] = STEPS2TIME(phase->maxDuration);
        nextMaxGreen[i] = maxGreen[i];
        vehExt[i] = STEPS2TIME(phase->vehext);
        yellowTime[i] = STEPS2TIME(phase->yellow);
        redTime[i] = STEPS2TIME(phase->red);
        //map state G index to laneIDs
        std::string state = phase->getState();
        std::set<std::string> laneIDs = getLaneIDsFromNEMAState(state);
        std::vector<std::string> laneIDs_vector;
        for (std::string laneID : laneIDs) {
            laneIDs_vector.push_back(laneID);
        }
        phase2ControllerLanesMap[NEMAPhase] = laneIDs_vector;
#ifdef DEBUG_NEMA
        std::cout << "minGreen = " << minGreen[i] << "; maxGreen = " << maxGreen[i] << "; vehext = " << vehExt[i] << "; yellow = " << yellowTime[i] << "; redTime = " << redTime[i] << std::endl;
#endif
    }


#ifdef DEBUG_NEMA
    //print to check the phase2ControllerLanesMap
    for (auto item : phase2ControllerLanesMap) {
        std::cout << "NEMA phase index = " << item.first << " have lanes: ";
        for (auto id : item.second) {
            std::cout << id << " ";
        }
        std::cout << std::endl;
    }
#endif


    //init rings
    rings.push_back(readParaFromString(ring1));
    rings.push_back(readParaFromString(ring2));



#ifdef DEBUG_NEMA
    //print to check
    for (int i = 0; i < (int)rings.size(); i++) {
        int count = 0;
        std::cout << "Ring" << i + 1 << " includes phases: \t";
        for (auto j : rings[i]) {
            count++;
            std::cout << j << " ";
            if (count == 2 || count == 4) {
                std::cout << " | ";
            }
        }
        std::cout << std::endl;
    }
#endif

    //init barriers
    barrierPhaseIndecies = readParaFromString(barriers);
    coordinatePhaseIndecies = readParaFromString(coordinates);

    //init the active index for rings and barriers
    activeRing1Index = 0;
    activeRing2Index = 0;
    activeRing1Phase = 0;
    activeRing2Phase = 0;

    for (int i = 0; (int)rings[0].size(); i++) {
        if (rings[0][i] != 0) {
#ifdef DEBUG_NEMA
            std::cout << "rings[0][" << i << "] = " << rings[0][i] << std::endl;
#endif
            activeRing1Index = i;
            activeRing1Phase = rings[0][activeRing1Index];
            break;
        }
    }
    for (int i = 0; (int)rings[1].size(); i++) {
        if (rings[1][i] != 0) {
#ifdef DEBUG_NEMA
            std::cout << "rings[1][" << i << "] = " << rings[1][i] << std::endl;
#endif
            activeRing2Index = i;
            activeRing2Phase = rings[1][activeRing2Index];
            break;
        }
    }
    int initialIndexRing [2] = { activeRing1Index, activeRing2Index};
    // calculate force offs
    for (int ringNumber = 0; ringNumber < 2; ringNumber++) {
        int length = (int)rings[ringNumber].size();
        int aPhaseNumber = rings[ringNumber][initialIndexRing[ringNumber]];
        int aPhaseIndex = aPhaseNumber - 1;
        int nPhaseIndex = aPhaseIndex; //next phase
        int nPhaseNumber = aPhaseNumber;
        forceOffs[aPhaseNumber - 1] = maxGreen[aPhaseNumber - 1];

#ifdef DEBUG_NEMA
        std::cout << "Phase  " << aPhaseNumber << ": force off " << forceOffs[aPhaseNumber - 1] << std::endl;
#endif
        for (int i = initialIndexRing[ringNumber] + 1; i < length; i++) {
            nPhaseNumber = rings[ringNumber][i];
            nPhaseIndex = nPhaseNumber - 1;
            // std::cout <<" ring "<<ringNumber <<" i: "<<i<< " phase: "<<nPhaseNumber<< std::endl;
            if (nPhaseNumber != 0) {
                forceOffs[nPhaseIndex] = forceOffs[aPhaseIndex] + maxGreen[nPhaseIndex] + yellowTime[aPhaseIndex] + redTime[aPhaseIndex];
                aPhaseNumber = nPhaseNumber;
                aPhaseIndex = nPhaseIndex;

#ifdef DEBUG_NEMA
                std::cout << "- Phase " << aPhaseNumber << ": force off " << forceOffs[aPhaseIndex] << std::endl;
#endif
            }
        }
    }

    // calculate initial phases based on in cycle clock
    for (int ringNumber = 0; ringNumber < 2; ringNumber++) {
        int length = (int)rings[ringNumber].size();
        for (int i = initialIndexRing[ringNumber]; i < length; i++) {
            int aPhaseIndex = rings[ringNumber][i] - 1;
            if (aPhaseIndex != -1) {
                phaseCutOffs[aPhaseIndex] = forceOffs[aPhaseIndex] - minGreen[aPhaseIndex];
#ifdef DEBUG_NEMA
                std::cout << "Phase " << aPhaseIndex + 1 << " cut off is " << phaseCutOffs[aPhaseIndex] << std::endl;
#endif
            }
        }
    }

    // find the current in cycle time
    SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    double currentTimeInSecond = STEPS2TIME(now);
    double currentInCycleTime = ModeCycle(currentTimeInSecond - cycleRefPoint - offset, myCycleLength);

    // find the initial phases
    for (int ringNumber = 0; ringNumber < 2; ringNumber++) {
        int length = (int)rings[ringNumber].size();
        int aPhaseIndex = -1;
        bool found = false;
        for (int i = initialIndexRing[ringNumber]; i < length; i++) {
            aPhaseIndex = rings[ringNumber][i] - 1;
            if (aPhaseIndex != -1) {
                if (currentInCycleTime < phaseCutOffs[aPhaseIndex]) {
#ifdef DEBUG_NEMA
                    std::cout << "current in cycle time=" << currentInCycleTime << " phase: " << aPhaseIndex << std::endl;
#endif
                    found = true;
                    break;
                }
            }
        }
        if (!found) {
            aPhaseIndex = rings[ringNumber][initialIndexRing[ringNumber]] - 1; // if the break didn't get triggered, go back to the beginning.
        }
#ifdef DEBUG_NEMA
        std::cout << "current in cycle time=" << currentInCycleTime << " ring " << ringNumber << " aphase: " << aPhaseIndex + 1 << std::endl;
#endif
        if (ringNumber == 0) {
            activeRing1Index = aPhaseIndex;
            activeRing1Phase = activeRing1Index + 1;
        } else {
            activeRing2Index = aPhaseIndex;
            activeRing2Phase = activeRing2Index + 1;
        }
    }



#ifdef DEBUG_NEMA
    //print to check the rings and barriers active phase
    std::cout << "After init, active ring1 phase is " << activeRing1Phase << std::endl;
    std::cout << "After init, active ring2 phase is " << activeRing2Phase << std::endl;


    //print to check the phase definition is correct
    std::cout << "Print to check NEMA phase definitions\n";
    for (auto p : myPhases) {
        std::cout << "index = " << p->getName() << "; ";
        std::cout << "duration (useless) = " << time2string(p->duration) << "; ";
        std::cout << "minDur = " << time2string(p->minDuration) << "; ";
        std::cout << "maxDur = " << time2string(p->maxDuration) << "; ";
        std::cout << "vehext = " << time2string(p->vehext) << "; ";
        std::cout << "yellow = " << time2string(p->yellow) << "; ";
        std::cout << "red = " << time2string(p->red) << "; ";
        std::cout << "state = " << p->getState() << std::endl;
    }
#endif

    //init the traffic light
    MSTrafficLightLogic::init(nb);
    assert(myLanes.size() > 0);
    //iterate through the lanes and build one E2 detector for each lane associated with the traffic light control junction
    for (const LaneVector& lanes : myLanes) {
        for (MSLane* const lane : lanes) {
            //decide the detector length
            double detector_length = 0;
            if (isLeftTurnLane(lane)) {
                detector_length = myDetectorLengthLeftTurnLane;
            } else {
                detector_length = myDetectorLength;
            }
            if (noVehicles(lane->getPermissions())) {
                // do not build detectors on green verges or sidewalks
                continue;
            }
            // Build detector and register them in the detector control
            if (myLaneDetectorMap.find(lane) == myLaneDetectorMap.end()) {
                MSE2Collector* det = nullptr;
                const std::string customID = getParameter(lane->getID());
                if (customID != "") {
                    det = dynamic_cast<MSE2Collector*>(MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_LANE_AREA_DETECTOR).get(customID));
                    if (det == nullptr) {
                        WRITE_ERROR("Unknown laneAreaDetector '" + customID + "' given as custom detector for NEMA tlLogic '" + getID() + "', program '" + getProgramID() + ".");
                        continue;
                    }
                    //set the detector to be visible in gui
                    det->setVisible(myShowDetectors);
                } else {
                    std::string id = "TLS_" + myID + "_" + myProgramID + "_E2DetectorOnLane_" + lane->getID();
                    // std::cout << "The detectorID = " << id << std::endl;
                    //createE2Detector() method will lead to bad detector showing in sumo-gui
                    //so it is better to use build2Detector() rather than createE2Detector()
                    // det = nb.createE2Detector(id, DU_TL_CONTROL, lane, INVALID_POSITION, lane->getLength(), myDetectorLength, 0, 0, 0, myVehicleTypes, myShowDetectors);
                    // MSNet::getInstance()->getDetectorControl().add(SUMO_TAG_LANE_AREA_DETECTOR, det, myFile, myFreq);
                    nb.buildE2Detector(id, //detectorID
                                       lane, //lane to build this detector
                                       INVALID_POSITION, // set the detector location by end point and length, so this one is set to invalue value so this parameter can be passed
                                       lane->getLength(), // set the end position of the detector at the end of the lane, which is right at the position of stop bar of a junction
                                       detector_length, //detector length
                                       myFile, // detector information output file
                                       myFreq, // detector reading interval
                                       0, // time-based threshold that decribes how much time has to pass until a vehicle is considerred as halting
                                       0, // speed threshold as halting
                                       0, // minimum dist to the next standing vehicle to make this vehicle count as a participant to the jam
                                       myVehicleTypes, //vehicle types to consider, if it is empty, meaning consider all types of vehicles
                                       false, // detector position check. More details could be found on SUMO web
                                       myShowDetectors, // whether to show detectors in sumo-gui
                                       0, //traffic light that triggers aggregation when swithing
                                       0); // outgoing lane that associated with the traffic light

                    //get the detector to be used in the lane detector map loading
                    det = dynamic_cast<MSE2Collector*>(MSNet::getInstance()->getDetectorControl().getTypedDetectors(SUMO_TAG_LANE_AREA_DETECTOR).get(id));
                }
                // print to check
                // std::cout << "E2Detector " << det->getID() << " is built on laneID = " << lane->getID() << std::endl;

                //map the detector to lane and lane to detector
                myLaneDetectorMap[lane] = det;
                myDetectorLaneMap[det] = lane;
                myDetectorInfoVector.push_back(DetectorInfo(det, (int)myPhases.size()));

            }
        }
    }
    //map NEMA phase to detectors
    // std::cout << "start of NEMA phase to detector map building " << std::endl;
    for (auto item : phase2ControllerLanesMap) {
        int NEMAPhaseIndex = item.first;
        std::vector<std::string> laneIDs = item.second;
        std::vector<MSE2Collector*> detectors;
        MSE2Collector* detector = nullptr;
        for (std::string laneID : laneIDs) {
            MSLane* lane = MSLane::dictionary(laneID);
            detector = myLaneDetectorMap[lane];
            detectors.push_back(detector);
        }
        phase2DetectorMap.find(NEMAPhaseIndex) -> second.detectors = detectors;
    }
#ifdef DEBUG_NEMA
    // print to check phase2DetectorMap
    std::cout << "Print to check phase2DetectorMap" << std::endl;
    for (auto item : phase2DetectorMap) {
        std::cout << "The NEMA phase index = " << item.first << " has detectors: \n";
        for (auto det : item.second.detectors) {
            std::cout << '\t' << det->getID() << std::endl;
        }
    }
#endif

    R1State = activeRing1Phase;
    R2State = activeRing2Phase;

    // set the next phase to current for initialization
    myNextPhaseR1 = R1State;
    myNextPhaseR2 = R2State;

    // std::cout << "After init, R1State = " << R1State << std::endl;
    // std::cout << "After init, R2State = " << R2State << std::endl;

    R1RYG = GREEN;
    R2RYG = GREEN;

    wait4R1Green = false;
    wait4R2Green = false;

    r1barrier = barrierPhaseIndecies[0];
    r2barrier = barrierPhaseIndecies[1];

    r1coordinatePhase = coordinatePhaseIndecies[0];
    r2coordinatePhase = coordinatePhaseIndecies[1];

    // Create the barrier to phase mapping;
    constructBarrierMap(0, myRingBarrierMapping[0]);
    constructBarrierMap(1, myRingBarrierMapping[1]);


#ifdef DEBUG_NEMA
    std::cout << "After init, r1/r2 barrier phase = " << r1barrier << " and " << r2barrier << std::endl;
    std::cout << "After init, r1/r2 coordinate phase = " << r1coordinatePhase << " and " << r2coordinatePhase << std::endl;
#endif


    currentState = "";
    // currentR1State = myPhases[R1State - 1]->getState();
    // currentR2State = myPhases[R2State - 1]->getState();
    for (const MSPhaseDefinition* const p : myPhases) {
        if (R1State == string2int(p->getName())) {
            currentR1State = p->getState();
        }
        if (R2State == string2int(p->getName())) {
            currentR2State = p->getState();
        }
    }
#ifdef DEBUG_NEMA
    std::cout << "R1State = " << R1State << " and its state = " << currentR1State << std::endl;
    std::cout << "R2State = " << R2State << " and its state = " << currentR2State << std::endl;
#endif

    //Do not delete. SUMO traffic logic check.
    //SUMO check begin
    const SVCPermissions motorized = ~(SVC_PEDESTRIAN | SVC_BICYCLE);
    std::map<int, std::set<MSE2Collector*>> linkToDetectors;
    std::set<int> actuatedLinks;

    const int numLinks = (int)myLinks.size();
    std::vector<bool> neverMajor(numLinks, true);
    for (const MSPhaseDefinition* phase : myPhases) {
        const std::string& state = phase->getState();
        for (int i = 0; i < numLinks; i++) {
            if (state[i] == LINKSTATE_TL_GREEN_MAJOR) {
                neverMajor[i] = false;
            }
        }
    }
    std::vector<bool> oneLane(numLinks, false);
    for (int i = 0; i < numLinks; i++) {
        for (MSLane* lane : getLanesAt(i)) {
            int numMotorized = 0;
            for (MSLane* l : lane->getEdge().getLanes()) {
                if ((l->getPermissions() & motorized) != 0) {
                    numMotorized++;
                }
            }
            if (numMotorized == 1) {
                oneLane[i] = true;
                break;
            }
        }
    }

    for (const MSPhaseDefinition* phase : myPhases) {
        const int phaseIndex = (int)myDetectorForPhase.size();
        std::set<MSE2Collector*> detectors;
        if (phase->isActuted()) {
            const std::string& state = phase->getState();
            std::set<int> greenLinks;
            std::map<MSE2Collector*, std::set<int>> detectorLinks;

            for (int i = 0; i < numLinks; i++)  {
                if (state[i] == LINKSTATE_TL_GREEN_MAJOR
                        || (state[i] == LINKSTATE_TL_GREEN_MINOR
                            && ((neverMajor[i]  // check1a
                                 && hasMajor(state, getLanesAt(i))) // check1b
                                || oneLane[i])) // check1c
                   ) {
                    greenLinks.insert(i);
                    actuatedLinks.insert(i);
                }

                for (MSLane* lane : getLanesAt(i)) {
                    if (myLaneDetectorMap.count(lane) != 0) {
                        detectorLinks[myLaneDetectorMap[lane]].insert(i);
                    }
                }
            }
            for (auto& item : detectorLinks) {
                MSE2Collector* det = item.first;
                MSLane* detectorLane = myDetectorLaneMap[det];
                bool usable = true;
                // check 1
                for (int j : item.second) {
                    if (greenLinks.count(j) == 0) {
                        usable = false;
                    }
                }

                //check 2
                if (usable) {
                    for (MSLink* link : detectorLane->getLinkCont()) {
                        MSLane* next = link->getLane();
                        if (myLaneDetectorMap.count(next) != 0) {
                            MSE2Collector* nextDet = myLaneDetectorMap[next];
                            for (int j : detectorLinks[nextDet]) {
                                if (greenLinks.count(j) == 0) {
                                    usable = false;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (usable) {
                    detectors.insert(item.first);
                    for (int j : item.second) {
                        linkToDetectors[j].insert(item.first);
                    }
                }
            }
            if (detectors.size() == 0) {
                WRITE_WARNINGF("At NEMA tlLogic '%', actuated phase % has no controlling detector", getID(), toString(phaseIndex));
            }
        }
        std::vector<DetectorInfo*> detectorInfos;
        myDetectorForPhase.push_back(detectorInfos);
        for (MSE2Collector* det : detectors) {
            for (DetectorInfo& detInfo : myDetectorInfoVector) {
                if (detInfo.det == det) {
                    myDetectorForPhase.back().push_back(&detInfo);
                    detInfo.servedPhase[phaseIndex] = true;
                }
            }
        }
    }

    for (int i : actuatedLinks) {
        if (linkToDetectors[i].size() == 0 && myLinks[i].size() > 0
                && (myLinks[i].front()->getLaneBefore()->getPermissions() & motorized) != 0) {
            WRITE_WARNINGF("At NEMA tlLogic '%, linkIndex % has no controlling detector", getID(), toString(i));
        }
    }

    std::string state1 = transitionState(currentR1State, GREEN);
    std::string state2 = transitionState(currentR2State, GREEN);
    myPhase.setState(combineStates(state1, state2));
    myPhase.setName(toString(activeRing1Phase) + "+" + toString(activeRing2Phase));


    //validating timing
    validate_timing();
#ifdef DEBUG_NEMA
    //std::cout << "reach the end of init()\n";
#endif
}

void
NEMALogic::validate_timing() {
    const bool ignoreErrors = StringUtils::toBool(getParameter("ignore-errors", "false"));
    //check cycle length
    for (int ringIndex = 0; ringIndex <= 1; ringIndex++) {
        int lastPhasePositionIndex = (int)rings[ringIndex].size() - 1;
        int lastPhase = rings[ringIndex][lastPhasePositionIndex];
        if (lastPhase == 0) { //should be enough
            lastPhase = rings[ringIndex][lastPhasePositionIndex - 1];
        }
        int lastPhaseIndex = lastPhase - 1;
        double cycleLengthCalculated = forceOffs[lastPhaseIndex] + yellowTime[lastPhaseIndex] + redTime[lastPhaseIndex];
        if (coordinateMode && cycleLengthCalculated != myCycleLength) {
            int ringNumber = ringIndex + 1;
            const std::string error = "At NEMA tlLogic '" + getID() + "', Ring " + toString(ringNumber) + " does not add to cycle length.";
            if (ignoreErrors) {
                WRITE_WARNING(error);
            } else {
                throw  ProcessError(error);
            }
        }
    }
    // check barriers
    double ring1barrier1_length = forceOffs[r1barrier - 1] + yellowTime[r1barrier - 1] + redTime[r1barrier - 1];
    double ring2barrier1_length = forceOffs[r2barrier - 1] + yellowTime[r2barrier - 1] + redTime[r2barrier - 1];
    if (ring1barrier1_length != ring2barrier1_length) {
        const std::string error = "At NEMA tlLogic '" + getID() + "', the phases before barrier 1 from both rings do not add up. (ring1="
                                  + toString(ring1barrier1_length) + ", ring2=" + toString(ring2barrier1_length) + ")";
        if (coordinateMode && !ignoreErrors) {
            throw  ProcessError(error);
        } else {
            WRITE_WARNING(error);
        }
    }
    double ring1barrier2_length = forceOffs[r2coordinatePhase - 1] + yellowTime[r2coordinatePhase - 1] + redTime[r2coordinatePhase - 1];
    double ring2barrier2_length = forceOffs[r1coordinatePhase - 1] + yellowTime[r1coordinatePhase - 1] + redTime[r1coordinatePhase - 1];
    if (ring1barrier2_length != ring2barrier2_length) {
        const std::string error = "At NEMA tlLogic '" + getID() + "', the phases before barrier 2 from both rings do not add up. (ring1="
                                  + toString(ring1barrier2_length) + ", ring2=" + toString(ring2barrier2_length) + ")";
        if (coordinateMode && !ignoreErrors) {
            throw  ProcessError(error);
        } else {
            WRITE_WARNING(error);
        }
    }
    // no offset for non coordinated
    if (!coordinateMode && offset != 0) {
        WRITE_WARNINGF("NEMA tlLogic '%' is not coordinated but an offset was set.", getID());
    }
}

void
NEMALogic::setNewSplits(std::vector<double> newSplits) {
    assert(newSplits.size() == 8);
    for (int i = 0; i < 8; i++) {
        nextMaxGreen[i] = newSplits[i] - yellowTime[i] - redTime[i];
    }
}


void
NEMALogic::setNewMaxGreens(std::vector<double> newMaxGreens) {
    for (int i = 0; i < 8; i++) {
        nextMaxGreen[i] = newMaxGreens[i];
    }
}


void
NEMALogic::setNewCycleLength(double newCycleLength) {
    myNextCycleLength = newCycleLength;
}


void
NEMALogic::setNewOffset(double newOffset) {
    myNextOffset = newOffset;
}

//helper methods

std::vector<int> NEMALogic::readParaFromString(std::string s) {
    std::vector<int> output;
    for (char c : s) {
        if (c >= '0' && c <= '9') {
            int temp = c - '0';
            output.push_back(temp);
        }
    }
    return output;
}

std::vector<std::string> NEMALogic::string2vector(std::string s) {
    std::vector<std::string> output;
    std::stringstream ss(s);
    while (ss.good()) {
        std::string substr;
        std::getline(ss, substr, ',');
        output.push_back(substr);
    }
#ifdef DEBUG_NEMA
    //print to check
    for (auto i : output) {
        std::cout << i << std::endl;
    }
#endif
    return output;
}

std::string NEMALogic::combineStates(std::string state1, std::string state2) {
    std::string output = "";
    if (state1.size() != state2.size()) {
        throw ProcessError("At NEMA tlLogic '" + getID() + "', different sizes of NEMA phase states. Please check the NEMA XML");
    }
    for (int i = 0; i < (int)state1.size(); i++) {
        char ch1 = state1[i];
        char ch2 = state2[i];

        // check through this order. 'G' overwrite 'g'.
        if (ch1 == 'G' || ch2 == 'G') {
            output += 'G';
        } else if (ch1 == 'g' || ch2 == 'g') {
            output += 'g';
        } else if (ch1 == 's' || ch2 == 's') {
            output += 's';
        } else if (ch1 == 'y' || ch2 == 'y') {
            output += 'y';
        } else if (ch1 == 'u' || ch2 == 'u') {
            output += 'u';
        } else if (ch1 == 'O' || ch2 == 'O') {
            output += 'O';
        } else if (ch1 == 'o' || ch2 == 'o') {
            output += 'o';
        } else {
            output += 'r';
        }
    }
    return output;
}

bool NEMALogic::isDetectorActivated(int phaseNumber, int depth = 0) const {
    if (phase2DetectorMap.find(phaseNumber) == phase2DetectorMap.end()) {
        return false;
    } else {
        auto const& detectInfo = phase2DetectorMap.find(phaseNumber)->second;
        if ((phaseNumber != R1State) && (phaseNumber != R2State) && depth < 1) {
            // If I am not the active phase & my target is an active phase, don't report when I am called for my own phase
            if ((detectInfo.cpdTarget == R1State && R1RYG >= GREEN) || (detectInfo.cpdTarget == R2State && R2RYG >= GREEN)) {
                return false;
            }
        }
        for (auto det : detectInfo.detectors) {
            if (det->getCurrentVehicleNumber() > 0) {
                return true;
            }
        }
        if (detectInfo.cpdSource > 0 && depth < 1) {
            return isDetectorActivated(detectInfo.cpdSource, depth + 1);
        }
        return false;
    }
}


std::map<std::string, double>
NEMALogic::getDetectorStates() const {
    std::map<std::string, double> result;
    for (auto item : myDetectorLaneMap) {
        result[item.first->getID()] = item.first->getCurrentVehicleNumber();
    }
    return result;
}


const MSPhaseDefinition&
NEMALogic::getCurrentPhaseDef() const {
    return myPhase;
}

SUMOTime
NEMALogic::trySwitch() {
    const std::string newState = NEMA_control();
    if (newState != myPhase.getState()) {
        myPhase.setState(newState);
        // ensure that SwitchCommand::execute notices a change
        myStep = 1 - myStep;
    }
    //std::cout << SIMTIME << " " << myPhase.getState() << "\n";
    return TIME2STEPS(TS);
}


std::string
NEMALogic::NEMA_control() {
    std::string outputState = "";
    //controller starts
    SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    double currentTimeInSecond = STEPS2TIME(now);

#ifdef DEBUG_NEMA
    //print to check
    //I didn't use getTimeInCycle(). This is because the cycle reference point may change in the future.
    double currentInCycleTime = ModeCycle(currentTimeInSecond - cycleRefPoint - offset, myCycleLength);
    std::cout << "current time in cycle:\t" << currentInCycleTime << "\t" << "phases: " << R1State << '\t' << R2State << std::endl;
#endif
    //int R1Phase = activeRing1Phase;
    int R1Phase = R1State;
    int R1Index = R1Phase - 1;

    double durationR1 = currentTimeInSecond - phaseStartTime[R1Index];
    double phaseStartTimeInCycleR1 = ModeCycle(phaseStartTime[R1Index] - cycleRefPoint - offset, myCycleLength);
    //ensure minGreen for each phase
    if (maxRecalls[R1Index]) {
        phaseExpectedDuration[R1Index] = maxGreen[R1Index];
    } else {
        phaseExpectedDuration[R1Index] = MAX2(phaseExpectedDuration[R1Index], minGreen[R1Index]);
    }
    if (((R1Phase != r1coordinatePhase) || (vehExt[R1Index] > 0 && !coordinateMode)) && (R1RYG == GREEN || R1RYG == GREENREST)) {
        if (isDetectorActivated(R1Phase)) {
            phaseExpectedDuration[R1Index] = MAX2(phaseExpectedDuration[R1Index], durationR1 + vehExt[R1Index]);
            if (fixForceOff) {
                phaseExpectedDuration[R1Index] = MIN2(phaseExpectedDuration[R1Index], ModeCycle(forceOffs[R1Index] - phaseStartTimeInCycleR1, myCycleLength));
#ifdef DEBUG_NEMA
                std::cout << "R1 phase " << R1State << " forceOff " << forceOffs[R1Index] << "\tphase start " << phaseStartTimeInCycleR1 << std::endl;
#endif
            } else {
                phaseExpectedDuration[R1Index] = MIN2(phaseExpectedDuration[R1Index], maxGreen[R1Index]);
            }
        }
    }

    int R2Phase = R2State;
    int R2Index = R2Phase - 1;
    double durationR2 = currentTimeInSecond - phaseStartTime[R2Index];
    double phaseStartTimeInCycleR2 = ModeCycle(phaseStartTime[R2Index] - cycleRefPoint - offset, myCycleLength);

    if (maxRecalls[R2Index]) {
        phaseExpectedDuration[R2Index] = maxGreen[R2Index];
    } else {
        phaseExpectedDuration[R2Index] = MAX2(phaseExpectedDuration[R2Index], minGreen[R2Index]);
    }
    if ((((R2Phase != r2coordinatePhase && R2Phase >= 5) || ((R2Phase >= 5 && vehExt[R2Index] > 0) && !coordinateMode)) && (R2RYG == GREEN || R2RYG == GREENREST))) {
        if (isDetectorActivated(R2Phase)) {
            phaseExpectedDuration[R2Index] = MAX2(phaseExpectedDuration[R2Index], durationR2 + vehExt[R2Index]);
            if (fixForceOff) {
                phaseExpectedDuration[R2Index] = MIN2(phaseExpectedDuration[R2Index], ModeCycle(forceOffs[R2Index] - phaseStartTimeInCycleR2, myCycleLength));
#ifdef DEBUG_NEMA
                std::cout << "R2 phase " << R1State << " forceOff " << forceOffs[R2Index] << "\tphase start " << phaseStartTimeInCycleR2 << std::endl;
#endif
            } else {
                phaseExpectedDuration[R2Index] = MIN2(phaseExpectedDuration[R2Index], maxGreen[R2Index]);
            }
        }
    }

    bool EndCurrentPhaseR1 = false;
    bool EndCurrentPhaseR2 = false;
    if (durationR1 >= phaseExpectedDuration[R1Index]) {
        EndCurrentPhaseR1 = true;
    }
    if (durationR2 >= phaseExpectedDuration[R2Index]) {
        EndCurrentPhaseR2 = true;
    }
    // Green rest can always transition, even if it is at the barrier
    if (EndCurrentPhaseR1 && (R1Phase == r1barrier)) {
        if ((!EndCurrentPhaseR2  || R2RYG < GREEN) && R1RYG != GREENREST) {
            EndCurrentPhaseR1 = false;
        }
    }
    if (EndCurrentPhaseR1 && (R1Phase == r1coordinatePhase)) {
        if ((!EndCurrentPhaseR2 || R2RYG < GREEN) && R1RYG != GREENREST) {
            EndCurrentPhaseR1 = false;
        }
    }
    if (EndCurrentPhaseR2 && (R2Phase == r2barrier)) {
        if ((!EndCurrentPhaseR1 || R1RYG < GREEN) && R2RYG != GREENREST) {
            EndCurrentPhaseR2 = false;
        }
    }
    if (EndCurrentPhaseR2 && (R2Phase == r2coordinatePhase)) {
        if ((!EndCurrentPhaseR1 || R1RYG < GREEN) && R2RYG != GREENREST) {
            EndCurrentPhaseR2 = false;
        }
    }

    // Falling Edge of Green
    if (EndCurrentPhaseR1 && (!wait4R1Green)) {
        phaseEndTimeR1 = currentTimeInSecond;
        phaseExpectedDuration[R1Index] = 0;
        wait4R1Green = true;
    }
    if (EndCurrentPhaseR2 && (!wait4R2Green)) {
        phaseEndTimeR2 = currentTimeInSecond;
        phaseExpectedDuration[R2Index] = 0;
        wait4R2Green = true;
    }

    // Logic for Green Rest & Green Transfer
    // This requires a detector check. It should only be entered when the lights are green
    // This logic doesn't need to enter at all if in coordinated mode and greenTransfer is disabled
    if (((EndCurrentPhaseR1 && R1RYG >= GREEN) || (EndCurrentPhaseR2 && R2RYG >= GREEN)) && (!coordinateMode || greenTransfer)) {
        // Calculate the potential next phases.
        // Have to do it here and below because the "final" traffic light check is at the end of yellow
        int tempR1Phase;
        int tempR2Phase;
        // Get the next phases, with the first option being staying in the current phase
        std::tie(tempR1Phase, tempR2Phase) = getNextPhases(R1Phase, R2Phase, wait4R1Green, wait4R2Green, true);
        // entry point to green rest. First check detector status, then determine if this should be up next.
        // Green rest is effectively the same as being perpetually past the minimum green timer but not changing
        if ((tempR1Phase == R1Phase && EndCurrentPhaseR1) && (tempR2Phase == R2Phase && EndCurrentPhaseR2) && !coordinateMode) {
            // mark that the phases are not desired to end
            EndCurrentPhaseR1 = false;
            EndCurrentPhaseR2 = false;
            wait4R1Green = false;
            wait4R2Green = false;
            // Timing update. This logic should be checked the next step, so only add the simulation timestep.
            // Potential that this needs to be extended in the future.
            phaseEndTimeR1 += TS;
            phaseEndTimeR2 += TS;
            // setting the phase start time to current time - the minimum timer
            // will still allow the phase to be extended with vehicle detection
            phaseStartTime[R1Index] = currentTimeInSecond - minGreen[R1Index];
            phaseStartTime[R2Index] = currentTimeInSecond - minGreen[R2Index];

            // Set my state to Green Rest
            R1RYG = GREENREST;
            R2RYG = GREENREST;

        } else if (tempR1Phase == R1Phase && EndCurrentPhaseR1 && greenTransfer) {
            // This is the logic for green transfer on Ring 1
            // Green transfer occurs when current phase should end but there isn't a better one to go to,
            // even though the other phase might be transitioning
            if (!EndCurrentPhaseR2 || (tempR2Phase != R2Phase)) {
                EndCurrentPhaseR1 = false;
                wait4R1Green = false;
                phaseEndTimeR1 += TS;
                if ((R1Phase == r1barrier || R1Phase == r1coordinatePhase) && R1RYG != GREENREST) {
                    // If the "green transfer" is at the barrier, it can't actually move until the other phase is done
                    phaseEndTimeR1 = currentTimeInSecond + phaseExpectedDuration[tempR2Phase - 1];
                    phaseExpectedDuration[R1Index] = phaseExpectedDuration[tempR2Phase - 1];
                }
                R1RYG = R1RYG == GREENREST ? GREENREST : GREENTRANSFER;
            }
        } else if (tempR2Phase == R2Phase && EndCurrentPhaseR2 && greenTransfer) {
            if (!EndCurrentPhaseR1 || (tempR1Phase != R1Phase)) {
                // This is the logic for green transfer on Ring 2
                EndCurrentPhaseR2 = false;
                wait4R2Green = false;
                phaseEndTimeR2 += TS;
                if ((R2Phase == r2barrier || R2Phase == r2coordinatePhase) && R2RYG != GREENREST) {
                    // If the "green transfer" is at the barrier, it can't actually move until the other phase is done
                    phaseEndTimeR2 = currentTimeInSecond + phaseExpectedDuration[tempR1Phase - 1];
                    phaseExpectedDuration[R2Index] = phaseExpectedDuration[tempR1Phase - 1];
                }
                R2RYG = R2RYG == GREENREST ? GREENREST : GREENTRANSFER;
            }
        }
    }


    // Calculate the next phase with knowledge of both rings
    // Next Phase should be calculated on the falling edge of yellow
    bool calculate = false;
    if (wait4R1Green || wait4R2Green) {
        if ((currentTimeInSecond - phaseEndTimeR1 >= yellowTime[R1Index]) && (R1RYG == YELLOW)) {
            R1RYG = RED; //red
            calculate = true;
        }
        if ((currentTimeInSecond - phaseEndTimeR2 >= yellowTime[R2Index]) && (R2RYG == YELLOW)) {
            R2RYG = RED; //red
            calculate = true;
        }
        if (calculate) {
            std::tie(myNextPhaseR1, myNextPhaseR2) = getNextPhases(R1Phase, R2Phase, wait4R1Green, wait4R2Green);
        }
    }

    //enter transtion phase for Ring1
    if (wait4R1Green) {
        if (currentTimeInSecond - phaseEndTimeR1 < yellowTime[R1Index]) {
            // Not removing this if statement for clarity on the transition timing
            R1RYG = YELLOW; //yellow
        } else if (currentTimeInSecond - phaseEndTimeR1 < (yellowTime[R1Index] + redTime[R1Index])) {
            R1RYG = RED; //red
            // TODO: remove the 0.5 (it has timing issues with <1 timesteps)
            bool toUpdate = (currentTimeInSecond - phaseEndTimeR1) < yellowTime[R1Index] + 0.5;
            if (R1Phase == r1coordinatePhase && toUpdate) {
                for (int i = 0; i < 8; i++) {
                    maxGreen[i] = nextMaxGreen[i];
                }
                offset = myNextOffset;
                myCycleLength = myNextCycleLength;
            }
        } else {
            //next phase
            //time 10 R1Phase = 4. Checked
            // R1Phase = nextPhase(rings[0], R1Phase);
            R1Phase = myNextPhaseR1;
            //offset control not included for now
            R1RYG = GREEN; //green
            //update phaseStartTime
            phaseStartTime[R1Phase - 1] = currentTimeInSecond;

            R1State = R1Phase;
            if (R1Phase == r1coordinatePhase) {
                if (coordinateMode) {
                    phaseExpectedDuration[R1Phase - 1] = ModeCycle(myCycleLength - (currentTimeInSecond - cycleRefPoint - offset) - yellowTime[R1Phase - 1] - redTime[R1Phase - 1], myCycleLength);
                }
            }
            wait4R1Green = false;
        }
    }

    if (wait4R2Green) {
        if ((currentTimeInSecond - phaseEndTimeR2) < yellowTime[R2Index]) {
            R2RYG = YELLOW;
        } else if ((currentTimeInSecond - phaseEndTimeR2) < (yellowTime[R2Index] + redTime[R2Index])) {
            R2RYG = RED;
        } else {
            R2Phase = myNextPhaseR2;
            // R2Phase = nextPhase(rings[1], R2Phase);
            R2RYG = GREEN;
            //update phaseStartTime
            phaseStartTime[R2Phase - 1] = currentTimeInSecond;
            R2State = R2Phase;
            if (R2Phase == r2coordinatePhase) {
                if (coordinateMode) {
                    phaseExpectedDuration[R2Phase - 1] = ModeCycle(myCycleLength - (currentTimeInSecond - cycleRefPoint - offset) - yellowTime[R2Phase - 1] - redTime[R2Phase - 1], myCycleLength);
#ifdef DEBUG_NEMA
                    std::cout << "R2 phase " << R1Phase << " is coordinated and has an expected duration of " << phaseExpectedDuration[R1Phase - 1] << std::endl;
#endif
                }

            }
            wait4R2Green = false;
        }
    }


    std::string state1 = "";
    for (auto p : myPhases) {
        if (R1State == string2int(p->getName())) {
            state1 = p->getState();
        }
    }
    state1 = transitionState(state1, R1RYG);
    currentR1State = state1;

    std::string state2 = "";
    for (auto p : myPhases) {
        if (R2State == string2int(p->getName())) {
            state2 = p->getState();
        }
    }
    state2 = transitionState(state2, R2RYG);
    currentR2State = state2;

    outputState = combineStates(state1, state2);

    if (currentState != outputState) {
        currentState = outputState;
        if (whetherOutputState) {
            outputStateFile.open(outputStateFilePath, std::ios_base::app);
            outputStateFile << currentTimeInSecond << "\t" << currentState << std::endl;
            outputStateFile.close();
        }

    }
    myPhase.setName(toString(R1Phase) + "+" + toString(R2Phase));
    return outputState;
}

int NEMALogic::nextPhase(std::vector<int> ring, int currentPhase, int& distance, bool sameAllowed) {
    int length = (int)ring.size();
    int flag = 0;
    int nphase = 0; // next phase
    int i = 0; // i represents the distance
    int matching_i = 0;
    for (i = 0; i < length * 2; i++) {
        if (flag == 1) {
            if (ring[i % length] != 0) {
                int tempPhase = ring[i % length];
                if (recall[tempPhase - 1] || isDetectorActivated(tempPhase)) {
                    nphase = tempPhase;
                    break;
                }

#ifdef DEBUG_NEMA
                else {
                    std::cout << "phase " << tempPhase << " was skipped" << std::endl;
                }
#endif
            }
        }
        if (ring[i % length] == currentPhase) {
            flag = 1;
            matching_i = i;
        }
    }
    if (nphase != 0) {
        distance = i;
        return nphase;
    } else {
        // this should only occur in the subset
        if (sameAllowed) {
            distance = i;
            return ring[matching_i % length];
        } else {
            distance = i + 1;
            return ring[(matching_i + 1) % length];
        }
    }
}


void NEMALogic::constructBarrierMap(int ring, std::vector<std::vector<int>>& barrierMap) {
    int flag = 0;
    std::vector<int> barrierOne;
    std::vector<int> barrierTwo;
    for (int localPhase : rings[ring]) {
        if (!flag) {
            barrierOne.push_back(localPhase);
            if (((localPhase == r1coordinatePhase || localPhase == r1barrier) && ring == 0) || ((localPhase == r2coordinatePhase || localPhase == r2barrier) && ring == 1)) {
                flag = 1;
            };
        } else {
            barrierTwo.push_back(localPhase);
        }
    };
    barrierMap.push_back(barrierOne);
    barrierMap.push_back(barrierTwo);
}

int NEMALogic::findBarrier(int phase, int ring) {
    int barrier = 0;
    for (int localPhase : myRingBarrierMapping[ring][1]) {
        if (phase == localPhase) {
            barrier = 1;
            break;
        }
    }
    return barrier;
}


std::tuple<int, int> NEMALogic::getNextPhases(int R1Phase, int R2Phase, bool toUpdateR1, bool toUpdateR2, bool stayOk) {
    // Only 1 or both can be !toUpdate (otherwise we wouldn't be in this situation)
    int nextR1Phase = R1Phase;
    int nextR2Phase = R2Phase;
    if (!toUpdateR1) {
        int r1BarrierNum = findBarrier(R1Phase, 0);
        int d = 0;
        nextR2Phase = nextPhase(myRingBarrierMapping[1][r1BarrierNum], R2Phase, d, stayOk);
        // If we aren't updating both, the search range is only the subset of values on the same side of the barrier;
    } else if (!toUpdateR2) {
        int r2BarrierNum = findBarrier(R2Phase, 1);
        int d = 0;
        nextR1Phase = nextPhase(myRingBarrierMapping[0][r2BarrierNum], R1Phase, d, stayOk);
    } else {
        // Both can be updated. We should take the change requiring the least distance travelled around the loop,
        // and then recalculate the other ring if it is not in the same barrier
        int r1Distance = 0;
        int r2Distance = 0;
        nextR1Phase = nextPhase(rings[0], R1Phase, r1Distance, stayOk);
        nextR2Phase = nextPhase(rings[1], R2Phase, r2Distance, stayOk);
        int r1Barrier = findBarrier(nextR1Phase, 0);
        int r2Barrier = findBarrier(nextR2Phase, 1);
        if ((r1Distance <= r2Distance) && (r1Barrier != r2Barrier)) {
            nextR2Phase = nextPhase(myRingBarrierMapping[1][r1Barrier], R2Phase, r2Distance, stayOk);
        } else if ((r1Distance > r2Distance) && (r1Barrier != r2Barrier)) {
            nextR1Phase = nextPhase(myRingBarrierMapping[0][r2Barrier], R1Phase, r1Distance, stayOk);
        };
    };
    return std::make_tuple(nextR1Phase, nextR2Phase);
}

//b should be the base of mode
double NEMALogic::ModeCycle(double a, double b) {
    double c = a - b;
    while (c >= b) {
        c = c - b;
    }
    while (c < 0) { //should be minimum green (or may be  not)
        c += b;
    }
    return c;
}

std::string NEMALogic::transitionState(std::string curState, int RYG) {
    std::string newState = "";
    if (RYG >= GREEN) {
        //Green
        newState = curState;

    } else if (RYG == RED) {
        // red
        for (char ch : curState) {
            UNUSED_PARAMETER(ch);
            newState += 'r';
        }
    } else {
        // yellow
        for (char ch : curState) {
            if (ch == 'G' || ch == 'g') {
                newState += 'y';
            } else {
                newState += ch;
            }
        }
    }
    return newState;

}


std::set<std::string> NEMALogic::getLaneIDsFromNEMAState(std::string state) {
    std::set<std::string> output;
    const MSTrafficLightLogic::LinkVectorVector& linkV = MSNet::getInstance()->getTLSControl().get(myID).getActive()->getLinks();
    for (int i = 0; i < (int)state.size(); i++) {
        char ch = state[i];
        if (ch == 'G') {
            for (auto link : linkV[i]) {
                output.insert(link->getLaneBefore()->getID());
            }
        }
    }
    return output;
}

bool NEMALogic::isLeftTurnLane(const MSLane* const lane) const {
    const std::vector<MSLink*> links = lane->getLinkCont();
    if (links.size() == 1 && links.front()->getDirection() == LinkDirection::LEFT) {
        return true;;
    }
    return false;
}

bool
NEMALogic::hasMajor(const std::string& state, const LaneVector& lanes) const {
    for (int i = 0; i < (int)state.size(); i++) {
        if (state[i] == LINKSTATE_TL_GREEN_MAJOR) {
            for (MSLane* cand : getLanesAt(i)) {
                for (MSLane* lane : lanes) {
                    if (lane == cand) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}


void
NEMALogic::activateProgram() {
    MSTrafficLightLogic::activateProgram();
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(true);
    }
}

void
NEMALogic::deactivateProgram() {
    MSTrafficLightLogic::deactivateProgram();
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(false);
    }
}

void
NEMALogic::setShowDetectors(bool show) {
    myShowDetectors = show;
    for (auto& item : myLaneDetectorMap) {
        item.second->setVisible(myShowDetectors);
    }
}

int NEMALogic::string2int(std::string s) {
    std::stringstream ss(s);
    int ret = 0;
    ss >> ret;
    return ret;
}


const std::string
NEMALogic::getParameter(const std::string& key, const std::string defaultValue) const {
    if (StringUtils::startsWith(key, "NEMA.")) {
        if (key == "NEMA.phaseCall") {
            std::string out_str = std::to_string(isDetectorActivated(1));
            for (int i = 2; i <= 8; i++) {
                out_str += ",";
                out_str += std::to_string(isDetectorActivated(i));
            }
            return out_str;
        } else {
            throw InvalidArgument("Unsupported parameter '" + key + "' for NEMA controller '" + getID() + "'");
        }
    } else {
        return Parameterised::getParameter(key, defaultValue);
    }
}


void
NEMALogic::setParameter(const std::string& key, const std::string& value) {
    if (StringUtils::startsWith(key, "NEMA.")) {
        if (key == "NEMA.splits" || key == "NEMA.maxGreens") {
            //splits="2.0 3.0 4.0 5.0 2.0 3.0 4.0 5.0"
            const std::vector<std::string>& tmp = StringTokenizer(value).getVector();
            if (tmp.size() != 8) {
                throw InvalidArgument("Parameter '" + key + "' for NEMA controller '" + getID() + "' requires 8 space-separated values");
            }
            std::vector<double> timing;
            for (const std::string& s : tmp) {
                timing.push_back(StringUtils::toDouble(s));
            }
            if (key == "NEMA.maxGreens") {
                setNewMaxGreens(timing);
            } else {
                setNewSplits(timing);
            }
        } else if (key == "NEMA.cycleLength") {
            setNewCycleLength(StringUtils::toDouble(value));
        } else if (key == "NEMA.offset") {
            setNewOffset(StringUtils::toDouble(value));
        } else {
            throw InvalidArgument("Unsupported parameter '" + key + "' for NEMA controller '" + getID() + "'");
        }
    }
    Parameterised::setParameter(key, value);
}

void
NEMALogic::error_handle_not_set(std::string param_variable, std::string param_name) {
    if (param_variable == "") {
        throw InvalidArgument("Please set " + param_name + " for NEMA tlLogic '" + getID() + "'");
    }
}
