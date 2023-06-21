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
/// @file    GNEFrameModules.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Aug 2019
///
// Auxiliar class for GNEFrame Modules
/****************************************************************************/
#include <config.h>

#include <netedit/GNEApplicationWindow.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/changes/GNEChange_Children.h>
#include <netedit/elements/additional/GNEAccess.h>
#include <netedit/elements/additional/GNEBusStop.h>
#include <netedit/elements/additional/GNECalibrator.h>
#include <netedit/elements/additional/GNECalibratorFlow.h>
#include <netedit/elements/additional/GNEChargingStation.h>
#include <netedit/elements/additional/GNEClosingLaneReroute.h>
#include <netedit/elements/additional/GNEClosingReroute.h>
#include <netedit/elements/additional/GNEContainerStop.h>
#include <netedit/elements/additional/GNEDestProbReroute.h>
#include <netedit/elements/additional/GNEDetectorE1.h>
#include <netedit/elements/additional/GNEDetectorE1Instant.h>
#include <netedit/elements/additional/GNEDetectorE2.h>
#include <netedit/elements/additional/GNEDetectorE3.h>
#include <netedit/elements/additional/GNEDetectorEntryExit.h>
#include <netedit/elements/additional/GNEPOI.h>
#include <netedit/elements/additional/GNEParkingArea.h>
#include <netedit/elements/additional/GNEParkingAreaReroute.h>
#include <netedit/elements/additional/GNEParkingSpace.h>
#include <netedit/elements/additional/GNEPoly.h>
#include <netedit/elements/additional/GNERerouter.h>
#include <netedit/elements/additional/GNERerouterInterval.h>
#include <netedit/elements/additional/GNERouteProbReroute.h>
#include <netedit/elements/additional/GNERouteProbe.h>
#include <netedit/elements/additional/GNETAZ.h>
#include <netedit/elements/additional/GNETAZSourceSink.h>
#include <netedit/elements/additional/GNEVaporizer.h>
#include <netedit/elements/additional/GNEVariableSpeedSign.h>
#include <netedit/elements/additional/GNEVariableSpeedSignStep.h>
#include <netedit/elements/data/GNEDataInterval.h>
#include <netedit/elements/demand/GNEContainer.h>
#include <netedit/elements/demand/GNEPerson.h>
#include <netedit/elements/demand/GNEPersonTrip.h>
#include <netedit/elements/demand/GNERide.h>
#include <netedit/elements/demand/GNERoute.h>
#include <netedit/elements/demand/GNEStop.h>
#include <netedit/elements/demand/GNETranship.h>
#include <netedit/elements/demand/GNETransport.h>
#include <netedit/elements/demand/GNEVehicle.h>
#include <netedit/elements/demand/GNEVType.h>
#include <netedit/elements/demand/GNEVTypeDistribution.h>
#include <netedit/elements/demand/GNEWalk.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <utils/foxtools/MFXMenuHeader.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <utils/gui/windows/GUIAppEnum.h>

#include "GNEFrameModules.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEFrameModules::TagSelector) TagSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_TAG_SELECTED,        GNEFrameModules::TagSelector::onCmdSelectTag)
};

FXDEFMAP(GNEFrameModules::DemandElementSelector) DemandElementSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SET_TYPE,    GNEFrameModules::DemandElementSelector::onCmdSelectDemandElement),
};

FXDEFMAP(GNEFrameModules::HierarchicalElementTree) HierarchicalElementTreeMap[] = {
    FXMAPFUNC(SEL_COMMAND,              MID_GNE_CENTER,                     GNEFrameModules::HierarchicalElementTree::onCmdCenterItem),
    FXMAPFUNC(SEL_COMMAND,              MID_GNE_INSPECT,                    GNEFrameModules::HierarchicalElementTree::onCmdInspectItem),
    FXMAPFUNC(SEL_COMMAND,              MID_GNE_DELETE,                     GNEFrameModules::HierarchicalElementTree::onCmdDeleteItem),
    FXMAPFUNC(SEL_COMMAND,              MID_GNE_ACHIERARCHY_MOVEUP,         GNEFrameModules::HierarchicalElementTree::onCmdMoveItemUp),
    FXMAPFUNC(SEL_COMMAND,              MID_GNE_ACHIERARCHY_MOVEDOWN,       GNEFrameModules::HierarchicalElementTree::onCmdMoveItemDown),
    FXMAPFUNC(SEL_RIGHTBUTTONRELEASE,   MID_GNE_ACHIERARCHY_SHOWCHILDMENU,  GNEFrameModules::HierarchicalElementTree::onCmdShowChildMenu)
};

FXDEFMAP(GNEFrameModules::DrawingShape) DrawingShapeMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_STARTDRAWING,   GNEFrameModules::DrawingShape::onCmdStartDrawing),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_STOPDRAWING,    GNEFrameModules::DrawingShape::onCmdStopDrawing),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_ABORTDRAWING,   GNEFrameModules::DrawingShape::onCmdAbortDrawing)
};

FXDEFMAP(GNEFrameModules::OverlappedInspection) OverlappedInspectionMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_NEXT,            GNEFrameModules::OverlappedInspection::onCmdNextElement),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_PREVIOUS,        GNEFrameModules::OverlappedInspection::onCmdPreviousElement),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_SHOWLIST,        GNEFrameModules::OverlappedInspection::onCmdShowList),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_OVERLAPPED_ITEMSELECTED,    GNEFrameModules::OverlappedInspection::onCmdListItemSelected),
    FXMAPFUNC(SEL_COMMAND,  MID_HELP,                           GNEFrameModules::OverlappedInspection::onCmdOverlappingHelp)
};

FXDEFMAP(GNEFrameModules::PathCreator) PathCreatorMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGEPATH_ABORT,          GNEFrameModules::PathCreator::onCmdAbortPathCreation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGEPATH_FINISH,         GNEFrameModules::PathCreator::onCmdCreatePath),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGEPATH_REMOVELAST,     GNEFrameModules::PathCreator::onCmdRemoveLastElement),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGEPATH_SHOWCANDIDATES, GNEFrameModules::PathCreator::onCmdShowCandidateEdges)
};


// Object implementation
FXIMPLEMENT(GNEFrameModules::TagSelector,                FXGroupBoxModule,     TagSelectorMap,                 ARRAYNUMBER(TagSelectorMap))
FXIMPLEMENT(GNEFrameModules::DemandElementSelector,      FXGroupBoxModule,     DemandElementSelectorMap,       ARRAYNUMBER(DemandElementSelectorMap))
FXIMPLEMENT(GNEFrameModules::HierarchicalElementTree,    FXGroupBoxModule,     HierarchicalElementTreeMap,     ARRAYNUMBER(HierarchicalElementTreeMap))
FXIMPLEMENT(GNEFrameModules::DrawingShape,               FXGroupBoxModule,     DrawingShapeMap,                ARRAYNUMBER(DrawingShapeMap))
FXIMPLEMENT(GNEFrameModules::OverlappedInspection,       FXGroupBoxModule,     OverlappedInspectionMap,        ARRAYNUMBER(OverlappedInspectionMap))
FXIMPLEMENT(GNEFrameModules::PathCreator,                FXGroupBoxModule,     PathCreatorMap,                 ARRAYNUMBER(PathCreatorMap))


// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNEFrameModules::TagSelector - methods
// ---------------------------------------------------------------------------

GNEFrameModules::TagSelector::TagSelector(GNEFrame* frameParent, GNETagProperties::TagType type, SumoXMLTag tag, bool onlyDrawables) :
    FXGroupBoxModule(frameParent->myContentFrame, "Element"),
    myFrameParent(frameParent),
    myTagType(type),
    myCurrentTemplateAC(nullptr) {
    // Create MFXIconComboBox
    myTagsMatchBox = new MFXIconComboBox(getCollapsableFrame(), GUIDesignComboBoxNCol, this, MID_GNE_TAG_SELECTED, GUIDesignComboBox);
    // set current tag type without notifying
    setCurrentTagType(myTagType, onlyDrawables, false);
    // set current tag without notifying
    setCurrentTag(tag, false);
    // TagSelector is always shown
    show();
}


GNEFrameModules::TagSelector::~TagSelector() {
    // clear myACTemplates and myTagsMatchBox
    for (const auto& ACTemplate : myACTemplates) {
        delete ACTemplate;
    }
    myACTemplates.clear();
}


void
GNEFrameModules::TagSelector::showTagSelector() {
    show();
}


void
GNEFrameModules::TagSelector::hideTagSelector() {
    hide();
}


GNEAttributeCarrier*
GNEFrameModules::TagSelector::getTemplateAC(SumoXMLTag ACTag) const {
    // clear myACTemplates and myTagsMatchBox
    for (const auto& ACTemplate : myACTemplates) {
        if (ACTemplate->getAC()->getTagProperty().getTag() == ACTag) {
            return ACTemplate->getAC();
        }
    }
    return nullptr;
}


GNEAttributeCarrier*
GNEFrameModules::TagSelector::getCurrentTemplateAC() const {
    return myCurrentTemplateAC;
}


void
GNEFrameModules::TagSelector::setCurrentTagType(GNETagProperties::TagType tagType, const bool onlyDrawables, const bool notifyFrameParent) {
    // check if net has proj
    const bool proj = (GeoConvHelper::getFinal().getProjString() != "!");
    // set new tagType
    myTagType = tagType;
    // change TagSelector text
    switch (myTagType) {
        case GNETagProperties::TagType::NETWORKELEMENT:
            setText("network elements");
            break;
        case GNETagProperties::TagType::ADDITIONALELEMENT:
            setText("Additional elements");
            break;
        case GNETagProperties::TagType::SHAPE:
            setText("Shape elements");
            break;
        case GNETagProperties::TagType::TAZELEMENT:
            setText("TAZ elements");
            break;
        case GNETagProperties::TagType::VEHICLE:
            setText("Vehicles");
            break;
        case GNETagProperties::TagType::STOP:
            setText("Stops");
            break;
        case GNETagProperties::TagType::PERSON:
            setText("Persons");
            break;
        case GNETagProperties::TagType::PERSONPLAN:
            setText("Person plans");
            break;
        case GNETagProperties::TagType::CONTAINER:
            setText("Container");
            break;
        case GNETagProperties::TagType::CONTAINERPLAN:
            setText("Container plans");
            break;
        case GNETagProperties::TagType::PERSONTRIP:
            setText("Person trips");
            break;
        case GNETagProperties::TagType::WALK:
            setText("Walks");
            break;
        case GNETagProperties::TagType::RIDE:
            setText("Rides");
            break;
        case GNETagProperties::TagType::STOPPERSON:
            setText("Person stops");
            break;
        default:
            throw ProcessError("invalid tag property");
    }
    // clear myACTemplates and myTagsMatchBox
    for (const auto& ACTemplate : myACTemplates) {
        delete ACTemplate;
    }
    myACTemplates.clear();
    myTagsMatchBox->clearItems();
    // get tag properties
    const auto tagProperties = GNEAttributeCarrier::getTagPropertiesByType(myTagType);
    // fill myACTemplates and myTagsMatchBox
    for (const auto& tagProperty : tagProperties) {
        if ((!onlyDrawables || tagProperty.isDrawable()) && (!tagProperty.requireProj() || proj)) {
            myACTemplates.push_back(new ACTemplate(myFrameParent->getViewNet()->getNet(), tagProperty));
            myTagsMatchBox->appendIconItem(tagProperty.getFieldString().c_str(), GUIIconSubSys::getIcon(tagProperty.getGUIIcon()), tagProperty.getBackGroundColor());
        }
    }
    // set color of myTypeMatchBox to black (valid)
    myTagsMatchBox->setTextColor(FXRGB(0, 0, 0));
    // Set visible items
    myTagsMatchBox->setNumVisible((int)myTagsMatchBox->getNumItems());
    // set first myACTemplate as edited AC
    myCurrentTemplateAC = myACTemplates.front()->getAC();
    // call tag selected function
    if (notifyFrameParent) {
        myFrameParent->tagSelected();
    }
}


void
GNEFrameModules::TagSelector::setCurrentTag(SumoXMLTag newTag, const bool notifyFrameParent) {
    // first reset myCurrentTemplateAC
    myCurrentTemplateAC = nullptr;
    // iterate over all myTagsMatchBox
    for (int i = 0; i < (int)myACTemplates.size(); i++) {
        if (myACTemplates.at(i)->getAC() && (myACTemplates.at(i)->getAC()->getTagProperty().getTag() == newTag)) {
            // set current template and currentItem
            myCurrentTemplateAC = myACTemplates.at(i)->getAC();
            myTagsMatchBox->setCurrentItem(i);
            // set color of myTypeMatchBox to black (valid)
            myTagsMatchBox->setTextColor(FXRGB(0, 0, 0));
        }
    }
    // call tag selected function
    if (notifyFrameParent) {
        myFrameParent->tagSelected();
    }
}


void
GNEFrameModules::TagSelector::refreshTagSelector() {
    // call tag selected function
    myFrameParent->tagSelected();
}


long
GNEFrameModules::TagSelector::onCmdSelectTag(FXObject*, FXSelector, void*) {
    // iterate over all myTagsMatchBox
    for (int i = 0; i < (int)myACTemplates.size(); i++) {
        if (myACTemplates.at(i)->getAC() && myACTemplates.at(i)->getAC()->getTagProperty().getFieldString() == myTagsMatchBox->getText().text()) {
            // set templateAC and currentItem
            myCurrentTemplateAC = myACTemplates.at(i)->getAC();
            myTagsMatchBox->setCurrentItem(i);
            // set color of myTypeMatchBox to black (valid)
            myTagsMatchBox->setTextColor(FXRGB(0, 0, 0));
            // call tag selected function
            myFrameParent->tagSelected();
            // Write Warning in console if we're in testing mode
            WRITE_DEBUG(("Selected item '" + myTagsMatchBox->getText() + "' in TagSelector").text());
            return 1;
        }
    }
    // reset templateAC
    myCurrentTemplateAC = nullptr;
    // set color of myTypeMatchBox to red (invalid)
    myTagsMatchBox->setTextColor(FXRGB(255, 0, 0));
    // Write Warning in console if we're in testing mode
    WRITE_DEBUG("Selected invalid item in TagSelector");
    // call tag selected function
    myFrameParent->tagSelected();
    return 1;
}


GNEAttributeCarrier*
GNEFrameModules::TagSelector::ACTemplate::getAC() const {
    return myAC;
}


GNEFrameModules::TagSelector::ACTemplate::ACTemplate(GNENet* net, const GNETagProperties tagProperty) :
    myAC(nullptr) {
    // create attribute carrier depending of
    switch (tagProperty.getTag()) {
        // additional elements
        case SUMO_TAG_BUS_STOP:
        case SUMO_TAG_TRAIN_STOP:
            myAC = new GNEBusStop(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_ACCESS:
            myAC = new GNEAccess(net);
            break;
        case SUMO_TAG_CONTAINER_STOP:
            myAC = new GNEContainerStop(net);
            break;
        case SUMO_TAG_CHARGING_STATION:
            myAC = new GNEChargingStation(net);
            break;
        case SUMO_TAG_PARKING_AREA:
            myAC = new GNEParkingArea(net);
            break;
        case SUMO_TAG_PARKING_SPACE:
            myAC = new GNEParkingSpace(net);
            break;
        case SUMO_TAG_E1DETECTOR:
            myAC = new GNEDetectorE1(net);
            break;
        case SUMO_TAG_E2DETECTOR:
        case GNE_TAG_E2DETECTOR_MULTILANE:
            myAC = new GNEDetectorE2(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_E3DETECTOR:
            myAC = new GNEDetectorE3(net);
            break;
        case SUMO_TAG_DET_ENTRY:
        case SUMO_TAG_DET_EXIT:
            myAC = new GNEDetectorEntryExit(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_INSTANT_INDUCTION_LOOP:
            myAC = new GNEDetectorE1Instant(net);
            break;
        case SUMO_TAG_VSS:
            myAC = new GNEVariableSpeedSign(net);
            break;
        case SUMO_TAG_STEP:
            myAC = new GNEVariableSpeedSignStep(net);
            break;
        case SUMO_TAG_CALIBRATOR:
        case SUMO_TAG_LANECALIBRATOR:
            myAC = new GNECalibrator(tagProperty.getTag(), net);
            break;
        case GNE_TAG_FLOW_CALIBRATOR:
            myAC = new GNECalibratorFlow(net);
            break;
        case SUMO_TAG_REROUTER:
            myAC = new GNERerouter(net);
            break;
        case SUMO_TAG_INTERVAL:
            myAC = new GNERerouterInterval(net);
            break;
        case SUMO_TAG_CLOSING_REROUTE:
            myAC = new GNEClosingReroute(net);
            break;
        case SUMO_TAG_CLOSING_LANE_REROUTE:
            myAC = new GNEClosingLaneReroute(net);
            break;
        case SUMO_TAG_DEST_PROB_REROUTE:
            myAC = new GNEDestProbReroute(net);
            break;
        case SUMO_TAG_PARKING_AREA_REROUTE:
            myAC = new GNEParkingAreaReroute(net);
            break;
        case SUMO_TAG_ROUTE_PROB_REROUTE:
            myAC = new GNERouteProbReroute(net);
            break;
        case SUMO_TAG_ROUTEPROBE:
            myAC = new GNERouteProbe(net);
            break;
        case SUMO_TAG_VAPORIZER:
            myAC = new GNEVaporizer(net);
            break;
        // shapes
        case SUMO_TAG_POLY:
            myAC = new GNEPoly(net);
            break;
        case SUMO_TAG_POI:
        case GNE_TAG_POILANE:
        case GNE_TAG_POIGEO:
            myAC = new GNEPOI(tagProperty.getTag(), net);
            break;
        // TAZs
        case SUMO_TAG_TAZ:
            myAC = new GNETAZ(net);
            break;
        case SUMO_TAG_TAZSOURCE:
        case SUMO_TAG_TAZSINK:
            myAC = new GNETAZSourceSink(tagProperty.getTag(), net);
            break;
        // Demand elements
        case SUMO_TAG_ROUTE:
        case GNE_TAG_ROUTE_EMBEDDED:
            myAC = new GNERoute(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_VTYPE:
            myAC = new GNEVType(net);
            break;
        case SUMO_TAG_VTYPE_DISTRIBUTION:
            myAC = new GNEVTypeDistribution(net);
            break;
        case SUMO_TAG_VEHICLE:
        case GNE_TAG_VEHICLE_WITHROUTE:
        case GNE_TAG_FLOW_ROUTE:
        case GNE_TAG_FLOW_WITHROUTE:
        case SUMO_TAG_TRIP:
        case GNE_TAG_TRIP_JUNCTIONS:
        case SUMO_TAG_FLOW:
        case GNE_TAG_FLOW_JUNCTIONS:
            myAC = new GNEVehicle(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_STOP_LANE:
        case SUMO_TAG_STOP_BUSSTOP:
        case SUMO_TAG_STOP_CONTAINERSTOP:
        case SUMO_TAG_STOP_CHARGINGSTATION:
        case SUMO_TAG_STOP_PARKINGAREA:
        case GNE_TAG_STOPPERSON_EDGE:
        case GNE_TAG_STOPPERSON_BUSSTOP:
        case GNE_TAG_STOPCONTAINER_EDGE:
        case GNE_TAG_STOPCONTAINER_CONTAINERSTOP:
            myAC = new GNEStop(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_PERSON:
        case SUMO_TAG_PERSONFLOW:
            myAC = new GNEPerson(tagProperty.getTag(), net);
            break;
        case SUMO_TAG_CONTAINER:
        case SUMO_TAG_CONTAINERFLOW:
            myAC = new GNEContainer(tagProperty.getTag(), net);
            break;
        case GNE_TAG_TRANSPORT_EDGE:
        case GNE_TAG_TRANSPORT_CONTAINERSTOP:
            myAC = new GNETransport(tagProperty.getTag(), net);
            break;
        case GNE_TAG_TRANSHIP_EDGE:
        case GNE_TAG_TRANSHIP_CONTAINERSTOP:
        case GNE_TAG_TRANSHIP_EDGES:
            myAC = new GNETranship(tagProperty.getTag(), net);
            break;
        case GNE_TAG_PERSONTRIP_EDGE:
        case GNE_TAG_PERSONTRIP_BUSSTOP:
        case GNE_TAG_PERSONTRIP_JUNCTIONS:
            myAC = new GNEPersonTrip(tagProperty.getTag(), net);
            break;
        case GNE_TAG_WALK_EDGE:
        case GNE_TAG_WALK_BUSSTOP:
        case GNE_TAG_WALK_EDGES:
        case GNE_TAG_WALK_ROUTE:
        case GNE_TAG_WALK_JUNCTIONS:
            myAC = new GNEWalk(tagProperty.getTag(), net);
            break;
        case GNE_TAG_RIDE_EDGE:
        case GNE_TAG_RIDE_BUSSTOP:
            myAC = new GNERide(tagProperty.getTag(), net);
            break;
        default:
            throw ProcessError("Non-supported tagProperty in ACTemplate");
            break;
    }
}


GNEFrameModules::TagSelector::ACTemplate::~ACTemplate() {
    delete myAC;
}

// ---------------------------------------------------------------------------
// GNEFrameModules::DemandElementSelector - methods
// ---------------------------------------------------------------------------

GNEFrameModules::DemandElementSelector::DemandElementSelector(GNEFrame* frameParent, SumoXMLTag demandElementTag, GNEDemandElement* defaultElement) :
    FXGroupBoxModule(frameParent->myContentFrame, ("Parent " + toString(demandElementTag)).c_str()),
    myFrameParent(frameParent),
    myCurrentDemandElement(defaultElement),
    myDemandElementTags({demandElementTag}) {
    // Create MFXIconComboBox
    myDemandElementsMatchBox = new MFXIconComboBox(getCollapsableFrame(), GUIDesignComboBoxNCol, this, MID_GNE_SET_TYPE, GUIDesignComboBox);
    // refresh demand element MatchBox
    refreshDemandElementSelector();
    // shown after creation
    show();
}


GNEFrameModules::DemandElementSelector::DemandElementSelector(GNEFrame* frameParent, const std::vector<GNETagProperties::TagType>& tagTypes) :
    FXGroupBoxModule(frameParent->myContentFrame, "Parent element"),
    myFrameParent(frameParent),
    myCurrentDemandElement(nullptr) {
    // fill myDemandElementTags
    for (const auto& tagType : tagTypes) {
        const auto tagProperties = GNEAttributeCarrier::getTagPropertiesByType(tagType);
        for (const auto& tagProperty : tagProperties) {
            myDemandElementTags.push_back(tagProperty.getTag());
        }
    }
    // Create MFXIconComboBox
    myDemandElementsMatchBox = new MFXIconComboBox(getCollapsableFrame(), GUIDesignComboBoxNCol, this, MID_GNE_SET_TYPE, GUIDesignComboBox);
    // refresh demand element MatchBox
    refreshDemandElementSelector();
    // shown after creation
    show();
}


GNEFrameModules::DemandElementSelector::~DemandElementSelector() {}


GNEDemandElement*
GNEFrameModules::DemandElementSelector::getCurrentDemandElement() const {
    return myCurrentDemandElement;
}


const std::vector<SumoXMLTag>&
GNEFrameModules::DemandElementSelector::getAllowedTags() const {
    return myDemandElementTags;
}


void
GNEFrameModules::DemandElementSelector::setDemandElement(GNEDemandElement* demandElement) {
    // first check that demandElement tag correspond to a tag of myDemandElementTags
    if (std::find(myDemandElementTags.begin(), myDemandElementTags.end(), demandElement->getTagProperty().getTag()) != myDemandElementTags.end()) {
        // update text of myDemandElementsMatchBox
        myDemandElementsMatchBox->setItem(demandElement->getID().c_str(), demandElement->getIcon());
        // Set new current demand element
        myCurrentDemandElement = demandElement;
        // call demandElementSelected function
        myFrameParent->demandElementSelected();
    }
}


void
GNEFrameModules::DemandElementSelector::showDemandElementSelector() {
    // first refresh modul
    refreshDemandElementSelector();
    // if current selected item isn't valid, set DEFAULT_VTYPE_ID or DEFAULT_PEDTYPE_ID
    if (myCurrentDemandElement) {
        myDemandElementsMatchBox->setItem(myCurrentDemandElement->getID().c_str(), myCurrentDemandElement->getIcon());
    } else if (myDemandElementTags.size() == 1) {
        if (myDemandElementTags.at(0) == SUMO_TAG_VTYPE) {
            const auto defaultVType = myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE, DEFAULT_VTYPE_ID);
            myDemandElementsMatchBox->setItem(defaultVType->getID().c_str(), defaultVType->getIcon());
        }
    }
    onCmdSelectDemandElement(nullptr, 0, nullptr);
    show();
}


void
GNEFrameModules::DemandElementSelector::hideDemandElementSelector() {
    hide();
}


bool
GNEFrameModules::DemandElementSelector::isDemandElementSelectorShown() const {
    return shown();
}


void
GNEFrameModules::DemandElementSelector::refreshDemandElementSelector() {
    // get demand elemenst container
    const auto& demandElements = myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getDemandElements();
    // clear demand elements comboBox
    myDemandElementsMatchBox->clearItems();
    // fill myTypeMatchBox with list of demand elements
    for (const auto& demandElementTag : myDemandElementTags) {
        // special case for VTypes
        if (demandElementTag == SUMO_TAG_VTYPE) {
            // add default  types in the first positions
            myDemandElementsMatchBox->appendIconItem(DEFAULT_VTYPE_ID.c_str(), GUIIconSubSys::getIcon(GUIIcon::VTYPE));
            myDemandElementsMatchBox->appendIconItem(DEFAULT_BIKETYPE_ID.c_str(), GUIIconSubSys::getIcon(GUIIcon::VTYPE));
            myDemandElementsMatchBox->appendIconItem(DEFAULT_TAXITYPE_ID.c_str(), GUIIconSubSys::getIcon(GUIIcon::VTYPE));
            myDemandElementsMatchBox->appendIconItem(DEFAULT_PEDTYPE_ID.c_str(), GUIIconSubSys::getIcon(GUIIcon::VTYPE));
            myDemandElementsMatchBox->appendIconItem(DEFAULT_CONTAINERTYPE_ID.c_str(), GUIIconSubSys::getIcon(GUIIcon::VTYPE));
            // add rest of vTypes
            for (const auto& vType : demandElements.at(demandElementTag)) {
                // avoid insert duplicated default vType
                if (DEFAULT_VTYPES.count(vType->getID()) == 0) {
                    myDemandElementsMatchBox->appendIconItem(vType->getID().c_str(), vType->getIcon());
                }
            }
        } else {
            // insert all Ids
            for (const auto& demandElement : demandElements.at(demandElementTag)) {
                myDemandElementsMatchBox->appendIconItem(demandElement->getID().c_str(), demandElement->getIcon());
            }
        }
    }
    // Set number of  items (maximum 10)
    if (myDemandElementsMatchBox->getNumItems() < 10) {
        myDemandElementsMatchBox->setNumVisible((int)myDemandElementsMatchBox->getNumItems());
    } else {
        myDemandElementsMatchBox->setNumVisible(10);
    }
    // update myCurrentDemandElement
    if (myDemandElementsMatchBox->getNumItems() == 0) {
        myCurrentDemandElement = nullptr;
    } else if (myCurrentDemandElement) {
        for (int i = 0; i < myDemandElementsMatchBox->getNumItems(); i++) {
            if (myDemandElementsMatchBox->getItem(i).text() == myCurrentDemandElement->getID()) {
                myDemandElementsMatchBox->setCurrentItem(i, FALSE);
            }
        }
    } else {
        // set first element in the list as myCurrentDemandElement (Special case for default person and vehicle type)
        if (myDemandElementsMatchBox->getItem(0).text() == DEFAULT_VTYPE_ID) {
            myCurrentDemandElement = myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getDefaultType();
        } else {
            // disable myCurrentDemandElement
            myCurrentDemandElement = nullptr;
            // update myCurrentDemandElement with the first allowed element
            for (auto i = myDemandElementTags.begin(); (i != myDemandElementTags.end()) && (myCurrentDemandElement == nullptr); i++) {
                if (demandElements.at(*i).size() > 0) {
                    myCurrentDemandElement = *demandElements.at(*i).begin();
                }
            }
        }
    }
}


GNEEdge*
GNEFrameModules::DemandElementSelector::getPersonPlanPreviousEdge() const {
    if (myCurrentDemandElement == nullptr) {
        return nullptr;
    }
    if (!myCurrentDemandElement->getTagProperty().isPerson()) {
        return nullptr;
    }
    if (myCurrentDemandElement->getChildDemandElements().empty()) {
        return nullptr;
    }
    // get last person plan
    const GNEDemandElement* lastPersonPlan = myCurrentDemandElement->getChildDemandElements().back();
    // check tag
    switch (lastPersonPlan->getTagProperty().getTag()) {
        // person trips
        case GNE_TAG_PERSONTRIP_EDGE:
        // rides
        case GNE_TAG_RIDE_EDGE:
        // walks
        case GNE_TAG_WALK_EDGE:
        case GNE_TAG_WALK_EDGES:
        // stops
        case GNE_TAG_STOPPERSON_EDGE:
            return lastPersonPlan->getParentEdges().back();
        // person trips
        case GNE_TAG_PERSONTRIP_BUSSTOP:
        // person trips
        case GNE_TAG_RIDE_BUSSTOP:
        // walks
        case GNE_TAG_WALK_BUSSTOP:
        // stops
        case GNE_TAG_STOPPERSON_BUSSTOP:
            return lastPersonPlan->getParentAdditionals().back()->getParentLanes().front()->getParentEdge();
        // route walks
        case GNE_TAG_WALK_ROUTE:
            return lastPersonPlan->getParentDemandElements().back()->getParentEdges().back();
        default:
            return nullptr;
    }
}


GNEEdge*
GNEFrameModules::DemandElementSelector::getContainerPlanPreviousEdge() const {
    if (myCurrentDemandElement == nullptr) {
        return nullptr;
    }
    if (!myCurrentDemandElement->getTagProperty().isContainer()) {
        return nullptr;
    }
    if (myCurrentDemandElement->getChildDemandElements().empty()) {
        return nullptr;
    }
    // get last container plan
    const GNEDemandElement* lastContainerPlan = myCurrentDemandElement->getChildDemandElements().back();
    // check tag
    switch (lastContainerPlan->getTagProperty().getTag()) {
        // transport
        case GNE_TAG_TRANSPORT_EDGE:
        // tranship
        case GNE_TAG_TRANSHIP_EDGE:
        case GNE_TAG_TRANSHIP_EDGES:
        // stop
        case GNE_TAG_STOPCONTAINER_EDGE:
            return lastContainerPlan->getParentEdges().back();
        // transport
        case GNE_TAG_TRANSPORT_CONTAINERSTOP:
        // tranship
        case GNE_TAG_TRANSHIP_CONTAINERSTOP:
        // stop
        case GNE_TAG_STOPCONTAINER_CONTAINERSTOP:
            return lastContainerPlan->getParentAdditionals().back()->getParentLanes().front()->getParentEdge();
        default:
            return nullptr;
    }
}


long
GNEFrameModules::DemandElementSelector::onCmdSelectDemandElement(FXObject*, FXSelector, void*) {
    // Check if value of myTypeMatchBox correspond to a demand element
    for (const auto& demandElementTag : myDemandElementTags) {
        for (const auto& demandElement : myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getDemandElements().at(demandElementTag)) {
            if (demandElement->getID() == myDemandElementsMatchBox->getText().text()) {
                // set color of myTypeMatchBox to black (valid)
                myDemandElementsMatchBox->setTextColor(FXRGB(0, 0, 0));
                // Set new current demand element
                myCurrentDemandElement = demandElement;
                // call demandElementSelected function
                myFrameParent->demandElementSelected();
                // Write Warning in console if we're in testing mode
                WRITE_DEBUG(("Selected item '" + myDemandElementsMatchBox->getText() + "' in DemandElementSelector").text());
                return 1;
            }
        }
    }
    // if demand element selected is invalid, set demand element as null
    myCurrentDemandElement = nullptr;
    // call demandElementSelected function
    myFrameParent->demandElementSelected();
    // change color of myDemandElementsMatchBox to red (invalid)
    myDemandElementsMatchBox->setTextColor(FXRGB(255, 0, 0));
    // Write Warning in console if we're in testing mode
    WRITE_DEBUG("Selected invalid item in DemandElementSelector");
    return 1;
}

// ---------------------------------------------------------------------------
// GNEFrameModules::HierarchicalElementTree - methods
// ---------------------------------------------------------------------------

GNEFrameModules::HierarchicalElementTree::HierarchicalElementTree(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Hierarchy"),
    myFrameParent(frameParent),
    myHE(nullptr),
    myClickedAC(nullptr),
    myClickedJunction(nullptr),
    myClickedEdge(nullptr),
    myClickedLane(nullptr),
    myClickedCrossing(nullptr),
    myClickedConnection(nullptr),
    myClickedShape(nullptr),
    myClickedTAZElement(nullptr),
    myClickedAdditional(nullptr),
    myClickedDemandElement(nullptr),
    myClickedDataSet(nullptr),
    myClickedDataInterval(nullptr),
    myClickedGenericData(nullptr) {
    // Create three list
    myTreeListDinamic = new FXTreeListDinamic(getCollapsableFrame(), this, MID_GNE_ACHIERARCHY_SHOWCHILDMENU, GUIDesignTreeListDinamic);
    hide();
}


GNEFrameModules::HierarchicalElementTree::~HierarchicalElementTree() {}


void
GNEFrameModules::HierarchicalElementTree::showHierarchicalElementTree(GNEAttributeCarrier* AC) {
    myHE = dynamic_cast<GNEHierarchicalElement*>(AC);
    // show HierarchicalElementTree and refresh HierarchicalElementTree
    if (myHE) {
        // refresh HierarchicalElementTree
        refreshHierarchicalElementTree();
        // show myTreeListDinamic
        myTreeListDinamic->show();
        //show modul
        show();
    }
}


void
GNEFrameModules::HierarchicalElementTree::hideHierarchicalElementTree() {
    // set all pointers null
    myHE = nullptr;
    myClickedAC = nullptr;
    myClickedJunction = nullptr;
    myClickedEdge = nullptr;
    myClickedLane = nullptr;
    myClickedCrossing = nullptr;
    myClickedConnection = nullptr;
    myClickedShape = nullptr;
    myClickedTAZElement = nullptr;
    myClickedAdditional = nullptr;
    myClickedDemandElement = nullptr;
    myClickedDataSet = nullptr;
    myClickedDataInterval = nullptr;
    myClickedGenericData = nullptr;
    // hide myTreeListDinamic
    myTreeListDinamic->hide();
    // hide modul
    hide();
}


void
GNEFrameModules::HierarchicalElementTree::refreshHierarchicalElementTree() {
    // clear items
    myTreeListDinamic->clearItems();
    myTreeItemToACMap.clear();
    myTreeItemsConnections.clear();
    // show children of myHE
    if (myHE) {
        showHierarchicalElementChildren(myHE, showAttributeCarrierParents());
    }
}


void
GNEFrameModules::HierarchicalElementTree::removeCurrentEditedAttributeCarrier(const GNEAttributeCarrier* AC) {
    // simply check if AC is the same of myHE
    if (AC == myHE) {
        myHE = nullptr;
    }
}


long
GNEFrameModules::HierarchicalElementTree::onCmdShowChildMenu(FXObject*, FXSelector, void* eventData) {
    // Obtain event
    FXEvent* e = (FXEvent*)eventData;
    // obtain FXTreeItem in the given position
    FXTreeItem* item = myTreeListDinamic->getItemAt(e->win_x, e->win_y);
    // open Pop-up if FXTreeItem has a Attribute Carrier vinculated
    if (item && (myTreeItemsConnections.find(item) == myTreeItemsConnections.end())) {
        createPopUpMenu(e->root_x, e->root_y, myTreeItemToACMap[item]);
    }
    return 1;
}


long
GNEFrameModules::HierarchicalElementTree::onCmdCenterItem(FXObject*, FXSelector, void*) {
    // Center item
    if (myClickedJunction) {
        myFrameParent->myViewNet->centerTo(myClickedJunction->getGlID(), true, -1);
    } else if (myClickedEdge) {
        myFrameParent->myViewNet->centerTo(myClickedEdge->getGlID(), true, -1);
    } else if (myClickedLane) {
        myFrameParent->myViewNet->centerTo(myClickedLane->getGlID(), true, -1);
    } else if (myClickedCrossing) {
        myFrameParent->myViewNet->centerTo(myClickedCrossing->getGlID(), true, -1);
    } else if (myClickedConnection) {
        myFrameParent->myViewNet->centerTo(myClickedConnection->getGlID(), true, -1);
    } else if (myClickedAdditional) {
        myFrameParent->myViewNet->centerTo(myClickedAdditional->getGlID(), true, -1);
    } else if (myClickedShape) {
        myFrameParent->myViewNet->centerTo(myClickedShape->getGlID(), true, -1);
    } else if (myClickedTAZElement) {
        myFrameParent->myViewNet->centerTo(myClickedTAZElement->getGlID(), true, -1);
    } else if (myClickedDemandElement) {
        myFrameParent->myViewNet->centerTo(myClickedDemandElement->getGlID(), true, -1);
    } else if (myClickedGenericData) {
        myFrameParent->myViewNet->centerTo(myClickedGenericData->getGlID(), true, -1);
    }
    // update view after centering
    myFrameParent->myViewNet->updateViewNet();
    return 1;
}


long
GNEFrameModules::HierarchicalElementTree::onCmdInspectItem(FXObject*, FXSelector, void*) {
    if ((myHE != nullptr) && (myClickedAC != nullptr)) {
        myFrameParent->myViewNet->getViewParent()->getInspectorFrame()->inspectChild(myClickedAC, myHE);
    }
    return 1;
}


long
GNEFrameModules::HierarchicalElementTree::onCmdDeleteItem(FXObject*, FXSelector, void*) {
    // Remove Attribute Carrier
    if (myClickedJunction) {
        myFrameParent->myViewNet->getNet()->deleteJunction(myClickedJunction, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedEdge) {
        myFrameParent->myViewNet->getNet()->deleteEdge(myClickedEdge, myFrameParent->myViewNet->getUndoList(), false);
    } else if (myClickedLane) {
        myFrameParent->myViewNet->getNet()->deleteLane(myClickedLane, myFrameParent->myViewNet->getUndoList(), false);
    } else if (myClickedCrossing) {
        myFrameParent->myViewNet->getNet()->deleteCrossing(myClickedCrossing, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedConnection) {
        myFrameParent->myViewNet->getNet()->deleteConnection(myClickedConnection, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedAdditional) {
        myFrameParent->myViewNet->getNet()->deleteAdditional(myClickedAdditional, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedShape) {
        myFrameParent->myViewNet->getNet()->deleteShape(myClickedShape, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedTAZElement) {
        myFrameParent->myViewNet->getNet()->deleteTAZElement(myClickedTAZElement, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedDemandElement) {
        // check that default VTypes aren't removed
        if ((myClickedDemandElement->getTagProperty().getTag() == SUMO_TAG_VTYPE) && (GNEAttributeCarrier::parse<bool>(myClickedDemandElement->getAttribute(GNE_ATTR_DEFAULT_VTYPE)))) {
            WRITE_WARNING("Default Vehicle Type '" + myClickedDemandElement->getAttribute(SUMO_ATTR_ID) + "' cannot be removed");
            return 1;
        } else if (myClickedDemandElement->getTagProperty().isPersonPlan() && (myClickedDemandElement->getParentDemandElements().front()->getChildDemandElements().size() == 1)) {
            // we need to check if we're removing the last person plan of a person.
            myFrameParent->myViewNet->getNet()->deleteDemandElement(myClickedDemandElement->getParentDemandElements().front(), myFrameParent->myViewNet->getUndoList());
        } else {
            myFrameParent->myViewNet->getNet()->deleteDemandElement(myClickedDemandElement, myFrameParent->myViewNet->getUndoList());
        }
    } else if (myClickedDataSet) {
        myFrameParent->myViewNet->getNet()->deleteDataSet(myClickedDataSet, myFrameParent->myViewNet->getUndoList());
    } else if (myClickedDataInterval) {
        // check if we have to remove data Set
        if (myClickedDataInterval->getDataSetParent()->getDataIntervalChildren().size() == 1) {
            myFrameParent->myViewNet->getNet()->deleteDataSet(myClickedDataInterval->getDataSetParent(), myFrameParent->myViewNet->getUndoList());
        } else {
            myFrameParent->myViewNet->getNet()->deleteDataInterval(myClickedDataInterval, myFrameParent->myViewNet->getUndoList());
        }
    } else if (myClickedGenericData) {
        // check if we have to remove interval
        if (myClickedGenericData->getDataIntervalParent()->getGenericDataChildren().size() == 1) {
            // check if we have to remove data Set
            if (myClickedGenericData->getDataIntervalParent()->getDataSetParent()->getDataIntervalChildren().size() == 1) {
                myFrameParent->myViewNet->getNet()->deleteDataSet(myClickedGenericData->getDataIntervalParent()->getDataSetParent(), myFrameParent->myViewNet->getUndoList());
            } else {
                myFrameParent->myViewNet->getNet()->deleteDataInterval(myClickedGenericData->getDataIntervalParent(), myFrameParent->myViewNet->getUndoList());
            }
        } else {
            myFrameParent->myViewNet->getNet()->deleteGenericData(myClickedGenericData, myFrameParent->myViewNet->getUndoList());
        }
    }
    // update net
    myFrameParent->myViewNet->updateViewNet();
    // refresh AC Hierarchy
    refreshHierarchicalElementTree();
    // check if inspector frame has to be shown again
    if (myFrameParent->myViewNet->getInspectedAttributeCarriers().size() == 1) {
        if (myFrameParent->myViewNet->getInspectedAttributeCarriers().front() != myClickedAC) {
            myFrameParent->myViewNet->getViewParent()->getInspectorFrame()->inspectSingleElement(myFrameParent->myViewNet->getInspectedAttributeCarriers().front());
        } else {
            // inspect a nullprt element to reset inspector frame
            myFrameParent->myViewNet->getViewParent()->getInspectorFrame()->inspectSingleElement(nullptr);
        }
    }
    return 1;
}


long
GNEFrameModules::HierarchicalElementTree::onCmdMoveItemUp(FXObject*, FXSelector, void*) {
    // currently only children of demand elements can be moved
    if (myClickedDemandElement) {
        myFrameParent->myViewNet->getUndoList()->begin(myClickedDemandElement->getTagProperty().getGUIIcon(), ("moving up " + myClickedDemandElement->getTagStr()).c_str());
        // move element one position back
        myFrameParent->myViewNet->getUndoList()->add(new GNEChange_Children(myClickedDemandElement->getParentDemandElements().at(0), myClickedDemandElement,
                GNEChange_Children::Operation::MOVE_BACK), true);
        myFrameParent->myViewNet->getUndoList()->end();
    }
    // refresh after moving child
    refreshHierarchicalElementTree();
    return 1;
}


long
GNEFrameModules::HierarchicalElementTree::onCmdMoveItemDown(FXObject*, FXSelector, void*) {
    // currently only children of demand elements can be moved
    if (myClickedDemandElement) {
        myFrameParent->myViewNet->getUndoList()->begin(myClickedDemandElement->getTagProperty().getGUIIcon(), ("moving down " + myClickedDemandElement->getTagStr()).c_str());
        // move element one position front
        myFrameParent->myViewNet->getUndoList()->add(new GNEChange_Children(myClickedDemandElement->getParentDemandElements().at(0), myClickedDemandElement,
                GNEChange_Children::Operation::MOVE_FRONT), true);
        myFrameParent->myViewNet->getUndoList()->end();
    }
    // refresh after moving child
    refreshHierarchicalElementTree();
    return 1;
}


void
GNEFrameModules::HierarchicalElementTree::createPopUpMenu(int X, int Y, GNEAttributeCarrier* clickedAC) {
    // get attributeCarrirs
    const auto& attributeCarriers = myFrameParent->myViewNet->getNet()->getAttributeCarriers();
    // first check that AC exist
    if (clickedAC) {
        // set current clicked AC
        myClickedAC = clickedAC;
        // cast all elements
        myClickedJunction = attributeCarriers->retrieveJunction(clickedAC->getID(), false);
        myClickedEdge = attributeCarriers->retrieveEdge(clickedAC->getID(), false);
        myClickedLane = attributeCarriers->retrieveLane(clickedAC, false);
        myClickedCrossing = attributeCarriers->retrieveCrossing(clickedAC, false);
        myClickedConnection = attributeCarriers->retrieveConnection(clickedAC, false);
        myClickedShape = attributeCarriers->retrieveShape(clickedAC, false);
        myClickedTAZElement = attributeCarriers->retrieveTAZElement(clickedAC, false);
        myClickedAdditional = attributeCarriers->retrieveAdditional(clickedAC, false);
        myClickedDemandElement = attributeCarriers->retrieveDemandElement(clickedAC, false);
        myClickedDataSet = attributeCarriers->retrieveDataSet(clickedAC, false);
        myClickedDataInterval = attributeCarriers->retrieveDataInterval(clickedAC, false);
        myClickedGenericData = attributeCarriers->retrieveGenericData(clickedAC, false);
        // create FXMenuPane
        FXMenuPane* pane = new FXMenuPane(myTreeListDinamic);
        // set item name and icon
        new MFXMenuHeader(pane, myFrameParent->myViewNet->getViewParent()->getGUIMainWindow()->getBoldFont(), myClickedAC->getPopUpID().c_str(), myClickedAC->getIcon());
        // insert separator
        new FXMenuSeparator(pane);
        // create center menu command
        FXMenuCommand* centerMenuCommand = GUIDesigns::buildFXMenuCommand(pane, "Center", GUIIconSubSys::getIcon(GUIIcon::RECENTERVIEW), this, MID_GNE_CENTER);
        // disable Centering for Vehicle Types, data sets and data intervals
        if (myClickedAC->getTagProperty().isVehicleType() || (myClickedAC->getTagProperty().getTag() == SUMO_TAG_DATASET) ||
                (myClickedAC->getTagProperty().getTag() == SUMO_TAG_DATAINTERVAL)) {
            centerMenuCommand->disable();
        }
        // create inspect and delete menu commands
        FXMenuCommand* inspectMenuCommand = GUIDesigns::buildFXMenuCommand(pane, "Inspect", GUIIconSubSys::getIcon(GUIIcon::MODEINSPECT), this, MID_GNE_INSPECT);
        FXMenuCommand* deleteMenuCommand = GUIDesigns::buildFXMenuCommand(pane, "Delete", GUIIconSubSys::getIcon(GUIIcon::MODEDELETE), this, MID_GNE_DELETE);
        // check if inspect and delete menu commands has to be disabled
        if (GNEFrameAttributeModules::isSupermodeValid(myFrameParent->myViewNet, myClickedAC) == false) {
            inspectMenuCommand->disable();
            deleteMenuCommand->disable();
        }
        // now chec if given AC support manually moving of their item up and down (Currently only for certain demand elements)
        /* if (myClickedDemandElement && myClickedAC->getTagProperty().canBeSortedManually()) {
            // insert separator
            new FXMenuSeparator(pane);
            // create both moving menu commands
            FXMenuCommand* moveUpMenuCommand = GUIDesigns::buildFXMenuCommand(pane, "Move up", GUIIconSubSys::getIcon(GUIIcon::ARROW_UP), this, MID_GNE_ACHIERARCHY_MOVEUP);
            FXMenuCommand* moveDownMenuCommand = GUIDesigns::buildFXMenuCommand(pane, "Move down", GUIIconSubSys::getIcon(GUIIcon::ARROW_DOWN), this, MID_GNE_ACHIERARCHY_MOVEDOWN);
            // check if both commands has to be disabled
            if (myClickedDemandElement->getTagProperty().isStopPerson()) {
                moveUpMenuCommand->setText("Move up (Stops cannot be moved)");
                moveDownMenuCommand->setText("Move down (Stops cannot be moved)");
                moveUpMenuCommand->disable();
                moveDownMenuCommand->disable();
            } else {
                // check if moveUpMenuCommand has to be disabled
                if (myClickedDemandElement->getParentDemandElements().front()->getChildDemandElements().front() == myClickedDemandElement) {
                    moveUpMenuCommand->setText("Move up (It's already the first element)");
                    moveUpMenuCommand->disable();
                } else if (myClickedDemandElement->getParentDemandElements().front()->getPreviousChildDemandElement(myClickedDemandElement)->getTagProperty().isStopPerson()) {
                    moveUpMenuCommand->setText("Move up (Previous element is a Stop)");
                    moveUpMenuCommand->disable();
                }
                // check if moveDownMenuCommand has to be disabled
                if (myClickedDemandElement->getParentDemandElements().front()->getChildDemandElements().back() == myClickedDemandElement) {
                    moveDownMenuCommand->setText("Move down (It's already the last element)");
                    moveDownMenuCommand->disable();
                } else if (myClickedDemandElement->getParentDemandElements().front()->getNextChildDemandElement(myClickedDemandElement)->getTagProperty().isStopPerson()) {
                    moveDownMenuCommand->setText("Move down (Next element is a Stop)");
                    moveDownMenuCommand->disable();
                }
            }
        } */
        // Center in the mouse position and create pane
        pane->setX(X);
        pane->setY(Y);
        pane->create();
        pane->show();
    } else {
        // set all clicked elements to null
        myClickedAC = nullptr;
        myClickedJunction = nullptr;
        myClickedEdge = nullptr;
        myClickedLane = nullptr;
        myClickedCrossing = nullptr;
        myClickedConnection = nullptr;
        myClickedShape = nullptr;
        myClickedTAZElement = nullptr;
        myClickedAdditional = nullptr;
        myClickedDemandElement = nullptr;
        myClickedDataSet = nullptr;
        myClickedDataInterval = nullptr;
        myClickedGenericData = nullptr;
    }
}


FXTreeItem*
GNEFrameModules::HierarchicalElementTree::showAttributeCarrierParents() {
    // get attributeCarrirs
    const auto& attributeCarriers = myFrameParent->myViewNet->getNet()->getAttributeCarriers();
    // check tags
    if (myHE->getTagProperty().isNetworkElement()) {
        // check demand element type
        switch (myHE->getTagProperty().getTag()) {
            case SUMO_TAG_EDGE: {
                // obtain Edge
                GNEEdge* edge = attributeCarriers->retrieveEdge(myHE->getID(), false);
                if (edge) {
                    // insert Junctions of edge in tree (Pararell because a edge has always two Junctions)
                    FXTreeItem* junctionSourceItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " origin").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
                    FXTreeItem* junctionDestinyItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " destiny").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
                    junctionDestinyItem->setExpanded(true);
                    // Save items in myTreeItemToACMap
                    myTreeItemToACMap[junctionSourceItem] = edge->getFromJunction();
                    myTreeItemToACMap[junctionDestinyItem] = edge->getToJunction();
                    // return junction destiny Item
                    return junctionDestinyItem;
                } else {
                    return nullptr;
                }
            }
            case SUMO_TAG_LANE: {
                // obtain lane
                GNELane* lane = attributeCarriers->retrieveLane(myHE->getID(), false);
                if (lane) {
                    // obtain parent edge
                    GNEEdge* edge = attributeCarriers->retrieveEdge(lane->getParentEdge()->getID());
                    //inser Junctions of lane of edge in tree (Pararell because a edge has always two Junctions)
                    FXTreeItem* junctionSourceItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " origin").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
                    FXTreeItem* junctionDestinyItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " destiny").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
                    junctionDestinyItem->setExpanded(true);
                    // Create edge item
                    FXTreeItem* edgeItem = myTreeListDinamic->insertItem(nullptr, junctionDestinyItem, edge->getHierarchyName().c_str(), edge->getIcon(), edge->getIcon());
                    edgeItem->setExpanded(true);
                    // Save items in myTreeItemToACMap
                    myTreeItemToACMap[junctionSourceItem] = edge->getFromJunction();
                    myTreeItemToACMap[junctionDestinyItem] = edge->getToJunction();
                    myTreeItemToACMap[edgeItem] = edge;
                    // return edge item
                    return edgeItem;
                } else {
                    return nullptr;
                }
            }
            case SUMO_TAG_CROSSING: {
                // obtain crossing parent junction
                GNEJunction* junction = attributeCarriers->retrieveCrossing(myHE)->getParentJunction();
                // create junction item
                FXTreeItem* junctionItem = myTreeListDinamic->insertItem(nullptr, nullptr, junction->getHierarchyName().c_str(), junction->getIcon(), junction->getIcon());
                junctionItem->setExpanded(true);
                // Save items in myTreeItemToACMap
                myTreeItemToACMap[junctionItem] = junction;
                // return junction Item
                return junctionItem;
            }
            case SUMO_TAG_CONNECTION: {
                // obtain Connection
                GNEConnection* connection = attributeCarriers->retrieveConnection(myHE->getID(), false);
                if (connection) {
                    // create edge from item
                    FXTreeItem* edgeFromItem = myTreeListDinamic->insertItem(nullptr, nullptr, connection->getEdgeFrom()->getHierarchyName().c_str(), connection->getEdgeFrom()->getIcon(), connection->getEdgeFrom()->getIcon());
                    edgeFromItem->setExpanded(true);
                    // create edge to item
                    FXTreeItem* edgeToItem = myTreeListDinamic->insertItem(nullptr, nullptr, connection->getEdgeTo()->getHierarchyName().c_str(), connection->getEdgeTo()->getIcon(), connection->getEdgeTo()->getIcon());
                    edgeToItem->setExpanded(true);
                    // create connection item
                    FXTreeItem* connectionItem = myTreeListDinamic->insertItem(nullptr, edgeToItem, connection->getHierarchyName().c_str(), connection->getIcon(), connection->getIcon());
                    connectionItem->setExpanded(true);
                    // Save items in myTreeItemToACMap
                    myTreeItemToACMap[edgeFromItem] = connection->getEdgeFrom();
                    myTreeItemToACMap[edgeToItem] = connection->getEdgeTo();
                    myTreeItemToACMap[connectionItem] = connection;
                    // return connection item
                    return connectionItem;
                } else {
                    return nullptr;
                }
            }
            default:
                break;
        }
    } else if (myHE->getTagProperty().getTag() == GNE_TAG_POILANE) {
        // Obtain POILane
        const GNEShape* POILane = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveShape(myHE);
        // obtain parent lane
        GNELane* lane = attributeCarriers->retrieveLane(POILane->getParentLanes().at(0)->getID());
        // obtain parent edge
        GNEEdge* edge = attributeCarriers->retrieveEdge(lane->getParentEdge()->getID());
        //inser Junctions of lane of edge in tree (Pararell because a edge has always two Junctions)
        FXTreeItem* junctionSourceItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " origin").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
        FXTreeItem* junctionDestinyItem = myTreeListDinamic->insertItem(nullptr, nullptr, (edge->getFromJunction()->getHierarchyName() + " destiny").c_str(), edge->getFromJunction()->getIcon(), edge->getFromJunction()->getIcon());
        junctionDestinyItem->setExpanded(true);
        // Create edge item
        FXTreeItem* edgeItem = myTreeListDinamic->insertItem(nullptr, junctionDestinyItem, edge->getHierarchyName().c_str(), edge->getIcon(), edge->getIcon());
        edgeItem->setExpanded(true);
        // Create lane item
        FXTreeItem* laneItem = myTreeListDinamic->insertItem(nullptr, edgeItem, lane->getHierarchyName().c_str(), lane->getIcon(), lane->getIcon());
        laneItem->setExpanded(true);
        // Save items in myTreeItemToACMap
        myTreeItemToACMap[junctionSourceItem] = edge->getFromJunction();
        myTreeItemToACMap[junctionDestinyItem] = edge->getToJunction();
        myTreeItemToACMap[edgeItem] = edge;
        myTreeItemToACMap[laneItem] = lane;
        // return Lane item
        return laneItem;
    } else if (myHE->getTagProperty().isAdditionalElement()) {
        // Obtain Additional
        const GNEAdditional* additional = attributeCarriers->retrieveAdditional(myHE);
        // declare auxiliar FXTreeItem, due a demand element can have multiple "roots"
        FXTreeItem* root = nullptr;
        // check if there is demand elements parents
        if (additional->getParentAdditionals().size() > 0) {
            // check if we have more than one edge
            if (additional->getParentAdditionals().size() > 1) {
                // insert first item
                addListItem(additional->getParentAdditionals().front());
                // insert "spacer"
                if (additional->getParentAdditionals().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)additional->getParentAdditionals().size() - 2) + " additionals...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(additional->getParentAdditionals().back());
        }
        // check if there is parent demand elements
        if (additional->getParentDemandElements().size() > 0) {
            // check if we have more than one demand element
            if (additional->getParentDemandElements().size() > 1) {
                // insert first item
                addListItem(additional->getParentDemandElements().front());
                // insert "spacer"
                if (additional->getParentDemandElements().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)additional->getParentDemandElements().size() - 2) + " demand elements...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(additional->getParentDemandElements().back());
        }
        // check if there is parent edges
        if (additional->getParentEdges().size() > 0) {
            // check if we have more than one edge
            if (additional->getParentEdges().size() > 1) {
                // insert first item
                addListItem(additional->getParentEdges().front());
                // insert "spacer"
                if (additional->getParentEdges().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)additional->getParentEdges().size() - 2) + " edges...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(additional->getParentEdges().back());
        }
        // check if there is parent lanes
        if (additional->getParentLanes().size() > 0) {
            // check if we have more than one parent lane
            if (additional->getParentLanes().size() > 1) {
                // insert first item
                addListItem(additional->getParentLanes().front());
                // insert "spacer"
                if (additional->getParentLanes().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)additional->getParentLanes().size() - 2) + " lanes...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(additional->getParentLanes().back());
        }
        // return last inserted list item
        return root;
    } else if (myHE->getTagProperty().isTAZElement()) {
        // Obtain TAZElement
        const GNETAZElement* TAZElement = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveTAZElement(myHE);
        // declare auxiliar FXTreeItem, due a demand element can have multiple "roots"
        FXTreeItem* root = nullptr;
        // check if there is demand elements parents
        if (TAZElement->getParentTAZElements().size() > 0) {
            // check if we have more than one edge
            if (TAZElement->getParentTAZElements().size() > 1) {
                // insert first item
                addListItem(TAZElement->getParentTAZElements().front());
                // insert "spacer"
                if (TAZElement->getParentTAZElements().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)TAZElement->getParentTAZElements().size() - 2) + " TAZElements...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(TAZElement->getParentTAZElements().back());
        }
        // check if there is parent demand elements
        if (TAZElement->getParentDemandElements().size() > 0) {
            // check if we have more than one demand element
            if (TAZElement->getParentDemandElements().size() > 1) {
                // insert first item
                addListItem(TAZElement->getParentDemandElements().front());
                // insert "spacer"
                if (TAZElement->getParentDemandElements().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)TAZElement->getParentDemandElements().size() - 2) + " demand elements...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(TAZElement->getParentDemandElements().back());
        }
        // check if there is parent edges
        if (TAZElement->getParentEdges().size() > 0) {
            // check if we have more than one edge
            if (TAZElement->getParentEdges().size() > 1) {
                // insert first item
                addListItem(TAZElement->getParentEdges().front());
                // insert "spacer"
                if (TAZElement->getParentEdges().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)TAZElement->getParentEdges().size() - 2) + " edges...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(TAZElement->getParentEdges().back());
        }
        // check if there is parent lanes
        if (TAZElement->getParentLanes().size() > 0) {
            // check if we have more than one parent lane
            if (TAZElement->getParentLanes().size() > 1) {
                // insert first item
                addListItem(TAZElement->getParentLanes().front());
                // insert "spacer"
                if (TAZElement->getParentLanes().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)TAZElement->getParentLanes().size() - 2) + " lanes...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(TAZElement->getParentLanes().back());
        }
        // return last inserted list item
        return root;
    } else if (myHE->getTagProperty().isDemandElement()) {
        // Obtain DemandElement
        GNEDemandElement* demandElement = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveDemandElement(myHE);
        // declare auxiliar FXTreeItem, due a demand element can have multiple "roots"
        FXTreeItem* root = nullptr;
        // check if there are demand element parents
        if (demandElement->getParentAdditionals().size() > 0) {
            // check if we have more than one edge
            if (demandElement->getParentAdditionals().size() > 1) {
                // insert first item
                addListItem(demandElement->getParentAdditionals().front());
                // insert "spacer"
                if (demandElement->getParentAdditionals().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)demandElement->getParentAdditionals().size() - 2) + " additionals...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(demandElement->getParentAdditionals().back());
        }
        // check if there is parent demand elements
        if (demandElement->getParentDemandElements().size() > 0) {
            // check if we have more than one demand element
            if (demandElement->getParentDemandElements().size() > 1) {
                // insert first item
                addListItem(demandElement->getParentDemandElements().front());
                // insert "spacer"
                if (demandElement->getParentDemandElements().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)demandElement->getParentDemandElements().size() - 2) + " demand elements...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(demandElement->getParentDemandElements().back());
        }
        // check if there is parent edges
        if (demandElement->getParentEdges().size() > 0) {
            // check if we have more than one edge
            if (demandElement->getParentEdges().size() > 1) {
                // insert first item
                addListItem(demandElement->getParentEdges().front());
                // insert "spacer"
                if (demandElement->getParentEdges().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)demandElement->getParentEdges().size() - 2) + " edges...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(demandElement->getParentEdges().back());
        }
        // check if there is parent lanes
        if (demandElement->getParentLanes().size() > 0) {
            // check if we have more than one parent lane
            if (demandElement->getParentLanes().size() > 1) {
                // insert first item
                addListItem(demandElement->getParentLanes().front());
                // insert "spacer"
                if (demandElement->getParentLanes().size() > 2) {
                    addListItem(nullptr, ("..." + toString((int)demandElement->getParentLanes().size() - 2) + " lanes...").c_str(), 0, false);
                }
            }
            // return last inserted item
            root = addListItem(demandElement->getParentLanes().back());
        }
        // return last inserted list item
        return root;
    } else if (myHE->getTagProperty().isDataElement()) {
        // check if is a GNEDataInterval or a GNEGenericData
        if (myHE->getTagProperty().getTag() == SUMO_TAG_DATASET) {
            return nullptr;
        } else if (myHE->getTagProperty().getTag() == SUMO_TAG_DATAINTERVAL) {
            return addListItem(myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveDataSet(myHE->getID()));
        } else {
            // Obtain DataElement
            GNEGenericData* dataElement = dynamic_cast<GNEGenericData*>(myHE);
            if (dataElement) {
                // declare auxiliar FXTreeItem, due a data element can have multiple "roots"
                FXTreeItem* root = nullptr;
                // set dataset
                addListItem(dataElement->getDataIntervalParent()->getDataSetParent());
                // set data interval
                addListItem(dataElement->getDataIntervalParent());
                // check if there is data elements parents
                if (dataElement->getParentAdditionals().size() > 0) {
                    // check if we have more than one edge
                    if (dataElement->getParentAdditionals().size() > 1) {
                        // insert first item
                        addListItem(dataElement->getParentAdditionals().front());
                        // insert "spacer"
                        if (dataElement->getParentAdditionals().size() > 2) {
                            addListItem(nullptr, ("..." + toString((int)dataElement->getParentAdditionals().size() - 2) + " additionals...").c_str(), 0, false);
                        }
                    }
                    // return last inserted item
                    root = addListItem(dataElement->getParentAdditionals().back());
                }
                // check if there is parent demand elements
                if (dataElement->getParentDemandElements().size() > 0) {
                    // check if we have more than one demand element
                    if (dataElement->getParentDemandElements().size() > 1) {
                        // insert first item
                        addListItem(dataElement->getParentDemandElements().front());
                        // insert "spacer"
                        if (dataElement->getParentDemandElements().size() > 2) {
                            addListItem(nullptr, ("..." + toString((int)dataElement->getParentDemandElements().size() - 2) + " demand elements...").c_str(), 0, false);
                        }
                    }
                    // return last inserted item
                    root = addListItem(dataElement->getParentDemandElements().back());
                }
                // check if there is parent edges
                if (dataElement->getParentEdges().size() > 0) {
                    // check if we have more than one edge
                    if (dataElement->getParentEdges().size() > 1) {
                        // insert first ege
                        if (dataElement->getTagProperty().getTag() == SUMO_TAG_EDGEREL) {
                            addListItem(dataElement->getParentEdges().front(), nullptr, "from ");
                        } else {
                            addListItem(dataElement->getParentEdges().front());
                        }
                        // insert "spacer"
                        if (dataElement->getParentEdges().size() > 2) {
                            addListItem(nullptr, ("..." + toString((int)dataElement->getParentEdges().size() - 2) + " edges...").c_str(), 0, false);
                        }
                    }
                    // insert last ege
                    if (dataElement->getTagProperty().getTag() == SUMO_TAG_EDGEREL) {
                        addListItem(dataElement->getParentEdges().back(), nullptr, "to ");
                    } else {
                        addListItem(dataElement->getParentEdges().back());
                    }
                }
                // check if there is parent lanes
                if (dataElement->getParentLanes().size() > 0) {
                    // check if we have more than one parent lane
                    if (dataElement->getParentLanes().size() > 1) {
                        // insert first item
                        addListItem(dataElement->getParentLanes().front());
                        // insert "spacer"
                        if (dataElement->getParentLanes().size() > 2) {
                            addListItem(nullptr, ("..." + toString((int)dataElement->getParentLanes().size() - 2) + " lanes...").c_str(), 0, false);
                        }
                    }
                    // return last inserted item
                    root = addListItem(dataElement->getParentLanes().back());
                }
                // return last inserted list item
                return root;
            }
        }
    }
    // there aren't parents
    return nullptr;
}


void
GNEFrameModules::HierarchicalElementTree::showHierarchicalElementChildren(GNEHierarchicalElement* HE, FXTreeItem* itemParent) {
    if (HE->getTagProperty().isNetworkElement()) {
        // Switch gl type of ac
        switch (HE->getTagProperty().getTag()) {
            case SUMO_TAG_JUNCTION: {
                // retrieve junction
                GNEJunction* junction = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(HE->getID(), false);
                if (junction) {
                    // insert junction item
                    FXTreeItem* junctionItem = addListItem(HE, itemParent);
                    // insert edges
                    for (const auto& edge : junction->getChildEdges()) {
                        showHierarchicalElementChildren(edge, junctionItem);
                    }
                    // insert crossings
                    for (const auto& crossing : junction->getGNECrossings()) {
                        showHierarchicalElementChildren(crossing, junctionItem);
                    }
                }
                break;
            }
            case SUMO_TAG_EDGE: {
                // retrieve edge
                GNEEdge* edge = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(HE->getID(), false);
                if (edge) {
                    // insert edge item
                    FXTreeItem* edgeItem = addListItem(HE, itemParent);
                    // insert lanes
                    for (const auto& lane : edge->getLanes()) {
                        showHierarchicalElementChildren(lane, edgeItem);
                    }
                    // insert child additional
                    for (const auto& additional : edge->getChildAdditionals()) {
                        showHierarchicalElementChildren(additional, edgeItem);
                    }
                    // insert child shapes
                    for (const auto& shape : edge->getChildShapes()) {
                        showHierarchicalElementChildren(shape, edgeItem);
                    }
                    // insert child TAZElements
                    for (const auto& TAZElement : edge->getChildTAZElements()) {
                        // use addListItem because TAZElement doesn't have children
                        addListItem(TAZElement, edgeItem);
                    }
                    // insert child demand elements
                    for (const auto& demandElement : edge->getChildDemandElements()) {
                        showHierarchicalElementChildren(demandElement, edgeItem);
                    }
                    /*
                    CHECK THIS

                    // insert demand elements children (note: use getChildDemandElementsSortedByType to avoid duplicated elements)
                    for (const auto& route : edge->getChildDemandElementsByType(SUMO_TAG_ROUTE)) {
                        showHierarchicalElementChildren(route, edgeItem);
                    }
                    for (const auto& trip : edge->getChildDemandElementsByType(SUMO_TAG_TRIP)) {
                        showHierarchicalElementChildren(trip, edgeItem);
                    }
                    for (const auto& flow : edge->getChildDemandElementsByType(SUMO_TAG_FLOW)) {
                        showHierarchicalElementChildren(flow, edgeItem);
                    }
                    */
                    // show data elements
                    for (const auto& genericDatas : edge->getChildGenericDatas()) {
                        showHierarchicalElementChildren(genericDatas, edgeItem);
                    }
                }
                break;
            }
            case SUMO_TAG_LANE: {
                // retrieve lane
                GNELane* lane = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveLane(HE->getID(), false);
                if (lane) {
                    // insert lane item
                    FXTreeItem* laneItem = addListItem(HE, itemParent);
                    // insert child additional
                    for (const auto& additional : lane->getChildAdditionals()) {
                        showHierarchicalElementChildren(additional, laneItem);
                    }
                    // insert child shapes
                    for (const auto& shape : lane->getChildShapes()) {
                        showHierarchicalElementChildren(shape, laneItem);
                    }
                    // insert child TAZElements
                    for (const auto& TAZElement : lane->getChildTAZElements()) {
                        // use addListItem because TAZElement doesn't have children
                        addListItem(TAZElement, laneItem);
                    }
                    // insert demand elements children
                    for (const auto& demandElement : lane->getChildDemandElements()) {
                        showHierarchicalElementChildren(demandElement, laneItem);
                    }
                    // insert incoming connections of lanes (by default isn't expanded)
                    if (lane->getGNEIncomingConnections().size() > 0) {
                        std::vector<GNEConnection*> incomingLaneConnections = lane->getGNEIncomingConnections();
                        // insert intermediate list item
                        FXTreeItem* incomingConnections = addListItem(laneItem, "Incomings", incomingLaneConnections.front()->getIcon(), false);
                        // insert incoming connections
                        for (const auto& connection : incomingLaneConnections) {
                            showHierarchicalElementChildren(connection, incomingConnections);
                        }
                    }
                    // insert outcoming connections of lanes (by default isn't expanded)
                    if (lane->getGNEOutcomingConnections().size() > 0) {
                        std::vector<GNEConnection*> outcomingLaneConnections = lane->getGNEOutcomingConnections();
                        // insert intermediate list item
                        FXTreeItem* outgoingConnections = addListItem(laneItem, "Outgoing", outcomingLaneConnections.front()->getIcon(), false);
                        // insert outcoming connections
                        for (const auto& connection : outcomingLaneConnections) {
                            showHierarchicalElementChildren(connection, outgoingConnections);
                        }
                    }
                }
                break;
            }
            case SUMO_TAG_CROSSING:
            case SUMO_TAG_CONNECTION: {
                // insert connection item
                addListItem(HE, itemParent);
                break;
            }
            default:
                break;
        }
    } else if (HE->getTagProperty().isAdditionalElement() || HE->getTagProperty().isShape() || HE->getTagProperty().isTAZElement() || HE->getTagProperty().isDemandElement()) {
        // insert additional item
        FXTreeItem* treeItem = addListItem(HE, itemParent);
        // insert child edges
        for (const auto& edge : HE->getChildEdges()) {
            showHierarchicalElementChildren(edge, treeItem);
        }
        // insert child lanes
        for (const auto& lane : HE->getChildLanes()) {
            showHierarchicalElementChildren(lane, treeItem);
        }
        // insert additional symbols
        std::vector<GNEAdditional*> symbols;
        for (const auto& additional : HE->getChildAdditionals()) {
            if (additional->getTagProperty().isSymbol()) {
                symbols.push_back(additional);
            }
        }
        if (symbols.size() > 0) {
            // insert intermediate list item
            const auto additionalParent = symbols.front()->getParentAdditionals().front();
            const std::string symbolType = additionalParent->getTagProperty().hasAttribute(SUMO_ATTR_EDGES) ? "Edges" : "Lanes";
            GUIIcon symbolIcon = additionalParent->getTagProperty().hasAttribute(SUMO_ATTR_EDGES) ? GUIIcon::EDGE : GUIIcon::LANE;
            FXTreeItem* symbolListItem = addListItem(treeItem, symbolType, GUIIconSubSys::getIcon(symbolIcon), false);
            // insert symbols
            for (const auto& symbol : symbols) {
                showHierarchicalElementChildren(symbol, symbolListItem);
            }
        }
        // insert additional children
        for (const auto& additional : HE->getChildAdditionals()) {
            if (!additional->getTagProperty().isSymbol()) {
                showHierarchicalElementChildren(additional, treeItem);
            }
        }
        // insert child shapes
        for (const auto& shape : HE->getChildShapes()) {
            showHierarchicalElementChildren(shape, treeItem);
        }
        // insert TAZElements children
        for (const auto& TAZElement : HE->getChildTAZElements()) {
            // use addListItem because TAZElement doesn't have children
            addListItem(TAZElement, treeItem);
        }
        // insert child demand elements
        for (const auto& demandElement : HE->getChildDemandElements()) {
            showHierarchicalElementChildren(demandElement, treeItem);
        }
    } else if (HE->getTagProperty().isDataElement()) {
        // insert data item
        FXTreeItem* dataElementItem = addListItem(HE, itemParent);
        // insert intervals
        if (HE->getTagProperty().getTag() == SUMO_TAG_DATASET) {
            GNEDataSet* dataSet = myFrameParent->myViewNet->getNet()->getAttributeCarriers()->retrieveDataSet(HE->getID());
            // iterate over intevals
            for (const auto& interval : dataSet->getDataIntervalChildren()) {
                showHierarchicalElementChildren(interval.second, dataElementItem);
            }
        } else if (HE->getTagProperty().getTag() == SUMO_TAG_DATAINTERVAL) {
            GNEDataInterval* dataInterval = dynamic_cast<GNEDataInterval*>(HE);
            // iterate over generic datas
            for (const auto& genericData : dataInterval->getGenericDataChildren()) {
                showHierarchicalElementChildren(genericData, dataElementItem);
            }
        }
    }
}


FXTreeItem*
GNEFrameModules::HierarchicalElementTree::addListItem(GNEAttributeCarrier* AC, FXTreeItem* itemParent, std::string prefix, std::string sufix) {
    // insert item in Tree list
    FXTreeItem* item = myTreeListDinamic->insertItem(nullptr, itemParent, (prefix + AC->getHierarchyName() + sufix).c_str(), AC->getIcon(), AC->getIcon());
    // insert item in map
    myTreeItemToACMap[item] = AC;
    // by default item is expanded
    item->setExpanded(true);
    // return created FXTreeItem
    return item;
}


FXTreeItem*
GNEFrameModules::HierarchicalElementTree::addListItem(FXTreeItem* itemParent, const std::string& text, FXIcon* icon, bool expanded) {
    // insert item in Tree list
    FXTreeItem* item = myTreeListDinamic->insertItem(nullptr, itemParent, text.c_str(), icon, icon);
    // expand item depending of flag expanded
    item->setExpanded(expanded);
    // return created FXTreeItem
    return item;
}

// ---------------------------------------------------------------------------
// GNEFrameModules::DrawingShape - methods
// ---------------------------------------------------------------------------

GNEFrameModules::DrawingShape::DrawingShape(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Drawing"),
    myFrameParent(frameParent),
    myDeleteLastCreatedPoint(false) {
    // create start and stop buttons
    myStartDrawingButton = new FXButton(getCollapsableFrame(), "Start drawing", 0, this, MID_GNE_STARTDRAWING, GUIDesignButton);
    myStopDrawingButton = new FXButton(getCollapsableFrame(), "Stop drawing", 0, this, MID_GNE_STOPDRAWING, GUIDesignButton);
    myAbortDrawingButton = new FXButton(getCollapsableFrame(), "Abort drawing", 0, this, MID_GNE_ABORTDRAWING, GUIDesignButton);
    // create information label
    std::ostringstream information;
    information
            << "- 'Start drawing' or ENTER\n"
            << "  to create shape.\n"
            << "- 'Stop drawing' or ENTER to\n"
            << "  finish shape creation.\n"
            << "- 'Abort drawing' or ESC to\n"
            << "  abort shape creation.\n"
            << "- 'Shift + Click' to remove\n"
            << "  last inserted point.";
    myInformationLabel = new FXLabel(getCollapsableFrame(), information.str().c_str(), 0, GUIDesignLabelFrameInformation);
    // disable stop and abort functions as init
    myStopDrawingButton->disable();
    myAbortDrawingButton->disable();
}


GNEFrameModules::DrawingShape::~DrawingShape() {}


void GNEFrameModules::DrawingShape::showDrawingShape() {
    // abort current drawing before show
    abortDrawing();
    // show FXGroupBoxModule
    FXGroupBoxModule::show();
}


void GNEFrameModules::DrawingShape::hideDrawingShape() {
    // abort current drawing before hide
    abortDrawing();
    // show FXGroupBoxModule
    FXGroupBoxModule::hide();
}


void
GNEFrameModules::DrawingShape::startDrawing() {
    // Only start drawing if DrawingShape modul is shown
    if (shown()) {
        // change buttons
        myStartDrawingButton->disable();
        myStopDrawingButton->enable();
        myAbortDrawingButton->enable();
    }
}


void
GNEFrameModules::DrawingShape::stopDrawing() {
    // try to build shape
    if (myFrameParent->shapeDrawed()) {
        // clear created points
        myTemporalShape.clear();
        // change buttons
        myStartDrawingButton->enable();
        myStopDrawingButton->disable();
        myAbortDrawingButton->disable();
    } else {
        // abort drawing if shape cannot be created
        abortDrawing();
    }
}


void
GNEFrameModules::DrawingShape::abortDrawing() {
    // clear created points
    myTemporalShape.clear();
    // change buttons
    myStartDrawingButton->enable();
    myStopDrawingButton->disable();
    myAbortDrawingButton->disable();
}


void
GNEFrameModules::DrawingShape::addNewPoint(const Position& P) {
    if (myStopDrawingButton->isEnabled()) {
        myTemporalShape.push_back(P);
    } else {
        throw ProcessError("A new point cannot be added if drawing wasn't started");
    }
}


void
GNEFrameModules::DrawingShape::removeLastPoint() {
    if (myTemporalShape.size() > 1) {
        myTemporalShape.pop_back();
    }
}


const PositionVector&
GNEFrameModules::DrawingShape::getTemporalShape() const {
    return myTemporalShape;
}


bool
GNEFrameModules::DrawingShape::isDrawing() const {
    return myStopDrawingButton->isEnabled();
}


void
GNEFrameModules::DrawingShape::setDeleteLastCreatedPoint(bool value) {
    myDeleteLastCreatedPoint = value;
}


bool
GNEFrameModules::DrawingShape::getDeleteLastCreatedPoint() {
    return myDeleteLastCreatedPoint;
}


long
GNEFrameModules::DrawingShape::onCmdStartDrawing(FXObject*, FXSelector, void*) {
    startDrawing();
    return 0;
}


long
GNEFrameModules::DrawingShape::onCmdStopDrawing(FXObject*, FXSelector, void*) {
    stopDrawing();
    return 0;
}


long
GNEFrameModules::DrawingShape::onCmdAbortDrawing(FXObject*, FXSelector, void*) {
    abortDrawing();
    return 0;
}

// ---------------------------------------------------------------------------
// GNEFrameModules::SelectorParent - methods
// ---------------------------------------------------------------------------

GNEFrameModules::SelectorParent::SelectorParent(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Parent selector"),
    myFrameParent(frameParent) {
    // Create label with the type of SelectorParent
    myParentsLabel = new FXLabel(getCollapsableFrame(), "No additional selected", nullptr, GUIDesignLabelLeftThick);
    // Create list
    myParentsList = new FXList(getCollapsableFrame(), this, MID_GNE_SET_TYPE, GUIDesignListSingleElementFixedHeight);
    // Hide List
    hideSelectorParentModule();
}


GNEFrameModules::SelectorParent::~SelectorParent() {}


std::string
GNEFrameModules::SelectorParent::getIdSelected() const {
    for (int i = 0; i < myParentsList->getNumItems(); i++) {
        if (myParentsList->isItemSelected(i)) {
            return myParentsList->getItem(i)->getText().text();
        }
    }
    return "";
}


void
GNEFrameModules::SelectorParent::setIDSelected(const std::string& id) {
    // first unselect all
    for (int i = 0; i < myParentsList->getNumItems(); i++) {
        myParentsList->getItem(i)->setSelected(false);
    }
    // select element if correspond to given ID
    for (int i = 0; i < myParentsList->getNumItems(); i++) {
        if (myParentsList->getItem(i)->getText().text() == id) {
            myParentsList->getItem(i)->setSelected(true);
        }
    }
    // recalc myFirstParentsList
    myParentsList->recalc();
}


bool
GNEFrameModules::SelectorParent::showSelectorParentModule(const std::vector<SumoXMLTag>& additionalTypeParents) {
    // make sure that we're editing an additional tag
    const auto listOfTags = GNEAttributeCarrier::getTagPropertiesByType(GNETagProperties::TagType::ADDITIONALELEMENT);
    for (const auto& tagIt : listOfTags) {
        if (std::find(additionalTypeParents.begin(), additionalTypeParents.end(), tagIt.getTag()) != additionalTypeParents.end()) {
            myParentTags = additionalTypeParents;
            myParentsLabel->setText(("Parent type: " + tagIt.getFieldString()).c_str());
            refreshSelectorParentModule();
            show();
            return true;
        }
    }
    return false;
}


void
GNEFrameModules::SelectorParent::hideSelectorParentModule() {
    myParentTags.clear();
    hide();
}


void
GNEFrameModules::SelectorParent::refreshSelectorParentModule() {
    // save current edited elements
    std::set<std::string> selectedItems;
    for (int i = 0; i < myParentsList->getNumItems(); i++) {
        if (myParentsList->isItemSelected(i)) {
            selectedItems.insert(myParentsList->getItem(i)->getText().text());
        }
    }
    myParentsList->clearItems();
    if (myParentTags.size() > 0) {
        // insert additionals sorted
        std::set<std::string> IDs;
        // fill list with IDs of additionals
        for (const auto& parentTag : myParentTags) {
            for (const auto& additional : myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getAdditionals().at(parentTag)) {
                IDs.insert(additional->getID().c_str());
            }
        }
        // fill list with IDs of additionals
        for (const auto& ID : IDs) {
            const int item = myParentsList->appendItem(ID.c_str());
            if (selectedItems.find(ID) != selectedItems.end()) {
                myParentsList->selectItem(item);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// GNEFrameModules::OverlappedInspection - methods
// ---------------------------------------------------------------------------

GNEFrameModules::OverlappedInspection::OverlappedInspection(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Overlapped elements"),
    myFrameParent(frameParent),
    myFilteredTag(SUMO_TAG_NOTHING),
    myItemIndex(0) {
    // build elements
    buildFXElements();
}


GNEFrameModules::OverlappedInspection::OverlappedInspection(GNEFrame* frameParent, const SumoXMLTag filteredTag) :
    FXGroupBoxModule(frameParent->myContentFrame, ("Overlapped " + toString(filteredTag) + "s").c_str()),
    myFrameParent(frameParent),
    myFilteredTag(filteredTag),
    myItemIndex(0) {
    // build elements
    buildFXElements();
}


GNEFrameModules::OverlappedInspection::~OverlappedInspection() {}


void
GNEFrameModules::OverlappedInspection::showOverlappedInspection(const GNEViewNetHelper::ObjectsUnderCursor& objectsUnderCursor, const Position& clickedPosition) {
    // first clear myOverlappedACs
    myOverlappedACs.clear();
    // reserve
    myOverlappedACs.reserve(objectsUnderCursor.getClickedAttributeCarriers().size());
    // iterate over objects under cursor
    for (const auto& AC : objectsUnderCursor.getClickedAttributeCarriers()) {
        bool insert = true;
        // check supermode demand
        if (myFrameParent->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                !AC->getTagProperty().isDemandElement()) {
            insert = false;
        }
        // check supermode data
        if (myFrameParent->getViewNet()->getEditModes().isCurrentSupermodeData() &&
                !AC->getTagProperty().isGenericData()) {
            insert = false;
        }
        // check filter
        if ((myFilteredTag != SUMO_TAG_NOTHING) && (AC->getTagProperty().getTag() != myFilteredTag)) {
            insert = false;
        }
        if (insert) {
            myOverlappedACs.push_back(AC);
        }
    }
    // continue depending of number of myOverlappedACs
    if (myOverlappedACs.size() > 1) {
        mySavedClickedPosition = clickedPosition;
        // by default we inspect first element
        myItemIndex = 0;
        // update text of current index button
        myCurrentIndexButton->setText(("1 / " + toString(myOverlappedACs.size())).c_str());
        // clear and fill list again
        myOverlappedElementList->clearItems();
        for (int i = 0; i < (int)myOverlappedACs.size(); i++) {
            myOverlappedElementList->insertItem(i, myOverlappedACs.at(i)->getID().c_str(), myOverlappedACs.at(i)->getIcon());
        }
        // set first element as selected element
        myOverlappedElementList->getItem(0)->setSelected(TRUE);
        // by default list hidden
        myOverlappedElementList->hide();
        // show OverlappedInspection modul
        show();
    } else {
        // hide OverlappedInspection modul
        hide();
    }
}


void
GNEFrameModules::OverlappedInspection::hideOverlappedInspection() {
    // hide OverlappedInspection modul
    hide();
}


bool
GNEFrameModules::OverlappedInspection::overlappedInspectionShown() const {
    // show OverlappedInspection modul
    return shown();
}


int
GNEFrameModules::OverlappedInspection::getNumberOfOverlappedACs() const {
    return (int)myOverlappedACs.size();
}


bool
GNEFrameModules::OverlappedInspection::checkSavedPosition(const Position& clickedPosition) const {
    return (mySavedClickedPosition.distanceSquaredTo2D(clickedPosition) < 0.25);
}


bool
GNEFrameModules::OverlappedInspection::nextElement(const Position& clickedPosition) {
    // first check if OverlappedInspection is shown
    if (shown()) {
        // check if given position is near saved position
        if (checkSavedPosition(clickedPosition)) {
            // inspect next element
            onCmdNextElement(0, 0, 0);
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


bool
GNEFrameModules::OverlappedInspection::previousElement(const Position& clickedPosition) {
    // first check if OverlappedInspection is shown
    if (shown()) {
        // check if given position is near saved position
        if (checkSavedPosition(clickedPosition)) {
            // inspect previousElement
            onCmdPreviousElement(0, 0, 0);
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


long
GNEFrameModules::OverlappedInspection::onCmdPreviousElement(FXObject*, FXSelector, void*) {
    // check if there is items
    if (myOverlappedElementList->getNumItems() > 0) {
        // unselect current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(FALSE);
        // set index (it works as a ring)
        if (myItemIndex > 0) {
            myItemIndex--;
        } else {
            myItemIndex = (myOverlappedACs.size() - 1);
        }
        // selected current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(TRUE);
        myOverlappedElementList->update();
        // update current index button
        myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
        // inspect overlapped attribute carrier
        myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
        // show OverlappedInspection again (because it's hidden in inspectSingleElement)
        show();
    }
    return 1;
}


long
GNEFrameModules::OverlappedInspection::onCmdNextElement(FXObject*, FXSelector, void*) {
    // check if there is items
    if (myOverlappedElementList->getNumItems() > 0) {
        // unselect current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(FALSE);
        // set index (it works as a ring)
        myItemIndex = (myItemIndex + 1) % myOverlappedACs.size();
        // selected current list element
        myOverlappedElementList->getItem((int)myItemIndex)->setSelected(TRUE);
        myOverlappedElementList->update();
        // update current index button
        myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
        // inspect overlapped attribute carrier
        myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
        // show OverlappedInspection again (because it's hidden in inspectSingleElement)
        show();
    }
    return 1;
}


long
GNEFrameModules::OverlappedInspection::onCmdShowList(FXObject*, FXSelector, void*) {
    // show or hidde element list
    if (myOverlappedElementList->shown()) {
        myOverlappedElementList->hide();
    } else {
        myOverlappedElementList->show();
    }
    if (myOverlappedElementList->getNumItems() <= 10) {
        myOverlappedElementList->setHeight(23 * myOverlappedElementList->getNumItems());
    } else {
        myOverlappedElementList->setHeight(230);
    }
    myOverlappedElementList->recalc();
    // recalc and update frame
    recalc();
    return 1;
}

long
GNEFrameModules::OverlappedInspection::onCmdListItemSelected(FXObject*, FXSelector, void*) {
    for (int i = 0; i < myOverlappedElementList->getNumItems(); i++) {
        if (myOverlappedElementList->getItem(i)->isSelected()) {
            myItemIndex = i;
            // update current index button
            myCurrentIndexButton->setText((toString(myItemIndex + 1) + " / " + toString(myOverlappedACs.size())).c_str());
            // inspect overlapped attribute carrier
            myFrameParent->selectedOverlappedElement(myOverlappedACs.at(myItemIndex));
            // show OverlappedInspection again (because it's hidden in inspectSingleElement)
            show();
            return 1;
        }
    }
    return 0;
}


long
GNEFrameModules::OverlappedInspection::onCmdOverlappingHelp(FXObject*, FXSelector, void*) {
    FXDialogBox* helpDialog = new FXDialogBox(getCollapsableFrame(), "GEO attributes Help", GUIDesignDialogBox);
    std::ostringstream help;
    help
            << " - Click in the same position\n"
            << "   for inspect next element\n"
            << " - Shift + Click in the same\n"
            << "   position for inspect\n"
            << "   previous element";
    new FXLabel(helpDialog, help.str().c_str(), nullptr, GUIDesignLabelFrameInformation);
    // "OK"
    new FXButton(helpDialog, "OK\t\tclose", GUIIconSubSys::getIcon(GUIIcon::ACCEPT), helpDialog, FXDialogBox::ID_ACCEPT, GUIDesignButtonOK);
    helpDialog->create();
    helpDialog->show();
    return 1;
}


GNEFrameModules::OverlappedInspection::OverlappedInspection() :
    myFrameParent(nullptr),
    myPreviousElement(nullptr),
    myCurrentIndexButton(nullptr),
    myNextElement(nullptr),
    myOverlappedElementList(nullptr),
    myHelpButton(nullptr),
    myFilteredTag(SUMO_TAG_NOTHING),
    myItemIndex(0) {
}


void
GNEFrameModules::OverlappedInspection::buildFXElements() {
    FXHorizontalFrame* frameButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // Create previous Item Button
    myPreviousElement = new FXButton(frameButtons, "", GUIIconSubSys::getIcon(GUIIcon::BIGARROWLEFT), this, MID_GNE_OVERLAPPED_PREVIOUS, GUIDesignButtonIconRectangular);
    // create current index button
    myCurrentIndexButton = new FXButton(frameButtons, "", nullptr, this, MID_GNE_OVERLAPPED_SHOWLIST, GUIDesignButton);
    // Create next Item Button
    myNextElement = new FXButton(frameButtons, "", GUIIconSubSys::getIcon(GUIIcon::BIGARROWRIGHT), this, MID_GNE_OVERLAPPED_NEXT, GUIDesignButtonIconRectangular);
    // Create list of overlapped elements (by default hidden)
    myOverlappedElementList = new FXList(getCollapsableFrame(), this, MID_GNE_OVERLAPPED_ITEMSELECTED, GUIDesignListFixedHeight);
    // by default list of overlapped elements is hidden)
    myOverlappedElementList->hide();
    // Create help button
    myHelpButton = new FXButton(getCollapsableFrame(), "Help", nullptr, this, MID_HELP, GUIDesignButtonRectangular);
}

// ---------------------------------------------------------------------------
// GNEFrameModules::PathCreator - methods
// ---------------------------------------------------------------------------

GNEFrameModules::PathCreator::Path::Path(const SUMOVehicleClass vClass, GNEEdge* edge) :
    mySubPath({edge}),
          myFromBusStop(nullptr),
          myToBusStop(nullptr),
          myConflictVClass(false),
myConflictDisconnected(false) {
    // check if we have to change vClass flag
    if (edge->getNBEdge()->getNumLanesThatAllow(vClass) == 0) {
        myConflictVClass = true;
    }
}


GNEFrameModules::PathCreator::Path::Path(GNEViewNet* viewNet, const SUMOVehicleClass vClass, GNEEdge* edgeFrom, GNEEdge* edgeTo) :
    myFromBusStop(nullptr),
    myToBusStop(nullptr),
    myConflictVClass(false),
    myConflictDisconnected(false) {
    // calculate subpath
    mySubPath = viewNet->getNet()->getPathManager()->getPathCalculator()->calculateDijkstraPath(vClass, {edgeFrom, edgeTo});
    // if subPath is empty, try it with pedestrian (i.e. ignoring vCass)
    if (mySubPath.empty()) {
        mySubPath = viewNet->getNet()->getPathManager()->getPathCalculator()->calculateDijkstraPath(SVC_PEDESTRIAN, {edgeFrom, edgeTo});
        if (mySubPath.empty()) {
            mySubPath = { edgeFrom, edgeTo };
            myConflictDisconnected = true;
        } else {
            myConflictVClass = true;
        }
    }
}


const std::vector<GNEEdge*>&
GNEFrameModules::PathCreator::Path::getSubPath() const {
    return mySubPath;
}


GNEAdditional* GNEFrameModules::PathCreator::Path::getFromBusStop() const {
    return myFromBusStop;
}


GNEAdditional* GNEFrameModules::PathCreator::Path::getToBusStop() const {
    return myToBusStop;
}


bool
GNEFrameModules::PathCreator::Path::isConflictVClass() const {
    return myConflictVClass;
}


bool
GNEFrameModules::PathCreator::Path::isConflictDisconnected() const {
    return myConflictDisconnected;
}


GNEFrameModules::PathCreator::Path::Path() :
    myFromBusStop(nullptr),
    myToBusStop(nullptr),
    myConflictVClass(false),
    myConflictDisconnected(false) {
}


GNEFrameModules::PathCreator::PathCreator(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Route creator"),
    myFrameParent(frameParent),
    myVClass(SVC_PASSENGER),
    myCreationMode(0),
    myToStoppingPlace(nullptr),
    myRoute(nullptr) {
    // create label for route info
    myInfoRouteLabel = new FXLabel(getCollapsableFrame(), "No edges selected", 0, GUIDesignLabelFrameThicked);
    // create button for finish route creation
    myFinishCreationButton = new FXButton(getCollapsableFrame(), "Finish route creation", nullptr, this, MID_GNE_EDGEPATH_FINISH, GUIDesignButton);
    myFinishCreationButton->disable();
    // create button for abort route creation
    myAbortCreationButton = new FXButton(getCollapsableFrame(), "Abort route creation", nullptr, this, MID_GNE_EDGEPATH_ABORT, GUIDesignButton);
    myAbortCreationButton->disable();
    // create button for remove last inserted edge
    myRemoveLastInsertedElement = new FXButton(getCollapsableFrame(), "Remove last inserted edge", nullptr, this, MID_GNE_EDGEPATH_REMOVELAST, GUIDesignButton);
    myRemoveLastInsertedElement->disable();
    // create check button
    myShowCandidateEdges = new FXCheckButton(getCollapsableFrame(), "Show candidate edges", this, MID_GNE_EDGEPATH_SHOWCANDIDATES, GUIDesignCheckButton);
    myShowCandidateEdges->setCheck(TRUE);
    // create shift label
    myShiftLabel = new FXLabel(this,
                               "SHIFT-click: ignore vClass",
                               0, GUIDesignLabelFrameInformation);
    // create control label
    myControlLabel = new FXLabel(this,
                                 "CTRL-click: add disconnected",
                                 0, GUIDesignLabelFrameInformation);
    // create backspace label (always shown)
    new FXLabel(this,
                "BACKSPACE: undo click",
                0, GUIDesignLabelFrameInformation);
}


GNEFrameModules::PathCreator::~PathCreator() {}


void
GNEFrameModules::PathCreator::showPathCreatorModule(SumoXMLTag element, const bool firstElement, const bool consecutives) {
    // declare flag
    bool showPathCreator = true;
    // first abort creation
    abortPathCreation();
    // disable buttons
    myFinishCreationButton->disable();
    myAbortCreationButton->disable();
    myRemoveLastInsertedElement->disable();
    // reset creation mode
    myCreationMode = 0;
    // set first element
    if (firstElement) {
        myCreationMode |= REQUIRE_FIRSTELEMENT;
    }
    // set consecutive or non consecuives
    if (consecutives) {
        myCreationMode |= CONSECUTIVE_EDGES;
    } else {
        myCreationMode |= NONCONSECUTIVE_EDGES;
    }
    // set specific mode depending of tag
    switch (element) {
        // routes
        case SUMO_TAG_ROUTE:
        case GNE_TAG_ROUTE_EMBEDDED:
            myCreationMode |= SHOW_CANDIDATE_EDGES;
            myCreationMode |= START_EDGE;
            myCreationMode |= END_EDGE;
            break;
        // vehicles
        case SUMO_TAG_VEHICLE:
        case GNE_TAG_FLOW_ROUTE:
        case GNE_TAG_WALK_ROUTE:
            myCreationMode |= SINGLE_ELEMENT;
            myCreationMode |= ROUTE;
            break;
        case SUMO_TAG_TRIP:
        case SUMO_TAG_FLOW:
        case GNE_TAG_VEHICLE_WITHROUTE:
        case GNE_TAG_FLOW_WITHROUTE:
            myCreationMode |= SHOW_CANDIDATE_EDGES;
            myCreationMode |= START_EDGE;
            myCreationMode |= END_EDGE;
            break;
        case GNE_TAG_TRIP_JUNCTIONS:
        case GNE_TAG_FLOW_JUNCTIONS:
            myCreationMode |= START_JUNCTION;
            myCreationMode |= END_JUNCTION;
            myCreationMode |= ONLY_FROMTO;
            break;
        // walk edges
        case GNE_TAG_WALK_EDGES:
            myCreationMode |= SHOW_CANDIDATE_EDGES;
            myCreationMode |= START_EDGE;
            myCreationMode |= END_EDGE;
            break;
        // edge->edge
        case GNE_TAG_PERSONTRIP_EDGE:
        case GNE_TAG_RIDE_EDGE:
        case GNE_TAG_WALK_EDGE:
            myCreationMode |= SHOW_CANDIDATE_EDGES;
            myCreationMode |= ONLY_FROMTO;
            myCreationMode |= START_EDGE;
            myCreationMode |= END_EDGE;
            break;
        // edge->busStop
        case GNE_TAG_PERSONTRIP_BUSSTOP:
        case GNE_TAG_RIDE_BUSSTOP:
        case GNE_TAG_WALK_BUSSTOP:
            myCreationMode |= SHOW_CANDIDATE_EDGES;
            myCreationMode |= ONLY_FROMTO;
            myCreationMode |= END_BUSSTOP;
            break;
        // junction->junction
        case GNE_TAG_PERSONTRIP_JUNCTIONS:
        case GNE_TAG_WALK_JUNCTIONS:
            myCreationMode |= START_JUNCTION;
            myCreationMode |= END_JUNCTION;
            myCreationMode |= ONLY_FROMTO;
            break;
        // stops
        case GNE_TAG_STOPPERSON_BUSSTOP:
            myCreationMode |= SINGLE_ELEMENT;
            myCreationMode |= END_BUSSTOP;
            break;
        case GNE_TAG_STOPPERSON_EDGE:
            myCreationMode |= SINGLE_ELEMENT;
            myCreationMode |= START_EDGE;
            break;
        // generic datas
        case SUMO_TAG_EDGEREL:
            myCreationMode |= ONLY_FROMTO;
            myCreationMode |= START_EDGE;
            myCreationMode |= END_EDGE;
            break;
        default:
            showPathCreator = false;
            break;
    }
    // check if show path creator
    if (showPathCreator) {
        // update colors
        if (myCreationMode & SHOW_CANDIDATE_EDGES) {
            updateEdgeColors();
        }
        if (myCreationMode & START_JUNCTION) {
            updateEdgeColors();
        }
        // recalc before show (to avoid graphic problems)
        recalc();
        // show modul
        show();
    } else {
        // hide modul
        hide();
    }
}


void
GNEFrameModules::PathCreator::hidePathCreatorModule() {
    // clear path
    clearPath();
    // hide modul
    hide();
}


SUMOVehicleClass
GNEFrameModules::PathCreator::getVClass() const {
    return myVClass;
}


void
GNEFrameModules::PathCreator::setVClass(SUMOVehicleClass vClass) {
    myVClass = vClass;
    // update edge colors
    updateEdgeColors();
}


bool
GNEFrameModules::PathCreator::addJunction(GNEJunction* junction, const bool /* shiftKeyPressed */, const bool /* controlKeyPressed */) {
    // check if junctions are allowed
    if (((myCreationMode & START_JUNCTION) + (myCreationMode & END_JUNCTION)) == 0) {
        return false;
    }
    // check if only an junction is allowed
    if ((myCreationMode & SINGLE_ELEMENT) && (mySelectedJunctions.size() == 1)) {
        return false;
    }
    // continue depending of number of selected edge
    if (mySelectedJunctions.size() > 0) {
        // check double junctions
        if (mySelectedJunctions.back() == junction) {
            // Write warning
            WRITE_WARNING("Double junctions aren't allowed");
            // abort add junction
            return false;
        }
    }
    // check number of junctions
    if (mySelectedJunctions.size() == 2 && (myCreationMode & Mode::ONLY_FROMTO)) {
        // Write warning
        WRITE_WARNING("Only two junctions are allowed");
        // abort add junction
        return false;
    }
    // All checks ok, then add it in selected elements
    mySelectedJunctions.push_back(junction);
    // enable abort route button
    myAbortCreationButton->enable();
    // enable finish button
    myFinishCreationButton->enable();
    // disable undo/redo
    myFrameParent->myViewNet->getViewParent()->getGNEAppWindows()->disableUndoRedo("route creation");
    // enable or disable remove last junction button
    if (mySelectedJunctions.size() > 1) {
        myRemoveLastInsertedElement->enable();
    } else {
        myRemoveLastInsertedElement->disable();
    }
    // recalculate path
    recalculatePath();
    // update info route label
    updateInfoRouteLabel();
    // update junction colors
    updateJunctionColors();
    return true;
}


bool
GNEFrameModules::PathCreator::addEdge(GNEEdge* edge, const bool shiftKeyPressed, const bool controlKeyPressed) {
    // check if edges are allowed
    if (((myCreationMode & CONSECUTIVE_EDGES) + (myCreationMode & NONCONSECUTIVE_EDGES) +
            (myCreationMode & START_EDGE) + (myCreationMode & END_EDGE)) == 0) {
        return false;
    }
    // check if only an edge is allowed
    if ((myCreationMode & SINGLE_ELEMENT) && (mySelectedEdges.size() == 1)) {
        return false;
    }
    // continue depending of number of selected eges
    if (mySelectedEdges.size() > 0) {
        // check double edges
        if (mySelectedEdges.back() == edge) {
            // Write warning
            WRITE_WARNING("Double edges aren't allowed");
            // abort add edge
            return false;
        }
        // check consecutive edges
        if (myCreationMode & Mode::CONSECUTIVE_EDGES) {
            // check that new edge is consecutive
            const auto& outgoingEdges = mySelectedEdges.back()->getToJunction()->getGNEOutgoingEdges();
            if (std::find(outgoingEdges.begin(), outgoingEdges.end(), edge) == outgoingEdges.end()) {
                // Write warning
                WRITE_WARNING("Only consecutives edges are allowed");
                // abort add edge
                return false;
            }
        }
    }
    // check number of edges
    if (mySelectedEdges.size() == 2 && (myCreationMode & Mode::ONLY_FROMTO)) {
        // Write warning
        WRITE_WARNING("Only two edges are allowed");
        // abort add edge
        return false;
    }
    // check candidate edge
    if ((myShowCandidateEdges->getCheck() == TRUE) && !edge->isPossibleCandidate()) {
        if (edge->isSpecialCandidate()) {
            if (!shiftKeyPressed) {
                // Write warning
                WRITE_WARNING("Invalid edge (SHIFT + click to add an invalid vClass edge)");
                // abort add edge
                return false;
            }
        } else if (edge->isConflictedCandidate()) {
            if (!controlKeyPressed) {
                // Write warning
                WRITE_WARNING("Invalid edge (CONTROL + click to add a disconnected edge)");
                // abort add edge
                return false;
            }
        }
    }
    // All checks ok, then add it in selected elements
    mySelectedEdges.push_back(edge);
    // enable abort route button
    myAbortCreationButton->enable();
    // enable finish button
    myFinishCreationButton->enable();
    // disable undo/redo
    myFrameParent->myViewNet->getViewParent()->getGNEAppWindows()->disableUndoRedo("route creation");
    // enable or disable remove last edge button
    if (mySelectedEdges.size() > 1) {
        myRemoveLastInsertedElement->enable();
    } else {
        myRemoveLastInsertedElement->disable();
    }
    // recalculate path
    recalculatePath();
    // update info route label
    updateInfoRouteLabel();
    // update edge colors
    updateEdgeColors();
    return true;
}


const std::vector<GNEEdge*>&
GNEFrameModules::PathCreator::getSelectedEdges() const {
    return mySelectedEdges;
}


const std::vector<GNEJunction*>&
GNEFrameModules::PathCreator::getSelectedJunctions() const {
    return mySelectedJunctions;
}


bool
GNEFrameModules::PathCreator::addStoppingPlace(GNEAdditional* stoppingPlace, const bool /*shiftKeyPressed*/, const bool /*controlKeyPressed*/) {
    // check if stoppingPlaces aren allowed
    if ((myCreationMode & END_BUSSTOP) == 0) {
        return false;
    }
    // check if previously stopping place from was set
    if (myToStoppingPlace) {
        return false;
    } else {
        myToStoppingPlace = stoppingPlace;
    }
    // enable abort route button
    myAbortCreationButton->enable();
    // enable finish button
    myFinishCreationButton->enable();
    // disable undo/redo
    myFrameParent->myViewNet->getViewParent()->getGNEAppWindows()->disableUndoRedo("route creation");
    // enable or disable remove last stoppingPlace button
    if (myToStoppingPlace) {
        myRemoveLastInsertedElement->enable();
    } else {
        myRemoveLastInsertedElement->disable();
    }
    // recalculate path
    recalculatePath();
    // update info route label
    updateInfoRouteLabel();
    // update stoppingPlace colors
    updateEdgeColors();
    return true;
}


GNEAdditional*
GNEFrameModules::PathCreator::getToStoppingPlace(SumoXMLTag expectedTag) const {
    if (myToStoppingPlace && (myToStoppingPlace->getTagProperty().getTag() == expectedTag)) {
        return myToStoppingPlace;
    } else {
        return nullptr;
    }
}


bool
GNEFrameModules::PathCreator::addRoute(GNEDemandElement* route, const bool /*shiftKeyPressed*/, const bool /*controlKeyPressed*/) {
    // check if routes aren allowed
    if ((myCreationMode & ROUTE) == 0) {
        return false;
    }
    // check if previously a route was added
    if (myRoute) {
        return false;
    }
    // set route
    myRoute = route;
    // recalculate path
    recalculatePath();
    updateInfoRouteLabel();
    updateEdgeColors();
    return true;
}


void
GNEFrameModules::PathCreator::removeRoute() {
    // set route
    myRoute = nullptr;
    // recalculate path
    recalculatePath();
    updateInfoRouteLabel();
    updateEdgeColors();
}


GNEDemandElement*
GNEFrameModules::PathCreator::getRoute() const {
    return myRoute;
}


const std::vector<GNEFrameModules::PathCreator::Path>&
GNEFrameModules::PathCreator::getPath() const {
    return myPath;
}


bool
GNEFrameModules::PathCreator::drawCandidateEdgesWithSpecialColor() const {
    return (myShowCandidateEdges->getCheck() == TRUE);
}


void
GNEFrameModules::PathCreator::updateJunctionColors() {
    // reset all flags
    for (const auto& junction : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getJunctions()) {
        junction.second->resetCandidateFlags();
        junction.second->setPossibleCandidate(true);
    }
    // set selected junctions
    if (mySelectedJunctions.size() > 0) {
        // mark selected eges
        for (const auto& junction : mySelectedJunctions) {
            junction->resetCandidateFlags();
            junction->setSourceCandidate(true);
        }
        // finally mark last selected element as target
        mySelectedJunctions.back()->resetCandidateFlags();
        mySelectedJunctions.back()->setTargetCandidate(true);
    }
    // update view net
    myFrameParent->myViewNet->updateViewNet();
}


void
GNEFrameModules::PathCreator::updateEdgeColors() {
    // reset all flags
    clearEdgeColors();
    // set reachability
    if (mySelectedEdges.size() > 0) {
        // only coloring edges if checkbox "show candidate edges" is enabled
        if ((myShowCandidateEdges->getCheck() == TRUE) && (myCreationMode & SHOW_CANDIDATE_EDGES)) {
            // mark all edges as conflicted (to mark special candidates)
            for (const auto& edge : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getEdges()) {
                edge.second->setConflictedCandidate(true);
            }
            // set special candidates (Edges that are connected but aren't compatibles with current vClass
            setSpecialCandidates(mySelectedEdges.back());
            // mark again all edges as conflicted (to mark possible candidates)
            for (const auto& edge : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getEdges()) {
                edge.second->setConflictedCandidate(true);
            }
            // set possible candidates (Edges that are connected AND are compatibles with current vClass
            setPossibleCandidates(mySelectedEdges.back(), myVClass);
        }
        // now mark selected eges
        for (const auto& edge : mySelectedEdges) {
            edge->resetCandidateFlags();
            edge->setSourceCandidate(true);
        }
        // finally mark last selected element as target
        mySelectedEdges.back()->resetCandidateFlags();
        mySelectedEdges.back()->setTargetCandidate(true);
    } else if (myShowCandidateEdges->getCheck() == TRUE && (myCreationMode & SHOW_CANDIDATE_EDGES)) {
        // mark all edges that have at least one lane that allow given vClass
        for (const auto& edge : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getEdges()) {
            if (edge.second->getNBEdge()->getNumLanesThatAllow(myVClass) > 0) {
                edge.second->setPossibleCandidate(true);
            } else {
                edge.second->setSpecialCandidate(true);
            }
        }
    }
    // update view net
    myFrameParent->myViewNet->updateViewNet();
}


void
GNEFrameModules::PathCreator::clearJunctionColors() {
    // reset all junction flags
    for (const auto& junction : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getJunctions()) {
        junction.second->resetCandidateFlags();
    }
}


void
GNEFrameModules::PathCreator::clearEdgeColors() {
    // reset all junction flags
    for (const auto& edge : myFrameParent->myViewNet->getNet()->getAttributeCarriers()->getEdges()) {
        edge.second->resetCandidateFlags();
    }
}


void
GNEFrameModules::PathCreator::drawTemporalRoute(const GUIVisualizationSettings& s) const {
    const double lineWidth = 0.35;
    const double lineWidthin = 0.25;
    // Add a draw matrix
    GLHelper::pushMatrix();
    // Start with the drawing of the area traslating matrix to origin
    glTranslated(0, 0, GLO_MAX - 0.1);
    // check if draw bewteen junction or edges
    if (myPath.size() > 0) {
        // set first color
        GLHelper::setColor(RGBColor::GREY);
        // iterate over path
        for (int i = 0; i < (int)myPath.size(); i++) {
            // get path
            const GNEFrameModules::PathCreator::Path& path = myPath.at(i);
            // draw line over
            for (int j = 0; j < (int)path.getSubPath().size(); j++) {
                const GNELane* lane = path.getSubPath().at(j)->getLanes().back();
                if (((i == 0) && (j == 0)) || (j > 0)) {
                    GLHelper::drawBoxLines(lane->getLaneShape(), lineWidth);
                }
                // draw connection between lanes
                if ((j + 1) < (int)path.getSubPath().size()) {
                    const GNELane* nextLane = path.getSubPath().at(j + 1)->getLanes().back();
                    if (lane->getLane2laneConnections().exist(nextLane)) {
                        GLHelper::drawBoxLines(lane->getLane2laneConnections().getLane2laneGeometry(nextLane).getShape(), lineWidth);
                    } else {
                        GLHelper::drawBoxLines({lane->getLaneShape().back(), nextLane->getLaneShape().front()}, lineWidth);
                    }
                }
            }
        }
        glTranslated(0, 0, 0.1);
        // iterate over path again
        for (int i = 0; i < (int)myPath.size(); i++) {
            // get path
            const GNEFrameModules::PathCreator::Path& path = myPath.at(i);
            // set path color color
            if ((myCreationMode & SHOW_CANDIDATE_EDGES) == 0) {
                GLHelper::setColor(RGBColor::ORANGE);
            } else if (path.isConflictDisconnected()) {
                GLHelper::setColor(s.candidateColorSettings.conflict);
            } else if (path.isConflictVClass()) {
                GLHelper::setColor(s.candidateColorSettings.special);
            } else {
                GLHelper::setColor(RGBColor::ORANGE);
            }
            // draw line over
            for (int j = 0; j < (int)path.getSubPath().size(); j++) {
                const GNELane* lane = path.getSubPath().at(j)->getLanes().back();
                if (((i == 0) && (j == 0)) || (j > 0)) {
                    GLHelper::drawBoxLines(lane->getLaneShape(), lineWidthin);
                }
                // draw connection between lanes
                if ((j + 1) < (int)path.getSubPath().size()) {
                    const GNELane* nextLane = path.getSubPath().at(j + 1)->getLanes().back();
                    if (lane->getLane2laneConnections().exist(nextLane)) {
                        GLHelper::drawBoxLines(lane->getLane2laneConnections().getLane2laneGeometry(nextLane).getShape(), lineWidthin);
                    } else {
                        GLHelper::drawBoxLines({ lane->getLaneShape().back(), nextLane->getLaneShape().front() }, lineWidthin);
                    }
                }
            }
        }
    } else if (mySelectedJunctions.size() > 0) {
        // set color
        GLHelper::setColor(RGBColor::ORANGE);
        // draw line between junctions
        for (int i = 0; i < (int)mySelectedJunctions.size() - 1; i++) {
            // get two points
            const Position posA = mySelectedJunctions.at(i)->getPositionInView();
            const Position posB = mySelectedJunctions.at(i + 1)->getPositionInView();
            const double rot = ((double)atan2((posB.x() - posA.x()), (posA.y() - posB.y())) * (double) 180.0 / (double)M_PI);
            const double len = posA.distanceTo2D(posB);
            // draw line
            GLHelper::drawBoxLine(posA, rot, len, 0.25);
        }
    }
    // Pop last matrix
    GLHelper::popMatrix();
}


void
GNEFrameModules::PathCreator::createPath() {
    // call create path implemented in frame parent
    myFrameParent->createPath();
}


void
GNEFrameModules::PathCreator::abortPathCreation() {
    // first check that there is elements
    if ((mySelectedJunctions.size() > 0) || (mySelectedEdges.size() > 0) || myToStoppingPlace || myRoute) {
        // unblock undo/redo
        myFrameParent->myViewNet->getViewParent()->getGNEAppWindows()->enableUndoRedo();
        // clear edges
        clearPath();
        // disable buttons
        myFinishCreationButton->disable();
        myAbortCreationButton->disable();
        myRemoveLastInsertedElement->disable();
        // update info route label
        updateInfoRouteLabel();
        // update junction colors
        updateJunctionColors();
        // update edge colors
        updateEdgeColors();
        // update view (to see the new route)
        myFrameParent->getViewNet()->updateViewNet();
    }
}


void
GNEFrameModules::PathCreator::removeLastElement() {
    if (mySelectedEdges.size() > 1) {
        // remove special color of last selected edge
        mySelectedEdges.back()->resetCandidateFlags();
        // remove last edge
        mySelectedEdges.pop_back();
        // change last edge flag
        if ((mySelectedEdges.size() > 0) && mySelectedEdges.back()->isSourceCandidate()) {
            mySelectedEdges.back()->setSourceCandidate(false);
            mySelectedEdges.back()->setTargetCandidate(true);
        }
        // enable or disable remove last edge button
        if (mySelectedEdges.size() > 1) {
            myRemoveLastInsertedElement->enable();
        } else {
            myRemoveLastInsertedElement->disable();
        }
        // recalculate path
        recalculatePath();
        // update info route label
        updateInfoRouteLabel();
        // update junction colors
        updateJunctionColors();
        // update edge colors
        updateEdgeColors();
        // update view
        myFrameParent->myViewNet->updateViewNet();
    }
}


long
GNEFrameModules::PathCreator::onCmdCreatePath(FXObject*, FXSelector, void*) {
    // just call create path
    createPath();
    return 1;
}


long
GNEFrameModules::PathCreator::onCmdAbortPathCreation(FXObject*, FXSelector, void*) {
    // just call abort path creation
    abortPathCreation();
    return 1;
}


long
GNEFrameModules::PathCreator::onCmdRemoveLastElement(FXObject*, FXSelector, void*) {
    // just call remove last element
    removeLastElement();
    return 1;
}


long
GNEFrameModules::PathCreator::onCmdShowCandidateEdges(FXObject*, FXSelector, void*) {
    // update labels
    if (myShowCandidateEdges->getCheck() == TRUE) {
        myShiftLabel->show();
        myControlLabel->show();
    } else {
        myShiftLabel->hide();
        myControlLabel->hide();
    }
    // recalc frame
    recalc();
    // update edge colors (view will be updated within function)
    updateEdgeColors();
    return 1;
}


void
GNEFrameModules::PathCreator::updateInfoRouteLabel() {
    if (myPath.size() > 0) {
        // declare variables for route info
        double length = 0;
        double speed = 0;
        int pathSize = 0;
        for (const auto& path : myPath) {
            for (const auto& edge : path.getSubPath()) {
                length += edge->getNBEdge()->getLength();
                speed += edge->getNBEdge()->getSpeed();
            }
            pathSize += (int)path.getSubPath().size();
        }
        // declare ostringstream for label and fill it
        std::ostringstream information;
        information
                << "- Selected edges: " << toString(mySelectedEdges.size()) << "\n"
                << "- Path edges: " << toString(pathSize) << "\n"
                << "- Length: " << toString(length) << "\n"
                << "- Average speed: " << toString(speed / pathSize);
        // set new label
        myInfoRouteLabel->setText(information.str().c_str());
    } else {
        myInfoRouteLabel->setText("No edges selected");
    }
}


void
GNEFrameModules::PathCreator::clearPath() {
    /// reset flags
    clearJunctionColors();
    clearEdgeColors();
    // clear junction, edges, additionals and route
    mySelectedJunctions.clear();
    mySelectedEdges.clear();
    myToStoppingPlace = nullptr;
    myRoute = nullptr;
    // clear path
    myPath.clear();
    // update info route label
    updateInfoRouteLabel();
}


void
GNEFrameModules::PathCreator::recalculatePath() {
    // first clear path
    myPath.clear();
    // set edges
    std::vector<GNEEdge*> edges;
    // add route edges
    if (myRoute) {
        edges = myRoute->getParentEdges();
    } else {
        // add selected edges
        for (const auto& edge : mySelectedEdges) {
            edges.push_back(edge);
        }
        // add to stopping place edge
        if (myToStoppingPlace) {
            edges.push_back(myToStoppingPlace->getParentLanes().front()->getParentEdge());
        }
    }
    // fill paths
    if (edges.size() == 1) {
        myPath.push_back(Path(myVClass, edges.front()));
    } else {
        // add every segment
        for (int i = 1; i < (int)edges.size(); i++) {
            myPath.push_back(Path(myFrameParent->getViewNet(), myVClass, edges.at(i - 1), edges.at(i)));
        }
    }
}


void
GNEFrameModules::PathCreator::setSpecialCandidates(GNEEdge* originEdge) {
    // first calculate reachability for pedestrians (we use it, because pedestran can walk in almost all edges)
    myFrameParent->getViewNet()->getNet()->getPathManager()->getPathCalculator()->calculateReachability(SVC_PEDESTRIAN, originEdge);
    // change flags
    for (const auto& edge : myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getEdges()) {
        for (const auto& lane : edge.second->getLanes()) {
            if (lane->getReachability() > 0) {
                lane->getParentEdge()->resetCandidateFlags();
                lane->getParentEdge()->setSpecialCandidate(true);
            }
        }
    }
}

void
GNEFrameModules::PathCreator::setPossibleCandidates(GNEEdge* originEdge, const SUMOVehicleClass vClass) {
    // first calculate reachability for pedestrians
    myFrameParent->getViewNet()->getNet()->getPathManager()->getPathCalculator()->calculateReachability(vClass, originEdge);
    // change flags
    for (const auto& edge : myFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getEdges()) {
        for (const auto& lane : edge.second->getLanes()) {
            if (lane->getReachability() > 0) {
                lane->getParentEdge()->resetCandidateFlags();
                lane->getParentEdge()->setPossibleCandidate(true);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// GNERouteFrame::Legend - methods
// ---------------------------------------------------------------------------

GNEFrameModules::PathLegend::PathLegend(GNEFrame* frameParent) :
    FXGroupBoxModule(frameParent->myContentFrame, "Information") {
    // declare label
    FXLabel* legendLabel = nullptr;
    // edge candidate
    legendLabel = new FXLabel(getCollapsableFrame(), " edge candidate", 0, GUIDesignLabelLeft);
    legendLabel->setBackColor(MFXUtils::getFXColor(frameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.possible));
    legendLabel->setTextColor(MFXUtils::getFXColor(RGBColor::WHITE));
    // last edge selected
    legendLabel = new FXLabel(getCollapsableFrame(), " last edge selected", 0, GUIDesignLabelLeft);
    legendLabel->setBackColor(MFXUtils::getFXColor(frameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.target));
    // edge selected
    legendLabel = new FXLabel(getCollapsableFrame(), " edge selected", 0, GUIDesignLabelLeft);
    legendLabel->setBackColor(MFXUtils::getFXColor(frameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.source));
    // edge conflict (vClass)
    legendLabel = new FXLabel(getCollapsableFrame(), " edge conflict (vClass)", 0, GUIDesignLabelLeft);
    legendLabel->setBackColor(MFXUtils::getFXColor(frameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.special));
    // edge disconnected
    legendLabel = new FXLabel(getCollapsableFrame(), " edge disconnected", 0, GUIDesignLabelLeft);
    legendLabel->setBackColor(MFXUtils::getFXColor(frameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.conflict));
}


GNEFrameModules::PathLegend::~PathLegend() {}


void
GNEFrameModules::PathLegend::showPathLegendModule() {
    show();
}

void
GNEFrameModules::PathLegend::hidePathLegendModule() {
    hide();
}

// ---------------------------------------------------------------------------
// GNEFrameModules - methods
// ---------------------------------------------------------------------------

FXLabel*
GNEFrameModules::buildRainbow(FXComposite* parent) {
    // create label for color information
    FXLabel* label = new FXLabel(parent, "Scale: Min -> Max", nullptr, GUIDesignLabelCenterThick);
    // create frame for color scale
    FXHorizontalFrame* horizontalFrameColors = new FXHorizontalFrame(parent, GUIDesignAuxiliarHorizontalFrame);
    for (const auto& color : GNEViewNetHelper::getRainbowScaledColors()) {
        FXLabel* colorLabel = new FXLabel(horizontalFrameColors, "", nullptr, GUIDesignLabelLeft);
        colorLabel->setBackColor(MFXUtils::getFXColor(color));
    }
    // return label
    return label;
}

/****************************************************************************/
