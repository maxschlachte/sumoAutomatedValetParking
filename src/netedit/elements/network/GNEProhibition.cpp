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
/// @file    GNEProhibition.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2016
///
// A class for visualizing prohibitions between edges
/****************************************************************************/
#include <utils/xml/SUMOXMLDefinitions.h>
#include "GNEProhibition.h"


// ===========================================================================
// static member definitions
// ===========================================================================


// ===========================================================================
// method definitions
// ===========================================================================

GNEProhibition::GNEProhibition(GNENet* net) :
    GNENetworkElement(net, "", GLO_EDGE, SUMO_TAG_PROHIBITION, {}, {}, {}, {}, {}, {}, {}, {}) {}


/****************************************************************************/
