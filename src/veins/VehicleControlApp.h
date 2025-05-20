//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "GraphProcessor.h"
#include <string>
#include <vector>
#include <map>

using namespace omnetpp;

namespace veins {

// Forward declarations
class DemoSafetyMessage;
class DemoServiceAdvertisment;

/**
 * Structure to represent a time window
 */
struct VehicleTimeWindow {
    double earliness;
    double tardiness;

    VehicleTimeWindow(double e = 0.0, double t = 0.0) : earliness(e), tardiness(t) {}
};

/**
 * Structure to represent a destination with time constraints
 */
struct VehicleDestination {
    std::string nodeId;
    VehicleTimeWindow timeWindow;

    VehicleDestination(const std::string& id, const VehicleTimeWindow& tw)
        : nodeId(id), timeWindow(tw) {}
};

/**
 * Vehicle Control Application that communicates with RSUs to get road information
 */
class VehicleControlApp : public TraCIDemo11p {
public:
    void initialize(int stage) override;
    void finish() override;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void handleSelfMsg(cMessage* msg) override;
    void onBSM(DemoSafetyMessage* bsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void handleLowerMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    
private:
    // Messages for periodic updates
    cMessage* statusUpdateMsg = nullptr;
    cMessage* requestRoadInfoMsg = nullptr;

    // Current road and road network data
    std::string currentRoadId;
    std::vector<std::string> allRoads;
    std::vector<std::string> accessibleRoads;
    std::vector<std::string> incomingRoads;
    std::vector<std::string> currentPath;
    std::map<std::string, std::string> currentRoadAttributes;
    std::vector<VehicleDestination> destinations;

    // Path finding components
    std::unique_ptr<GraphProcessor> graphProcessor;
    Graph roadNetwork;

    // RSU communication methods
    void sendStatusUpdate();
    void requestAllRoads();
    void requestAccessibleRoads(const std::string& roadId);
    void requestIncomingRoads(const std::string& roadId);
    void requestRoadAttributes(const std::string& roadId);
    void requestShortestPath(const std::string& sourceId, const std::string& targetId);
    void requestKPaths(const std::string& sourceId, const std::string& targetId, int k);
    void requestDestinations(int count);
    void requestValidAssignment(const std::vector<std::string>& sources, const std::vector<std::string>& destinations);

    // Response processing methods
    void processAllRoadsResponse(const std::string& data);
    void processAccessibleRoadsResponse(const std::string& data);
    void processIncomingRoadsResponse(const std::string& data);
    void processRoadAttributesResponse(const std::string& data);
    void processShortestPathResponse(const std::string& data);
    void processKPathsResponse(const std::string& data);
    void processDestinationsResponse(const std::string& data);
    void processValidAssignmentResponse(const std::string& data);

    // Helper methods
    std::vector<std::string> parseRoadList(const std::string& data, char delimiter = ',');
    std::map<std::string, std::string> parseAttributes(const std::string& data);
    void printRoadInfo();
    void buildLocalRoadNetwork();
    
    // Local path finding methods
    std::vector<std::string> findShortestPath(const std::string& sourceId, const std::string& targetId);
    double getShortestPathLength(const std::string& sourceId, const std::string& targetId);

    // Test methods
    void testPathFinding();
    void runPathFindingTests();
    void testPathBetween(const std::string& source, const std::string& target, const std::string& testName);
};

} // namespace veins
