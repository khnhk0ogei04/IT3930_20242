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
#include "TaskGenerator.h"
#include "SimulationLogger.h"
#include <string>
#include <vector>
#include <map>
#include <chrono>

using namespace omnetpp;

namespace veins {

// Forward declarations
class DemoSafetyMessage;
class DemoServiceAdvertisment;

/**
 * Vehicle Control Application that communicates with RSUs to get road information
 */
class VehicleControlApp : public TraCIDemo11p {
public:
    void initialize(int stage) override;
    void finish() override;

protected:
    virtual void onWSM(BaseFrame1609_4* wsm) override;
    virtual void onBSM(DemoSafetyMessage* bsm) override;
    virtual void onWSA(DemoServiceAdvertisment* wsa) override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void handlePositionUpdate(cObject* obj) override;
    virtual void handleLowerMsg(cMessage* msg) override;
    
    virtual void handleEndOfVehicleLifecycle();
    
private:
    // Messages for periodic communication
    cMessage* statusUpdateMsg;
    cMessage* requestRoadInfoMsg;
    cMessage* cleanupTimer;

    // ID management
    int myInternalId;       // The internal ID used by Veins/OMNeT++
    int mySimulationId;     // The simulation ID visible in the TreeView
    bool hasLoggedDeparture = false;
    
    // Timing and time window information
    double startTime;
    double endTime;
    double earliestArrival;
    double latestArrival;
    std::string startingRoad;
    std::string targetRoad;
    double pathLength;
    
    // Road network data
    std::string currentRoadId;
    std::vector<std::string> allRoads;
    std::vector<std::string> accessibleRoads;
    std::vector<std::string> incomingRoads;
    std::map<std::string, std::string> currentRoadAttributes;
    std::vector<std::string> currentPath;
    std::vector<Destination> destinations;

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

//    // Test methods
//    void testPathFinding();
//    void runPathFindingTests();
//    void testPathBetween(const std::string& source, const std::string& target, const std::string& testName);

    // Vehicle lifecycle management methods
    void cleanupMessages();
    void logDepartureIfNeeded();
    void checkVehicleStatus();
    void updateTimingFile(double endTime);
    
    // Speed control method
    void checkAndAdjustSpeed();
    
    double measureRoutingAlgorithmTime(const std::string& sourceId, const std::string& targetId);
};

} // namespace veins
