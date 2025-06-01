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
using namespace std;

namespace veins {
class DemoSafetyMessage;
class DemoServiceAdvertisment;

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
    cMessage* statusUpdateMsg;
    cMessage* requestRoadInfoMsg;
    cMessage* cleanupTimer;

    int myInternalId;       // the internal ID used by Veins/OMNeT++
    int mySimulationId;     // the simulation ID visible in the TreeView
    bool hasLoggedDeparture = false;
    double startTime;
    double endTime;
    double earliestArrival;
    double latestArrival;
    string startingRoad;
    string targetRoad;
    double pathLength;
    
    string currentRoadId;
    vector<string> allRoads;
    vector<string> accessibleRoads;
    vector<string> incomingRoads;
    map<string, string> currentRoadAttributes;
    vector<string> currentPath;
    vector<Destination> destinations;

    unique_ptr<GraphProcessor> graphProcessor;
    Graph roadNetwork;

    // RSU communication methods
    void sendStatusUpdate();
    void requestAllRoads();
    void requestAccessibleRoads(const string& roadId);
    void requestIncomingRoads(const string& roadId);
    void requestRoadAttributes(const string& roadId);
    void requestShortestPath(const string& sourceId, const string& targetId);
    void requestKPaths(const string& sourceId, const string& targetId, int k);
    void requestDestinations(int count);
    void requestValidAssignment(const vector<string>& sources, const vector<string>& destinations);

    // Response processing methods
    void processAllRoadsResponse(const string& data);
    void processAccessibleRoadsResponse(const string& data);
    void processIncomingRoadsResponse(const string& data);
    void processRoadAttributesResponse(const string& data);
    void processShortestPathResponse(const string& data);
    void processKPathsResponse(const string& data);
    void processDestinationsResponse(const string& data);
    void processValidAssignmentResponse(const string& data);

    // Helper methods
    vector<string> parseRoadList(const string& data, char delimiter = ',');
    map<string, string> parseAttributes(const string& data);
    void printRoadInfo();
    void buildLocalRoadNetwork();
    
    vector<string> findShortestPath(const string& sourceId, const string& targetId);
    double getShortestPathLength(const string& sourceId, const string& targetId);

    void cleanupMessages();
    void logDepartureIfNeeded();
    void checkVehicleStatus();
    void updateTimingFile(double endTime);
    
    void checkAndAdjustSpeed();
    
    double measureRoutingAlgorithmTime(const std::string& sourceId, const std::string& targetId);
};

} // namespace veins
