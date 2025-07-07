#include "RSUControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include<bits/stdc++.h>
#include <ctime>
#include <iomanip>
#include <chrono>
#ifdef _WIN32
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

using namespace veins;
using namespace std;

Define_Module(RSUControlApp);

void RSUControlApp::initialize(int stage) {
    TraCIDemoRSU11p::initialize(stage);
    if (stage == 0) {
        vehicleDataMap.clear();
        simulationIdToAddressMap.clear();
        numVehicles = 0;
        
        mapName = "erlangen";
        routingAlgorithm = "ShortestPath";
        implementationVersion = "2.0";
        SimulationLogger::getInstance().setSimulationInfo(mapName, routingAlgorithm, implementationVersion);
        
        statusCheckMsg = new cMessage("checkStatus");
        rerouteMsg = new cMessage("rerouteVehicles");
        scheduleAt(simTime() + 0.9, statusCheckMsg);
        scheduleAt(simTime() + 1.5, rerouteMsg);
        networkFilePath = par("netFile").stdstringValue();
        
        if (!networkFilePath.empty()) {
            xmlProcessor.reset(new XMLProcessor());

            if (xmlProcessor->loadNetworkFile(networkFilePath)) {
                graphProcessor.reset(new GraphProcessor(xmlProcessor->getGraph()));
                taskGenerator.reset(new TaskGenerator(*graphProcessor));
                
                if (xmlProcessor->loadRouteFile("./erlangen.rou.xml")) {
                    vector<Vehicle> vehicles = xmlProcessor->getVehicles();
                    numVehicles = vehicles.size();
                    printVehicleRouteInfo(vehicles);
                }
            }
        }
        
        for (int internalId = 0; internalId < numVehicles; internalId++) {
            int simulationId = 16 + 6 * internalId;
            vehicleDataMap[internalId].simulationId = simulationId;
        }
    }
}

void RSUControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* enc = wsm->getEncapsulatedPacket();
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(enc);
    if (!msg) return;
    
    LAddress::L2Type senderId = msg->getSenderAddress();
    string data = msg->getDemoData();
    vehicleDataMap[senderId].updateMessageTime(simTime().dbl());
    
    if (data.find("STATUS:") == 0) {
        size_t idPos = data.find("simId=");
        if (idPos != string::npos) {
            size_t idEnd = data.find(';', idPos);
            if (idEnd != string::npos) {
                string idStr = data.substr(idPos + 6, idEnd - idPos - 6);
                int simId = stoi(idStr);
                updateVehicleIdMapping(senderId, simId);
            }
        } else {
            size_t idPos = data.find("id=");
            if (idPos != string::npos) {
                size_t idEnd = data.find(';', idPos);
                if (idEnd != string::npos) {
                    string idStr = data.substr(idPos + 3, idEnd - idPos - 3);
                        int simId = stoi(idStr);
                        updateVehicleIdMapping(senderId, simId);
                }
            }
        }
    }
    
    handleVehicleMessage(data, senderId);
}

void RSUControlApp::updateVehicleIdMapping(LAddress::L2Type vehicleAddress, int simulationId) {
    if (simulationId > 0) {
        vehicleDataMap[vehicleAddress].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleAddress;
        return;
    }
    
    if (vehicleDataMap[vehicleAddress].simulationId <= 0) {
        int nextId = vehicleDataMap.size();
        while (vehicleDataMap.find(nextId) != vehicleDataMap.end()) {
            nextId++;
        }
        int newSimId = 16 + 6 * nextId;
        vehicleDataMap[vehicleAddress].simulationId = newSimId;
        simulationIdToAddressMap[newSimId] = vehicleAddress;
    }
}

void RSUControlApp::handleVehicleMessage(const string& message, LAddress::L2Type vehicleId) {
    if (message == "GET_ALL_ROADS") {
        auto roads = xmlProcessor->getAllRoads();
        sendRoadListMessage(vehicleId, roads);
    }
    else if (message.find("GET_ACCESSIBLE_ROADS:") == 0) {
        string roadId = message.substr(21);
        auto accessibleRoads = xmlProcessor->getAccessibleRoads(roadId);
        sendRoadListMessage(vehicleId, accessibleRoads);
    }
    else if (message.find("GET_INCOMING_ROADS:") == 0) {
        string roadId = message.substr(19);
        auto incomingRoads = xmlProcessor->getIncomingRoads(roadId);
        sendRoadListMessage(vehicleId, incomingRoads);
    }
    else if (message.find("GET_ROAD_ATTRIBUTES:") == 0) {
        string roadId = message.substr(20);
        auto attrs = xmlProcessor->getRoadAttributes(roadId);

        const Graph& graph = xmlProcessor->getGraph();
        bool edgeFound = false;

        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == roadId) {
                    edgeFound = true;
                    const auto& lanes = edge.getLanes();
                    attrs["laneCount"] = to_string(lanes.size());

                    for (size_t i = 0; i < lanes.size(); ++i) {
                        const auto& lane = lanes[i];
                        string lanePrefix = "lane" + to_string(i) + "_";
                        attrs[lanePrefix + "id"] = lane.id;
                        attrs[lanePrefix + "index"] = to_string(lane.index);
                        attrs[lanePrefix + "speed"] = to_string(lane.speed);
                        attrs[lanePrefix + "length"] = to_string(lane.length);
                    }
                    break;
                }
            }
            if (edgeFound) break;
        }

        string attrStr = roadId + ":";
        for (const auto& attr : attrs) {
            attrStr += attr.first + "=" + attr.second + ";";
        }

        auto* response = new TraCIDemo11pMessage();
        response->setDemoData(("ROAD_ATTRIBUTES:" + attrStr).c_str());
        response->setSenderAddress(myId);
        
        auto* wsm = new BaseFrame1609_4();
        wsm->encapsulate(response);
        populateWSM(wsm);
        wsm->setRecipientAddress(vehicleId);
        sendDown(wsm);
    }
    else if (message.find("FIND_SHORTEST_PATH:") == 0 && graphProcessor) {
        string params = message.substr(19);
        size_t commaPos = params.find(',');
        if (commaPos != string::npos) {
            string sourceId = params.substr(0, commaPos);
            string targetId = params.substr(commaPos + 1);
            
            auto path = graphProcessor->findShortestPath(sourceId, targetId);
            double pathLength = graphProcessor->getShortestPathLength(sourceId, targetId);
            
            vector<string> response;
            if (path.empty()) {
                response.push_back("NO_PATH_EXISTS");
            } else {
                response.push_back("LENGTH:" + to_string(pathLength));
                for (const auto& roadId : path) {
                    response.push_back(roadId);
                }
            }
            sendRoadListMessage(vehicleId, response);
        }
    }
    else if (message.find("EXISTS_VALID_ASSIGNMENT:") == 0 && taskGenerator) {
        string params = message.substr(24);
        size_t separatorPos = params.find('|');
        
        if (separatorPos != string::npos) {
            string sourcesStr = params.substr(0, separatorPos);
            string destsStr = params.substr(separatorPos + 1);
            
            vector<string> sources;
            vector<string> dests;
            
            size_t pos = 0;
            while ((pos = sourcesStr.find(',')) != string::npos) {
                sources.push_back(sourcesStr.substr(0, pos));
                sourcesStr.erase(0, pos + 1);
            }
            if (!sourcesStr.empty()) {
                sources.push_back(sourcesStr);
            }
            
            pos = 0;
            while ((pos = destsStr.find(',')) != string::npos) {
                dests.push_back(destsStr.substr(0, pos));
                destsStr.erase(0, pos + 1);
            }
            if (!destsStr.empty()) {
                dests.push_back(destsStr);
            }
            
            bool validAssignment = taskGenerator->existsValidAssignment(sources, dests);
            sendRoadListMessage(vehicleId, {validAssignment ? "TRUE" : "FALSE"});
        }
    }
}

void RSUControlApp::sendRoadListMessage(LAddress::L2Type vehicleId, const vector<string>& roadList) {
    ostringstream oss;
    oss << "ROAD_LIST:";

    for (size_t i = 0; i < roadList.size(); ++i) {
        oss << roadList[i];
        if (i < roadList.size() - 1) {
            oss << ",";
        }
    }

    auto* response = new TraCIDemo11pMessage();
    response->setDemoData(oss.str().c_str());
    response->setSenderAddress(myId);

    auto* wsm = new BaseFrame1609_4();
    wsm->encapsulate(response);
    populateWSM(wsm);
    wsm->setRecipientAddress(vehicleId);
    sendDown(wsm);
}

void RSUControlApp::handleSelfMsg(cMessage* msg) {
    if (msg == statusCheckMsg) {
        cleanupVehicleData();
        scheduleAt(simTime() + 2.0, statusCheckMsg);
    }
    else if (msg == rerouteMsg) {
        sendRerouteToAllVehicles();
    }
    else {
        TraCIDemoRSU11p::handleSelfMsg(msg);
    }
}

void RSUControlApp::sendRerouteToAllVehicles() {
    int vehiclesRerouted = 0;

    for (auto& vehiclePair : vehicleDataMap) {
        LAddress::L2Type vehicleId = vehiclePair.first;
        Vehicle& vehicle = vehiclePair.second;
        
        vector<string> edgePath;
        
        if (!vehicle.plannedPath.empty()) {
            edgePath = vehicle.plannedPath;
        }
        
        if (edgePath.size() > 1) {
            vector<string> simplifiedPath;
            simplifiedPath.push_back(edgePath.front());
            simplifiedPath.push_back(edgePath.back());

            if (!edgePath.empty()) {
                sendRerouteMessage(vehicleId, edgePath);
                vehiclesRerouted++;
                vehicle.lastSentPath = edgePath;
            }
        }
        else if (!edgePath.empty()) {
            sendRerouteMessage(vehicleId, edgePath);
            vehiclesRerouted++;
            vehicle.lastSentPath = edgePath;
        }
    }
}

void RSUControlApp::sendRerouteMessage(LAddress::L2Type vehicleId, const vector<string>& edgePath) {
    int simId = vehicleDataMap[vehicleId].simulationId;
    if (simId == -1) {
        simId = vehicleId;
    }
    
    ostringstream routeStr;
    routeStr << "CHANGE_ROUTE:" << simId << ":";
    if (!edgePath.empty()) {
        for (size_t i = 0; i < edgePath.size(); i++) {
            routeStr << edgePath[i];
            if (i < edgePath.size() - 1) {
                routeStr << " ";
            }
        }
        
        string message = routeStr.str();

        auto* response = new TraCIDemo11pMessage();
        response->setDemoData(message.c_str());
        response->setSenderAddress(myId);

        auto* wsm = new BaseFrame1609_4();
        wsm->encapsulate(response);
        populateWSM(wsm);
        sendDown(wsm);
    }
}

void RSUControlApp::finish() {
    SimulationLogger::getInstance().saveToCSV("simulation_results_3.csv");
    SimulationLogger::getInstance().printSummary();
    cancelAndDelete(statusCheckMsg);
    cancelAndDelete(rerouteMsg);
    TraCIDemoRSU11p::finish();
}

void RSUControlApp::cleanupVehicleData() {
    simtime_t now = simTime();
    const double CLEANUP_TIMEOUT = 10.0;
    
    for (auto it = vehicleDataMap.begin(); it != vehicleDataMap.end();) {
        if (now - it->second.lastMessageTime > CLEANUP_TIMEOUT) {
            it = vehicleDataMap.erase(it);
        } else {
            ++it;
        }
    }
}

void RSUControlApp::onWSA(DemoServiceAdvertisment* wsa) {
    TraCIDemoRSU11p::onWSA(wsa);
}

vector<string> RSUControlApp::getAllRoads() const {
    return xmlProcessor->getAllRoads();
}

vector<string> RSUControlApp::getAllNodes() const {
    vector<string> result;
    const Graph& graph = xmlProcessor->getGraph();
    const auto& nodes = graph.getNodes();

    for (const auto& nodePair : nodes) {
        result.push_back(nodePair.first);
    }
    return result;
}

vector<string> RSUControlApp::findShortestPath(const string& sourceId, const string& targetId) const {
    return graphProcessor->findShortestPath(sourceId, targetId);
}

double RSUControlApp::getShortestPathLength(const string& sourceId, const string& targetId) const {
    return graphProcessor->getShortestPathLength(sourceId, targetId);
}

void RSUControlApp::findEdgePathAndPrint(string sourceEdgeId, string targetEdgeId) const {
    if (!graphProcessor) {
        return;
    }
    
    const Graph& graph = graphProcessor->getGraph();
    bool sourceEdgeFound = false;
    bool targetEdgeFound = false;
    string sourceNodeId, targetNodeId;

    map<string, pair<string, string>> edgeToJunctions;
    
    for (const auto& nodePair : graph.getAdjList()) {
        const string& fromJunction = nodePair.first;
        for (const auto& edge : nodePair.second) {
            const string& edgeId = edge.getId();
            const string& toJunction = edge.getTo();
            edgeToJunctions[edgeId] = make_pair(fromJunction, toJunction);
            
            if (edge.getId() == sourceEdgeId) {
                sourceEdgeFound = true;
                sourceNodeId = nodePair.first;
            }
            if (edge.getId() == targetEdgeId) {
                targetEdgeFound = true;
                targetNodeId = nodePair.first;
            }
        }
    }
    
    if (!sourceEdgeFound || !targetEdgeFound) {
        return;
    }

    if (!sourceNodeId.empty() && !targetNodeId.empty()) {
        vector<string> nodePath = graphProcessor->findShortestPath(sourceNodeId, targetNodeId);
    }
    
    auto edgePath = graphProcessor->findEdgeShortestPath(sourceEdgeId, targetEdgeId);
    double totalLength = 0.0;
    for (const auto& edgeId : edgePath) {
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    totalLength += edge.getLength();
                    break;
                }
            }
        }
    }
}

void RSUControlApp::printVehicleRouteInfo(const vector<Vehicle>& vehicles) {
    generateAndAssignDestinations(vehicles);
}

int extractVehicleIndex(const string& vehicleId) {
    if (vehicleId.length() > 2 && vehicleId.substr(0, 2) == "t_") {
      return stoi(vehicleId.substr(2));
    }
    return stoi(vehicleId);
}

void RSUControlApp::generateAndAssignDestinations(const vector<Vehicle>& vehicles) {
    if (!taskGenerator || !graphProcessor || vehicles.empty()) {
        return;
    }

    cout << "\n=== Vehicle Index Mapping ===" << endl;
    vector<string> sourceRoads;
    for (const auto& vehicle : vehicles) {
        sourceRoads.push_back(vehicle.startingRoad);
        double startTime = vehicle.departTime;
        recordVehicleStart(vehicle.index, vehicle.startingRoad, startTime);
        cout << "Vehicle " << vehicle.index << ": Road " << vehicle.startingRoad << endl;
    }

    const Graph& graph = graphProcessor->getGraph();
    unsigned seedValue = static_cast<unsigned>(simTime().raw() + time(nullptr)) % UINT_MAX;
    auto destObjects = taskGenerator->generateDestinationsWithTimeWindows(
        vehicles.size(), sourceRoads, graph, seedValue);
    
    vector<string> destEdges;
    for (const auto& dest : destObjects) {
        destEdges.push_back(dest.nodeId);
    }

    int numVehicles = sourceRoads.size();
    int numDestinations = destObjects.size();
    int n = max(numVehicles, numDestinations);
    vector<vector<double>> deviationMatrix(n, vector<double>(n, 0));
    const double NO_PATH_PENALTY = 9999999.0;
    
    map<pair<int, int>, pair<vector<string>, double>> pathsAndTimes;
    
    cout << "\n=== Building Hungarian Matrix ===" << endl;
    auto startFullAlgorithm = chrono::high_resolution_clock::now();
    
    for (int i = 0; i < numVehicles; ++i) {
        const string& sourceEdgeId = sourceRoads[i];
        
        for (int j = 0; j < numDestinations; ++j) {
            const string& destEdgeId = destEdges[j];
            auto path = graphProcessor->findEdgeShortestPath(sourceEdgeId, destEdgeId);

            if (!path.empty()) {
                double optimalTime = 0.0;
                
                for (const auto& edgeId : path) {
                    bool edgeFound = false;
                    for (const auto& nodePair : graph.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                double edgeLength = edge.getLength();
                                double edgeSpeed = edge.getMaxSpeed();
                                optimalTime += edgeLength / edgeSpeed;
                                edgeFound = true;
                                break;
                            }
                        }
                        if (edgeFound) break;
                    }
                }
                
                double expectedTime = 1.2 * optimalTime;
                const TimeWindow& tw = destObjects[j].timeWindow;
                double deviation = 0.0;
                
                if (expectedTime < tw.earliness) {
                    deviation = tw.earliness - expectedTime;
                } else if (expectedTime > tw.tardiness) {
                    deviation = expectedTime - tw.tardiness;
                } else {
                    deviation = 0.0;
                }
                
                deviationMatrix[i][j] = deviation;
                pathsAndTimes[{i, j}] = {path, optimalTime};
                
            } else {
                deviationMatrix[i][j] = NO_PATH_PENALTY;
            }
        }
    }
    
    for (int i = 0; i < n; ++i) {
        for (int j = numDestinations; j < n; ++j) {
            deviationMatrix[i][j] = NO_PATH_PENALTY;
        }
    }
    for (int i = numVehicles; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            deviationMatrix[i][j] = NO_PATH_PENALTY;
        }
    }
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numDestinations; ++j) {
            cout << fixed << setprecision(2) << deviationMatrix[i][j] << "\t";
        }
        cout << endl;
    }
    
    vector<int> assignment = graphProcessor->getOptimalAssignmentWithMatrix(deviationMatrix);
    
    auto endFullAlgorithm = chrono::high_resolution_clock::now();
    chrono::duration<double> totalAlgorithmDuration = endFullAlgorithm - startFullAlgorithm;
    double totalAlgorithmTime = totalAlgorithmDuration.count();
    
    cout << "\n=== Algorithm Results ===" << endl;
    cout << "Total Algorithm Time: " << fixed << setprecision(6) 
         << totalAlgorithmTime << " seconds" << endl;
    
    SimulationLogger::getInstance().recordTotalAlgorithmTime(totalAlgorithmTime);
    
    cout << "\n=== Vehicle Assignment Distribution ===" << endl;
    double totalDeviation = 0.0;
    int onTimeCount = 0;
    
    for (int i = 0; i < assignment.size() && i < numVehicles; i++) {
        int destIndex = assignment[i];
        if (destIndex != -1 && destIndex < destObjects.size()) {
            double deviation = deviationMatrix[i][destIndex];
            totalDeviation += (deviation < NO_PATH_PENALTY ? deviation : 0);
            if (deviation == 0) onTimeCount++;
            
            auto pathTimeKey = make_pair(i, destIndex);
            double optimalTime = pathsAndTimes[pathTimeKey].second;
            double expectedTime = 1.2 * optimalTime;
            const TimeWindow& tw = destObjects[destIndex].timeWindow;
            
            string arrivalStatus;
            if (deviation == 0) {
                arrivalStatus = "ON_TIME";
            } else if (expectedTime < tw.earliness) {
                arrivalStatus = "EARLY";
            } else {
                arrivalStatus = "LATE";
            }
                
            cout << "Vehicle " << i << " -> Destination " << destObjects[destIndex].nodeId
               << " | Status: " << arrivalStatus << endl;
        }
    }
    
    cout << "\n=== Summary ===" << endl;
    cout << "Total vehicles: " << numVehicles << endl;
    cout << "On-time arrivals: " << onTimeCount << endl;
    cout << "Total deviation: " << totalDeviation << " seconds" << endl;

    for (size_t i = 0; i < assignment.size() && i < vehicles.size(); ++i) {
        int destIndex = assignment[i];
        if (destIndex == -1 || destIndex >= destObjects.size()) {
            continue;
        }
        
        const string& sourceRoad = sourceRoads[i];
        const string& targetRoad = destObjects[destIndex].nodeId;
        
        auto pathTimeKey = make_pair(i, destIndex);
        auto path = pathsAndTimes[pathTimeKey].first;
        double optimalTime = pathsAndTimes[pathTimeKey].second;
        
        double totalDistance = 0.0;
        for (const auto& edgeId : path) {
            for (const auto& nodePair : graph.getAdjList()) {
                for (const auto& edge : nodePair.second) {
                    if (edge.getId() == edgeId) {
                        totalDistance += edge.getLength();
                        break;
                    }
                }
            }
        }
        recordVehicleDestination(vehicles[i].index, targetRoad,
            destObjects[destIndex].timeWindow.earliness,
            destObjects[destIndex].timeWindow.tardiness,
            path, totalDistance);
        int vehicleIndex = vehicles[i].index;
        vehicleDataMap[vehicleIndex].assignedDestination = destObjects[destIndex];
        vehicleDataMap[vehicleIndex].plannedPath = path;
        vehicleDataMap[vehicleIndex].earliestArrival = destObjects[destIndex].timeWindow.earliness;
        vehicleDataMap[vehicleIndex].latestArrival = destObjects[destIndex].timeWindow.tardiness;
        vehicleDataMap[vehicleIndex].pathLength = totalDistance;
        vehicleDataMap[vehicleIndex].estimatedTravelTime = optimalTime;
        vehicleDataMap[vehicleIndex].startTime = vehicles[i].departTime;
            
        if (vehicleDataMap[vehicleIndex].simulationId == -1) {
            int simulationId = 16 + 6 * vehicleIndex;
            vehicleDataMap[vehicleIndex].simulationId = simulationId;
            simulationIdToAddressMap[simulationId] = vehicleIndex;
        }
    }
}

double RSUControlApp::getEdgeLength(const string& edgeId) const {
    if (graphProcessor) {
        const Graph& graph = graphProcessor->getGraph();
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    return edge.getLength();
                }
            }
        }
    }
    return 0;
}

void RSUControlApp::recordVehicleStart(int vehicleId, const string& startRoad, double startTime) {
    int simulationId = -1;
    
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    } else {
        simulationId = 16 + 6 * vehicleId;
        vehicleDataMap[vehicleId].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleId;
    }
    
    SimulationLogger::getInstance().recordVehicleStart(simulationId, startRoad, startTime);
}

void RSUControlApp::recordVehicleDestination(int vehicleId, const string& targetRoad, 
                                            double earliestArrival, double latestArrival, 
                                            const vector<string>& path, double pathLength) {
    int simulationId = -1;
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    } else {
        simulationId = 16 + 6 * vehicleId;
        vehicleDataMap[vehicleId].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleId;
    }
    
    SimulationLogger::getInstance().updateVehicleDestination(
        simulationId, targetRoad, earliestArrival, latestArrival, path, pathLength);
}

void RSUControlApp::processVehicleDepartureNotification(const string& data) {
    istringstream iss(data);
    string action;
    int vehicleId;
    iss >> action >> vehicleId;
}

void RSUControlApp::processVehicleArrivalNotification(const string& data) {
    istringstream iss(data);
    string action;
    int vehicleId;
    double arrivalTime;
    iss >> action >> vehicleId >> arrivalTime;
    
    if (action == "ARRIVAL") {
        SimulationLogger::getInstance().recordVehicleEnd(vehicleId, arrivalTime);
    }
}
