#include "RSUControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include<bits/stdc++.h>
#include <ctime>    // Add include for std::time
#include <iomanip>  // Add include for std::setprecision
#include <chrono>   // Add include for high resolution timer
// Platform-specific includes for getting working directory
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
    EV << "[RSU] Initialize called with stage " << stage << endl;
    if (stage == 0) {
        vehicleDataMap.clear();
        simulationIdToAddressMap.clear();
        
        // Will be updated when route file is loaded
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
        EV << "[RSU] Network file path parameter: " << networkFilePath << endl;
        
        if (!networkFilePath.empty()) {
            xmlProcessor.reset(new XMLProcessor());

            EV << "[RSU] Attempting to load network file from: " << networkFilePath << std::endl;

            if (xmlProcessor->loadNetworkFile(networkFilePath)) {
                EV << "[RSU] Successfully loaded road network from " << networkFilePath << std::endl;
                graphProcessor.reset(new GraphProcessor(xmlProcessor->getGraph()));
                EV << "[RSU] Successfully initialized GraphProcessor" << std::endl;
                taskGenerator.reset(new TaskGenerator(*graphProcessor));
                EV << "[RSU] Successfully initialized TaskGenerator" << std::endl;
                EV << "[RSU] Attempting to load route information" << std::endl;
                if (xmlProcessor->loadRouteFile("./erlangen.rou.xml")) {
                    vector<Vehicle> vehicles = xmlProcessor->getVehicles();
                    
                    // For informational purposes only
                    numVehicles = vehicles.size();
                    EV << "[RSU] Found " << numVehicles << " vehicles in route file" << std::endl;
                    
                    printVehicleRouteInfo(vehicles);
                } else {
                    EV << "[RSU] Failed to load route file" << endl;
                }
            }
        }
        
        // Now create the vehicle ID mapping based on actual vehicle count
        EV << "[RSU] Pre-mapping " << numVehicles << " vehicles" << std::endl;
        
        // Create mapping using formula: simulation_id = 16 + 6 * index
        for (int internalId = 0; internalId < numVehicles; internalId++) {
            int simulationId = 16 + 6 * internalId;
            vehicleDataMap[internalId].simulationId = simulationId;
            EV << "[RSU] Pre-mapped internal vehicle " << internalId
               << " to simulation ID " << simulationId << std::endl;
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
                try {
                    int simId = std::stoi(idStr);
                    updateVehicleIdMapping(senderId, simId);
                    EV << "[RSU] Updated vehicle ID mapping from STATUS message: Address " << senderId 
                       << " -> Simulation ID " << simId << std::endl;
                } catch (const exception& e) {
                    EV << "[RSU] Failed to parse simulation ID: " << e.what() << std::endl;
                }
            }
        }
        // Backward compatibility - check for old format with id=
        else {
            size_t idPos = data.find("id=");
            if (idPos != std::string::npos) {
                size_t idEnd = data.find(';', idPos);
                if (idEnd != std::string::npos) {
                    std::string idStr = data.substr(idPos + 3, idEnd - idPos - 3); // +3 for "id="
                    try {
                        int simId = std::stoi(idStr);
                        updateVehicleIdMapping(senderId, simId);
                        
                        // Debug output
                        EV << "[RSU] Updated vehicle ID mapping from old STATUS format: Address " << senderId 
                           << " -> Simulation ID " << simId << std::endl;
                        std::cout << "RSU: Updated vehicle ID mapping from old STATUS format: Address " << senderId
                                  << " -> Simulation ID " << simId << std::endl;
                    } catch (const std::exception& e) {
                        EV << "[RSU] Failed to parse simulation ID: " << e.what() << std::endl;
                    }
                }
            }
        }
    }
    
    handleVehicleMessage(data, senderId);
}

void RSUControlApp::updateVehicleIdMapping(LAddress::L2Type vehicleAddress, int simulationId) {
    // If simulationId is provided, use it directly
    if (simulationId > 0) {
        vehicleDataMap[vehicleAddress].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleAddress;
        EV << "[RSU] Updated mapping: Vehicle address " << vehicleAddress << " -> Simulation ID " << simulationId << std::endl;
        return;
    }
    
    // If simulationId is not provided or invalid, we need to generate a new one based on next available index
    if (vehicleDataMap[vehicleAddress].simulationId <= 0) {
        // Find next available internal ID for this vehicle
        int nextId = vehicleDataMap.size();
        
        // Check if we already have an entry with this ID (should not happen, but just in case)
        while (vehicleDataMap.find(nextId) != vehicleDataMap.end()) {
            nextId++;
        }
        
        // Apply our formula: simulation_id = 16 + 6 * index
        int newSimId = 16 + 6 * nextId;
        
        // Store the mapping
        vehicleDataMap[vehicleAddress].simulationId = newSimId;
        simulationIdToAddressMap[newSimId] = vehicleAddress;
        
        EV << "[RSU] Created new mapping: Vehicle address " << vehicleAddress 
           << " -> Simulation ID " << newSimId << " (internal index " << nextId << ")" << std::endl;
    }
}

void RSUControlApp::handleVehicleMessage(const string& message, LAddress::L2Type vehicleId) {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] XML processor not initialized or network not loaded" << std::endl;
        vector<string> errorMsg = {"ERROR: Network not loaded"};
        sendRoadListMessage(vehicleId, errorMsg);
        return;
    }
    EV << "[RSU] Received message from vehicle: '" << message << "'" << endl;

    if (message == "GET_ALL_ROADS") {
        auto roads = xmlProcessor->getAllRoads();
        EV << "[RSU] Responding with " << roads.size() << " roads for GET_ALL_ROADS request" << endl;
        sendRoadListMessage(vehicleId, roads);
    }
    else if (message.find("GET_ACCESSIBLE_ROADS:") == 0) {
        string roadId = message.substr(21);
        auto accessibleRoads = xmlProcessor->getAccessibleRoads(roadId);
        EV << "[RSU] Found " << accessibleRoads.size() << " accessible roads from '" << roadId << "': ";
        for (int i = 0; i < accessibleRoads.size(); ++i) {
            EV << accessibleRoads[i];
            if (i < accessibleRoads.size() - 1) {
                EV << ", ";
            }
        }
        EV << endl;
        EV << "[RSU] Source road '" << roadId << "' details:" << std::endl;
        const Graph& graph = xmlProcessor->getGraph();
        bool sourceRoadFound = false;
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == roadId) {
                    sourceRoadFound = true;

                    // Print source road info
                    EV << "[RSU] <edge id=\"" << edge.getId() 
                       << "\" from=\"" << edge.getFrom() 
                       << "\" to=\"" << edge.getTo() 
                       << "\" length=\"" << edge.getLength() << "\">" << std::endl;

                    // Get the lanes for this edge
                    const auto& lanes = edge.getLanes();
                    EV << "[RSU]   Source road has " << lanes.size() << " lanes" << std::endl;
                    
                    for (const auto& lane : lanes) {
                        EV << "[RSU]   <lane id=\"" << lane.id 
                           << "\" index=\"" << lane.index 
                           << "\" speed=\"" << lane.speed 
                           << "\" length=\"" << lane.length 
                           << "\" shape=\"" << lane.shape << "\"/>" << std::endl;
                    }
                    
                    EV << "[RSU] </edge>" << std::endl;
                    break;
                }
            }
            if (sourceRoadFound) break;
        }

        if (!sourceRoadFound) {
            EV << "[RSU] Source road '" << roadId << "' not found in graph" << std::endl;
        }

        sendRoadListMessage(vehicleId, accessibleRoads);
    }
    // Handle request for incoming roads
    else if (message.find("GET_INCOMING_ROADS:") == 0) {
        std::string roadId = message.substr(19);
        EV << "[RSU] Looking for incoming roads to '" << roadId << "'" << std::endl;
        auto incomingRoads = xmlProcessor->getIncomingRoads(roadId);
        EV << "[RSU] Found " << incomingRoads.size() << " incoming roads to '" << roadId << "'" << std::endl;
        sendRoadListMessage(vehicleId, incomingRoads);
    }
    // Handle request for road attributes
    else if (message.find("GET_ROAD_ATTRIBUTES:") == 0) {
        std::string roadId = message.substr(20);

        auto attrs = xmlProcessor->getRoadAttributes(roadId);

        // Find the edge object to get lane information
        const Graph& graph = xmlProcessor->getGraph();
        bool edgeFound = false;

        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == roadId) {
                    edgeFound = true;

                    // Add lane information to attributes
                    const auto& lanes = edge.getLanes();
                    attrs["laneCount"] = std::to_string(lanes.size());

                    // Add detailed lane information
                    for (size_t i = 0; i < lanes.size(); ++i) {
                        const auto& lane = lanes[i];
                        std::string lanePrefix = "lane" + std::to_string(i) + "_";
                        attrs[lanePrefix + "id"] = lane.id;
                        attrs[lanePrefix + "index"] = std::to_string(lane.index);
                        attrs[lanePrefix + "speed"] = std::to_string(lane.speed);
                        attrs[lanePrefix + "length"] = std::to_string(lane.length);
                    }

                    break;
                }
            }
            if (edgeFound) break;
        }

        std::string attrStr = roadId + ":";
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

        EV << "[RSU] Sent attributes for road " << roadId << ": " << attrs.size() << " attributes" << std::endl;
    }
    // Handle shortest path request
    else if (message.find("FIND_SHORTEST_PATH:") == 0 && graphProcessor) {
        // Extract source and target IDs
        std::string params = message.substr(19);
        size_t commaPos = params.find(',');
        if (commaPos != std::string::npos) {
            std::string sourceId = params.substr(0, commaPos);
            std::string targetId = params.substr(commaPos + 1);
            
            // Find the shortest path using GraphProcessor
            auto path = graphProcessor->findShortestPath(sourceId, targetId);
            
            // Get the path length 
            double pathLength = graphProcessor->getShortestPathLength(sourceId, targetId);
            
            // Create the response message with path and length
            std::vector<std::string> response;
            
            if (path.empty()) {
                response.push_back("NO_PATH_EXISTS");
            } else {
                response.push_back("LENGTH:" + std::to_string(pathLength));
                for (const auto& roadId : path) {
                    response.push_back(roadId);
                }
            }
            
            sendRoadListMessage(vehicleId, response);
            EV << "[RSU] Path from " << sourceId << " to " << targetId << ": ";
            if (path.empty()) {
                EV << "No path exists" << std::endl;
            } else {
                EV << pathLength << " units, " << path.size() << " segments" << std::endl;
                for (const auto& roadId : path) {
                    EV << "  - " << roadId << std::endl;
                }
            }
        }
    }
    else if (message.find("EXISTS_VALID_ASSIGNMENT:") == 0 && taskGenerator) {
        EV << "[RSU] DEBUG: Received EXISTS_VALID_ASSIGNMENT request: " << message << std::endl;
        
        string params = message.substr(24);
        size_t separatorPos = params.find('|');
        
        if (separatorPos != std::string::npos) {
            string sourcesStr = params.substr(0, separatorPos);
            std::string destsStr = params.substr(separatorPos + 1);
            
            // Parse sources and destinations
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
            while ((pos = destsStr.find(',')) != std::string::npos) {
                dests.push_back(destsStr.substr(0, pos));
                destsStr.erase(0, pos + 1);
            }
            if (!destsStr.empty()) {
                dests.push_back(destsStr);
            }
            EV << "[RSU] DEBUG: Sources (" << sources.size() << "): ";
            for (size_t i = 0; i < sources.size(); i++) {
                EV << sources[i];
                if (i < sources.size() - 1) EV << ", ";
            }
            EV << std::endl;
            
            EV << "[RSU] DEBUG: Destinations (" << dests.size() << "): ";
            for (size_t i = 0; i < dests.size(); i++) {
                EV << dests[i];
                if (i < dests.size() - 1) EV << ", ";
            }
            EV << endl;
            
            bool validAssignment = taskGenerator->existsValidAssignment(sources, dests);
            EV << "[RSU] Valid assignment: " << (validAssignment ? "TRUE" : "FALSE") << endl;
            
            sendRoadListMessage(vehicleId, {validAssignment ? "TRUE" : "FALSE"});
        } else {
            EV << "[RSU] ERROR: Malformed valid assignment request" << endl;
        }
    }
}

void RSUControlApp::sendRoadListMessage(LAddress::L2Type vehicleId, const vector<string>& roadList) {
    std::ostringstream oss;
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
        if (!networkFilePath.empty()) {
            EV << "Current network file: " << networkFilePath << std::endl;
            if (xmlProcessor && xmlProcessor->isNetworkLoaded()) {
                EV << "Network successfully loaded with "
                         << xmlProcessor->getGraph().getNodeCount() << " nodes and "
                         << xmlProcessor->getGraph().getEdgeCount() << " edges" << std::endl;
                auto roads = xmlProcessor->getAllRoads();
                EV << "First 10 roads: ";
                for (size_t i = 0; i < std::min(roads.size(), size_t(10)); i++) {
                    EV << roads[i] << " ";
                }
                EV << std::endl;
            } else {
                EV << "Network not loaded correctly!" << std::endl;
            }
        }
        scheduleAt(simTime() + 2.0, statusCheckMsg);
    }
    else if (msg == rerouteMsg) {
        EV << "\n[RSU] ******************************************" << std::endl;
        EV << "[RSU] REROUTING TRIGGERED AT t=" << simTime() << std::endl;
        EV << "[RSU] ******************************************" << std::endl;
        
        // Print vehicle data map content
        EV << "[RSU] Vehicle data map contains " << vehicleDataMap.size() << " vehicles:" << std::endl;
        for (const auto& pair : vehicleDataMap) {
            EV << "  Vehicle ID: " << pair.first 
               << ", last message time: " << pair.second.lastMessageTime
               << ", has destination: " << (!pair.second.assignedDestination.nodeId.empty() ? "yes" : "no")
               << ", path size: " << pair.second.plannedPath.size() << std::endl;
        }
        
        sendRerouteToAllVehicles();
    }
    else {
        TraCIDemoRSU11p::handleSelfMsg(msg);
    }
}

void RSUControlApp::sendRerouteToAllVehicles() {
    EV << "\n[RSU] ===== SENDING REROUTE MESSAGES TO ALL VEHICLES =====\n" << std::endl;
    std::cout << "RSU is sending reroute messages to all vehicles at time " << simTime() << std::endl;

    // We'll skip the direct TraCI access and just use the vehicle data map
    std::cout << "Rerouting based on stored vehicle data" << std::endl;

    int vehiclesRerouted = 0;


    // Process each vehicle in our data map
    for (auto& vehiclePair : vehicleDataMap) {
        LAddress::L2Type vehicleId = vehiclePair.first;
        Vehicle& vehicle = vehiclePair.second;
        
        EV << "[RSU] Processing vehicle " << vehicleId << ":" << std::endl;
        std::cout << "Processing vehicle " << vehicleId << std::endl;
        
        // Find the path from the vehicle's current road to its assigned destination
        vector<string> edgePath;
        
        // If we already stored the path during assignment, use that
        if (!vehicle.plannedPath.empty()) {
            edgePath = vehicle.plannedPath;
            EV << "[RSU] Using previously calculated path for vehicle " << vehicleId << std::endl;
            std::cout << "Using previously calculated path for vehicle " << vehicleId << std::endl;

            // Validate path: make sure it's not too long to avoid SUMO issues
            if (edgePath.size() > 50) {
                // If path is very long, truncate it to just source and destination
                std::string sourceEdge = edgePath.front();
                std::string destEdge = edgePath.back();
                edgePath.clear();
                edgePath.push_back(sourceEdge);
                edgePath.push_back(destEdge);

                EV << "[RSU] Path truncated for vehicle " << vehicleId << " (too long)" << std::endl;
                std::cout << "Path truncated for vehicle " << vehicleId << " (too long)" << std::endl;
            }

            // Print the path
            std::cout << "Path: ";
            for (const auto& edge : edgePath) {
                std::cout << edge << " ";
            }
            std::cout << std::endl;
        }
        else {
            // We don't have a stored path, calculate a new one
            std::string destEdge = vehicle.assignedDestination.nodeId;

            // Since we don't know the current edge, use a default source edge
            std::string sourceEdge = "-1154"; // Example source edge
            EV << "[RSU] No stored path, calculating from " << sourceEdge << " to " << destEdge << std::endl;
            std::cout << "No stored path, calculating from " << sourceEdge << " to " << destEdge << std::endl;

            if (graphProcessor) {
                edgePath = graphProcessor->findEdgeShortestPath(sourceEdge, destEdge);
                
                // Print the calculated path
                std::cout << "Calculated path: ";
                for (const auto& edge : edgePath) {
                    std::cout << edge << " ";
                }
                std::cout << std::endl;
            }
            else {
                EV << "[RSU] ERROR: GraphProcessor not available" << std::endl;
                std::cout << "ERROR: GraphProcessor not available" << std::endl;
            }
        }

        // Simplify the path if needed - sometimes we only need to send the destination
        // This is a fallback if direct route changes fail
        if (edgePath.size() > 1) {
            // Create a direct path that just sends the first and last edge
            std::vector<std::string> simplifiedPath;
            simplifiedPath.push_back(edgePath.front()); // Current edge
            simplifiedPath.push_back(edgePath.back());  // Destination edge

            // If the path is valid, send it
            if (!edgePath.empty()) {
                // First try sending the full path
                sendRerouteMessage(vehicleId, edgePath);
                vehiclesRerouted++;

                // Store the path we sent (for debugging/analysis)
                vehicle.lastSentPath = edgePath;
            }
        }
        else if (!edgePath.empty()) {
            // Just send the destination (single edge)
            sendRerouteMessage(vehicleId, edgePath);
            vehiclesRerouted++;

            // Store the path we sent
            vehicle.lastSentPath = edgePath;
        }
        else {
            EV << "[RSU] No path found for vehicle " << vehicleId << " to destination "
               << vehicle.assignedDestination.nodeId << std::endl;
        }
    }

    EV << "[RSU] Rerouted " << vehiclesRerouted << " vehicles out of "
       << vehicleDataMap.size() << " total vehicles" << std::endl;
    EV << "\n[RSU] ===== REROUTING COMPLETE =====\n" << std::endl;
}

void RSUControlApp::sendRerouteMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& edgePath) {
    int simId = vehicleDataMap[vehicleId].simulationId;
    if (simId == -1) {
        simId = vehicleId;
        EV << "[RSU] WARNING: No simulation ID mapping for vehicle " << vehicleId << std::endl;
    }
    std::ostringstream routeStr;
    routeStr << "CHANGE_ROUTE:" << simId << ":";

    // Include all edges in the path
    if (!edgePath.empty()) {
        for (size_t i = 0; i < edgePath.size(); i++) {
            routeStr << edgePath[i];
            if (i < edgePath.size() - 1) {
                routeStr << " ";  // Space-separated list of edges
            }
        }
        
        std::string message = routeStr.str();

        // Enhanced debug information
        EV << "\n[RSU] ====== SENDING BROADCAST REROUTE MESSAGE ======" << std::endl;
        EV << "[RSU] Target vehicle ID: " << simId << std::endl;
        EV << "[RSU] Message content: " << message << std::endl;
        EV << "[RSU] System time: " << simTime() << std::endl;
        EV << "[RSU] ====================================" << std::endl;


        // Create a broadcast message to all vehicles
        auto* response = new TraCIDemo11pMessage();
        response->setDemoData(message.c_str());
        response->setSenderAddress(myId);

        auto* wsm = new BaseFrame1609_4();
        wsm->encapsulate(response);
        populateWSM(wsm);
        // Important: Do not set a recipient address for broadcast
        // wsm->setRecipientAddress(vehicleId); - Remove this line

        sendDown(wsm);
        
        EV << "[RSU] Broadcast reroute message for vehicle " << simId << ": " << message << std::endl;
    } else {
        EV << "[RSU] Error: Cannot send empty path for vehicle " << vehicleId << " (sim ID: " << simId << ")" << std::endl;
        std::cout << "Error: Cannot send empty path for vehicle " << vehicleId << " (sim ID: " << simId << ")" << std::endl;
    }
}

void RSUControlApp::finish() {
    // Save all statistics to CSV files
    SimulationLogger::getInstance().saveToCSV("simulation_results_1.csv");
    
    // Print summary
    SimulationLogger::getInstance().printSummary();
    
    // Clean up messages
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


void RSUControlApp::findEdgePathAndPrint(std::string sourceEdgeId, std::string targetEdgeId) const {
    if (!graphProcessor) {
        EV << "[RSU] GraphProcessor chưa được khởi tạo" << std::endl;
        return;
    }
    
    EV << "[RSU] Tìm đường đi ngắn nhất từ edge " << sourceEdgeId << " đến edge " << targetEdgeId << std::endl;

    // Get the graph of roads
    const Graph& graph = graphProcessor->getGraph();
    
    bool sourceEdgeFound = false;
    bool targetEdgeFound = false;
    std::string sourceNodeId, targetNodeId;

    // Build a map from edge ID to "from" and "to" junctions
    std::map<std::string, std::pair<std::string, std::string>> edgeToJunctions;
    
    for (const auto& nodePair : graph.getAdjList()) {
        const std::string& fromJunction = nodePair.first;
        for (const auto& edge : nodePair.second) {
            const std::string& edgeId = edge.getId();
            const std::string& toJunction = edge.getTo();
            edgeToJunctions[edgeId] = std::make_pair(fromJunction, toJunction);
            
            if (edge.getId() == sourceEdgeId) {
                sourceEdgeFound = true;
                sourceNodeId = nodePair.first;
                EV << "[RSU] Source edge " << sourceEdgeId << " found: connects from node " 
                   << edge.getFrom() << " to node " << edge.getTo() << std::endl;
            }
            if (edge.getId() == targetEdgeId) {
                targetEdgeFound = true;
                targetNodeId = nodePair.first;
                EV << "[RSU] Target edge " << targetEdgeId << " found: connects from node " 
                   << edge.getFrom() << " to node " << edge.getTo() << std::endl;
            }
        }
    }
    
    if (!sourceEdgeFound) {
        EV << "[RSU] ERROR: Source edge " << sourceEdgeId << " not found in graph!" << std::endl;
        return;
    }
    if (!targetEdgeFound) {
        EV << "[RSU] ERROR: Target edge " << targetEdgeId << " not found in graph!" << std::endl;
        return;
    }

    if (!sourceNodeId.empty() && !targetNodeId.empty()) {
        EV << "[RSU] DEBUG: Checking if there's a node path from " << sourceNodeId << " to " << targetNodeId << std::endl;
        std::vector<std::string> nodePath = graphProcessor->findShortestPath(sourceNodeId, targetNodeId);
        if (!nodePath.empty()) {
            EV << "[RSU] DEBUG: Node path exists with " << nodePath.size() << " segments" << std::endl;
        } else {
            EV << "[RSU] DEBUG: No node path exists between the nodes" << std::endl;
        }
    }
    
    // Get the edge path
    auto edgePath = graphProcessor->findEdgeShortestPath(sourceEdgeId, targetEdgeId);
    // Calculate total path length
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
    
    EV << "[RSU] Đã tìm thấy đường đi với " << edgePath.size() << " phân đoạn, tổng độ dài: " << totalLength << std::endl;

    // Print the path (showing all road IDs in the path, including intermediate roads)
    if (!edgePath.empty()) {
        EV << "[RSU] Đường đi: ";

        // Always try to get a more detailed path between source and destination roads
        const std::string& sourceRoad = edgePath.front();
        const std::string& targetRoad = edgePath.back();
        
        // Get a detailed path between source and destination
        std::vector<std::string> detailedPath = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);

        // If we got a detailed path, use it; otherwise, use junction-to-junction paths to build a complete path
        if (detailedPath.size() > 2) {
            // Print the detailed path with all intermediate roads
            for (size_t j = 0; j < detailedPath.size(); j++) {
                EV << detailedPath[j];
                if (j < detailedPath.size() - 1) {
                    EV << " -> ";
                }
            }
        }
        else {
            // Try to find intermediate road segments using junction connections from the original path
            std::vector<std::string> intermediateRoads;

            // Since original path often just has start and end points or junction placeholders,
            // try to find intermediate roads between junctions
            auto sourceIt = edgeToJunctions.find(sourceRoad);
            auto targetIt = edgeToJunctions.find(targetRoad);
            
            if (sourceIt != edgeToJunctions.end() && targetIt != edgeToJunctions.end()) {
                const std::string& sourceToJunction = sourceIt->second.second;
                const std::string& targetFromJunction = targetIt->second.first;

                // Get a path between junctions
                std::vector<std::string> junctionPath = graphProcessor->findShortestPath(
                    sourceToJunction, targetFromJunction);

                // Build a complete path
                intermediateRoads.push_back(sourceRoad);

                // Add roads connecting the junctions
                for (size_t j = 0; j < junctionPath.size() - 1; j++) {
                    const std::string& fromJunction = junctionPath[j];
                    const std::string& toJunction = junctionPath[j + 1];

                    // Find the edge that connects these junctions
                    for (const auto& nodePair : graph.getAdjList()) {
                        if (nodePair.first == fromJunction) {
                            for (const auto& edge : nodePair.second) {
                                if (edge.getTo() == toJunction) {
                                    intermediateRoads.push_back(edge.getId());
                                    break;
                                }
                            }
                        }
                    }
                }

                intermediateRoads.push_back(targetRoad);
            } else {
                // Fallback to original path if junction info not available
                intermediateRoads = edgePath;
            }

            // Print the expanded intermediate roads
            for (size_t j = 0; j < intermediateRoads.size(); j++) {
                // Skip junction placeholders
                if (intermediateRoads[j].find("(junction") == std::string::npos && 
                    intermediateRoads[j].find("(path from") == std::string::npos) {

                    EV << intermediateRoads[j];
                    if (j < intermediateRoads.size() - 1) {
                        EV << " -> ";
                    }
                }
            }
        }
        
        EV << std::endl;
    } else {
        EV << "[RSU] Không tìm thấy đường đi!" << std::endl;
    }
}

void RSUControlApp::printVehicleRouteInfo(const std::vector<Vehicle>& vehicles) {
    EV << "[RSU] Generating random destinations for vehicles..." << std::endl;
    generateAndAssignDestinations(vehicles);
}

// Helper to check if a string can be converted to an integer
bool isConvertibleToInt(const std::string& str) {
    try {
        size_t pos = 0;
        std::stoi(str, &pos);
        return pos == str.length(); // Check if all characters were processed
    } catch (...) {
        return false;
    }
}

// Helper to extract the numeric index from vehicle IDs like "t_X", "t_1", etc.
// This is a fallback method, now we use the sequential index from VehicleInfo
int extractVehicleIndex(const std::string& vehicleId) {
    // Check if the ID is in format "t_X" where X is a number
    if (vehicleId.length() > 2 && vehicleId.substr(0, 2) == "t_") {
        try {
            return std::stoi(vehicleId.substr(2)); // Extract the number after "t_"
        } catch (...) {
            // If conversion fails, return -1
            return -1;
        }
    }
    
    // If it's already a numeric ID, try to convert it directly
    try {
        return std::stoi(vehicleId);
    } catch (...) {
        return -1; // Invalid ID
    }
}

void RSUControlApp::generateAndAssignDestinations(const std::vector<Vehicle>& vehicles) {
    if (!taskGenerator || !graphProcessor || vehicles.empty()) {
        EV << "[RSU] ERROR: Cannot generate destinations - required components not initialized" << std::endl;
        return;
    }

    EV << "\n=============== VEHICLE DESTINATION ASSIGNMENT ===============" << std::endl;
    std::vector<std::string> sourceRoads;
    for (const auto& vehicle : vehicles) {
        sourceRoads.push_back(vehicle.startingRoad);
        double startTime = vehicle.departTime;
        try {
            recordVehicleStart(vehicle.index, vehicle.startingRoad, startTime);
            EV << "[RSU] Vehicle " << vehicle.sumoId << " (index: " << vehicle.index 
               << ") starting at road " << vehicle.startingRoad << " at time " << startTime << std::endl;
        } catch (const std::exception& e) {
            EV << "[RSU] Error recording vehicle start: " << e.what() << std::endl;
        }
    }

    EV << "[RSU] Generating random destinations with time windows for " << vehicles.size() << " vehicles" << std::endl;
    
    // Tạo seed ngẫu nhiên
    unsigned seedValue = static_cast<unsigned>(simTime().raw() + std::time(nullptr)) % UINT_MAX;
    
    // Tạo destinations với time windows ngẫu nhiên
    const Graph& graph = graphProcessor->getGraph();
    auto destObjects = taskGenerator->generateDestinationsWithTimeWindows(
        vehicles.size(), sourceRoads, graph, seedValue);
    
    // Chuẩn bị danh sách edges cho Hungarian
    std::vector<std::string> destEdges;
    for (const auto& dest : destObjects) {
        destEdges.push_back(dest.nodeId);
    }

    // Xây dựng ma trận độ lệch thời gian cho Hungarian
    int numVehicles = sourceRoads.size();
    int numDestinations = destObjects.size();
    int n = std::max(numVehicles, numDestinations);
    std::vector<std::vector<double>> deviationMatrix(n, std::vector<double>(n, 0));
    const double NO_PATH_PENALTY = 9999999.0;
    
    // Lưu thời gian di chuyển tối ưu và đường đi
    std::map<std::pair<int, int>, std::pair<std::vector<std::string>, double>> pathsAndTimes;
    
    EV << "\n[RSU] ====== CALCULATING TIME DEVIATION MATRIX ======" << std::endl;
    std::cout << "\n====== CALCULATING TIME DEVIATION MATRIX ======" << std::endl;
    
    // Start measuring the TOTAL algorithm time here - BEFORE we start building the cost matrix
    // This includes both Dijkstra for every source-destination pair AND Hungarian algorithm
    auto startFullAlgorithm = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < numVehicles; ++i) {
        const std::string& sourceEdgeId = sourceRoads[i];
        
        for (int j = 0; j < numDestinations; ++j) {
            const std::string& destEdgeId = destEdges[j];
            auto path = graphProcessor->findEdgeShortestPath(sourceEdgeId, destEdgeId);

            if (!path.empty()) {
                // Tính thời gian di chuyển tối ưu = tổng (độ dài / tốc độ max)
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
                
                // Tính thời gian dự kiến = 1.2 × thời gian tối ưu
                double expectedTime = 1.2 * optimalTime;
                
                // Tính độ lệch so với time window dựa trên thời gian dự kiến
                const TimeWindow& tw = destObjects[j].timeWindow;
                double deviation = 0.0;
                
                if (expectedTime < tw.earliness) {
                    // Đến sớm
                    deviation = tw.earliness - expectedTime;
                } else if (expectedTime > tw.tardiness) {
                    // Đến muộn
                    deviation = expectedTime - tw.tardiness;
            } else {
                    // Trong khung thời gian
                    deviation = 0.0;
                }
                
                deviationMatrix[i][j] = deviation;
                pathsAndTimes[{i, j}] = {path, optimalTime};
                
                // Log chi tiết
                EV << "  Vehicle " << i << " -> Dest " << j << ": "
                   << "optimal=" << optimalTime << "s, "
                   << "expected=" << expectedTime << "s, "
                   << "window=[" << tw.earliness << ", " << tw.tardiness << "], "
                   << "deviation=" << deviation << std::endl;
                   
                std::cout << "  Vehicle " << i << " -> Dest " << j << ": "
                          << "optimal=" << optimalTime << "s, "
                          << "expected=" << expectedTime << "s, "
                          << "window=[" << tw.earliness << ", " << tw.tardiness << "], "
                          << "deviation=" << deviation << std::endl;
                
            } else {
                deviationMatrix[i][j] = NO_PATH_PENALTY;
                EV << "  Vehicle " << i << " -> Dest " << j << ": NO PATH" << std::endl;
                std::cout << "  Vehicle " << i << " -> Dest " << j << ": NO PATH" << std::endl;
            }
        }
    }
    
    EV << "====================================================\n" << std::endl;
    std::cout << "====================================================\n" << std::endl;
    
    // Padding matrix cho trường hợp không vuông
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
    
    // DEBUG: In ma trận deviation trước khi gọi Hungarian
    EV << "\n[RSU] DEBUG: Deviation Matrix being sent to Hungarian Algorithm:" << std::endl;
    std::cout << "\nDEBUG: Deviation Matrix being sent to Hungarian Algorithm:" << std::endl;
    for (int i = 0; i < numVehicles && i < 10; ++i) { // Chỉ in 10 xe đầu để tránh spam
        for (int j = 0; j < numDestinations && j < 10; ++j) { // Chỉ in 10 đích đầu
            EV << std::fixed << std::setprecision(2) << deviationMatrix[i][j] << "\t";
            std::cout << std::fixed << std::setprecision(2) << deviationMatrix[i][j] << "\t";
        }
        EV << std::endl;
        std::cout << std::endl;
    }
    EV << std::endl;
    std::cout << std::endl;
    
    // Sử dụng Hungarian với ma trận độ lệch
    EV << "[RSU] Computing optimal assignment using Hungarian algorithm with time deviation matrix" << std::endl;
    
    // Run the Hungarian algorithm once (no artificial workload)
    std::vector<int> assignment = graphProcessor->getOptimalAssignmentWithMatrix(deviationMatrix);
    
    // END TIMING HERE - total algorithm time = building cost matrix + hungarian algorithm
    auto endFullAlgorithm = std::chrono::high_resolution_clock::now();
    
    // Calculate total algorithm time in seconds
    std::chrono::duration<double> totalAlgorithmDuration = endFullAlgorithm - startFullAlgorithm;
    double totalAlgorithmTime = totalAlgorithmDuration.count();
    
    // This is the REAL total algorithm time for the entire process
    EV << "[RSU] TOTAL ALGORITHM EXECUTION TIME: " << std::fixed << std::setprecision(6) 
       << totalAlgorithmTime << " seconds" << std::endl;
    std::cout << "\n************************************************************" << std::endl;
    std::cout << "TOTAL ALGORITHM EXECUTION TIME: " << std::fixed << std::setprecision(6) 
              << totalAlgorithmTime << " seconds" << std::endl;
    std::cout << "************************************************************\n" << std::endl;
    
    // Ghi nhận tổng thời gian thuật toán cho toàn bộ mô phỏng
    SimulationLogger::getInstance().recordTotalAlgorithmTime(totalAlgorithmTime);
    
    // In kết quả phân công
    EV << "\n[RSU] ================ HUNGARIAN ASSIGNMENT RESULTS ================" << std::endl;
    std::cout << "\n================ HUNGARIAN ASSIGNMENT RESULTS ================" << std::endl;
    double totalDeviation = 0.0;
    int onTimeCount = 0;
    
    for (int i = 0; i < assignment.size() && i < numVehicles; i++) {
        int destIndex = assignment[i];
        if (destIndex != -1 && destIndex < destObjects.size()) {
            double deviation = deviationMatrix[i][destIndex];
            totalDeviation += (deviation < NO_PATH_PENALTY ? deviation : 0);
            if (deviation == 0) onTimeCount++;
            
            // Lấy thông tin chi tiết
            auto pathTimeKey = std::make_pair(i, destIndex);
            double optimalTime = pathsAndTimes[pathTimeKey].second;
            double expectedTime = 1.2 * optimalTime;
            const TimeWindow& tw = destObjects[destIndex].timeWindow;
            
            // Tạo chuỗi trạng thái đến
            std::string arrivalStatus;
            if (deviation == 0) {
                arrivalStatus = "ON TIME";
            } else if (expectedTime < tw.earliness) {
                arrivalStatus = "EARLY by " + std::to_string(deviation) + "s";
            } else {
                arrivalStatus = "LATE by " + std::to_string(deviation) + "s";
            }
                
            EV << "  Vehicle " << i << " (road: " << sourceRoads[i] 
               << ") -> Destination " << destObjects[destIndex].nodeId
               << " | Deviation: " << deviation << "s | Status: " << arrivalStatus << std::endl;
               
            std::cout << "  Vehicle " << i << " (road: " << sourceRoads[i] 
                      << ") -> Destination " << destObjects[destIndex].nodeId
                      << " | Deviation: " << deviation << "s | Status: " << arrivalStatus << std::endl;
        }
    }
    
    EV << "\n[RSU] SUMMARY:" << std::endl;
    EV << "  Total time deviation: " << totalDeviation << " seconds" << std::endl;
    EV << "  Vehicles arriving on time: " << onTimeCount << " out of " << numVehicles << std::endl;
    EV << "  Average deviation per vehicle: " << (numVehicles > 0 ? totalDeviation/numVehicles : 0) << " seconds" << std::endl;
    
    std::cout << "\nSUMMARY:" << std::endl;
    std::cout << "  Total time deviation: " << totalDeviation << " seconds" << std::endl;
    std::cout << "  Vehicles arriving on time: " << onTimeCount << " out of " << numVehicles << std::endl;
    std::cout << "  Average deviation per vehicle: " << (numVehicles > 0 ? totalDeviation/numVehicles : 0) << " seconds" << std::endl;
    std::cout << "================================================================\n" << std::endl;

    // Lưu thông tin assignment cho mỗi xe
    for (size_t i = 0; i < assignment.size() && i < vehicles.size(); ++i) {
        int destIndex = assignment[i];
        if (destIndex == -1 || destIndex >= destObjects.size()) {
            continue;
        }
        
        const std::string& sourceRoad = sourceRoads[i];
        const std::string& targetRoad = destObjects[destIndex].nodeId;
        
        // Lấy path và optimal time đã tính
        auto pathTimeKey = std::make_pair(i, destIndex);
        auto path = pathsAndTimes[pathTimeKey].first;
        double optimalTime = pathsAndTimes[pathTimeKey].second;
        
        // Tính tổng độ dài đường đi
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
        
        EV << "[RSU] Vehicle " << vehicles[i].sumoId << " (index: " << vehicles[i].index << ") route:" << std::endl;
        EV << "    From: Road " << sourceRoad << std::endl;
        EV << "    To: Road " << targetRoad << std::endl;
        EV << "    Path length: " << totalDistance << " m" << std::endl;
        EV << "    Optimal travel time: " << optimalTime << " s" << std::endl;
        EV << "    Time window: [" << destObjects[destIndex].timeWindow.earliness 
           << ", " << destObjects[destIndex].timeWindow.tardiness << "]" << std::endl;
        
        double deviation = deviationMatrix[i][destIndex];
        double expectedTime = 1.2 * optimalTime;
        if (deviation == 0) {
            EV << "    Expected to arrive ON TIME" << std::endl;
        } else if (expectedTime < destObjects[destIndex].timeWindow.earliness) {
            EV << "    Expected to arrive EARLY by " << deviation << " seconds" << std::endl;
        } else {
            EV << "    Expected to arrive LATE by " << deviation << " seconds" << std::endl;
        }
        
        // Ghi nhận thông tin
        try {
            recordVehicleDestination(vehicles[i].index, targetRoad, 
                                    destObjects[destIndex].timeWindow.earliness,
                                    destObjects[destIndex].timeWindow.tardiness,
                                    path, totalDistance);
            
            // Không ghi riêng cho từng xe nữa
            // recordAlgorithmTime(vehicles[i].index, totalAlgorithmTime);
            } catch (const std::exception& e) {
            EV << "[RSU] Error recording vehicle data: " << e.what() << std::endl;
    }
    
        // Cập nhật vehicleDataMap
        int vehicleIndex = vehicles[i].index;
            vehicleDataMap[vehicleIndex].assignedDestination = destObjects[destIndex];
            vehicleDataMap[vehicleIndex].plannedPath = path;
            vehicleDataMap[vehicleIndex].earliestArrival = destObjects[destIndex].timeWindow.earliness;
            vehicleDataMap[vehicleIndex].latestArrival = destObjects[destIndex].timeWindow.tardiness;
        vehicleDataMap[vehicleIndex].pathLength = totalDistance;
        vehicleDataMap[vehicleIndex].estimatedTravelTime = optimalTime;
            vehicleDataMap[vehicleIndex].startTime = vehicles[i].departTime;
            
        // Cập nhật simulation ID mapping nếu cần
        if (vehicleDataMap[vehicleIndex].simulationId == -1) {
            int simulationId = 16 + 6 * vehicleIndex;
            vehicleDataMap[vehicleIndex].simulationId = simulationId;
            simulationIdToAddressMap[simulationId] = vehicleIndex;
        }
    }

    EV << "\n[RSU] Assignment completed for " << vehicles.size() << " vehicles" << std::endl;
    EV << "==============================================================\n" << std::endl;
}

double RSUControlApp::getEdgeLength(const std::string& edgeId) const {
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

// Add new methods for logging

void RSUControlApp::recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime) {
    // Chuyển đổi từ index nội bộ sang simulation ID
    int simulationId = -1;
    
    // Sử dụng mapping hiện có của RSUControlApp
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    } else {
        // Nếu không tìm thấy mapping, sử dụng công thức tính
        simulationId = 16 + 6 * vehicleId;
        // Tự động tạo mapping cho xe này
        vehicleDataMap[vehicleId].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleId;
        EV << "[RSU] Auto-creating mapping for vehicle " << vehicleId 
           << " with simulation ID " << simulationId << std::endl;
    }
    
    // Ghi lại với ID simulation
    SimulationLogger::getInstance().recordVehicleStart(simulationId, startRoad, startTime);
    
    // Ghi log
    EV << "[RSU] Vehicle " << vehicleId << " (sim ID: " << simulationId << ") started at time " << startTime 
       << " from road " << startRoad << std::endl;
}

void RSUControlApp::recordVehicleDestination(int vehicleId, const std::string& targetRoad, 
                                            double earliestArrival, double latestArrival, 
                                            const std::vector<std::string>& path, double pathLength) {
    int simulationId = -1;
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    } else {
        // Nếu không tìm thấy mapping, sử dụng công thức tính
        simulationId = 16 + 6 * vehicleId;
        // Tự động tạo mapping cho xe này
        vehicleDataMap[vehicleId].simulationId = simulationId;
        simulationIdToAddressMap[simulationId] = vehicleId;
        EV << "[RSU] Auto-creating mapping for vehicle " << vehicleId 
           << " with simulation ID " << simulationId << std::endl;
    }
    SimulationLogger::getInstance().updateVehicleDestination(
        simulationId, targetRoad, earliestArrival, latestArrival, path, pathLength);
}

// Removed recordAlgorithmTime method as we now record total algorithm time directly

void RSUControlApp::processVehicleDepartureNotification(const std::string& data) {
    std::istringstream iss(data);
    std::string action;
    int vehicleId;
    iss >> action >> vehicleId;
    
    EV << "[RSU] Received notification: Vehicle " << vehicleId 
       << " " << action << " at time " << simTime() << std::endl;
}

void RSUControlApp::processVehicleArrivalNotification(const std::string& data) {
    std::istringstream iss(data);
    std::string action;
    int vehicleId;
    double arrivalTime;
    iss >> action >> vehicleId >> arrivalTime;
    
    if (action == "ARRIVAL") {
        // Record vehicle arrival to SimulationLogger
        SimulationLogger::getInstance().recordVehicleEnd(vehicleId, arrivalTime);
        
        EV << "[RSU] Vehicle " << vehicleId << " arrived at destination at time " 
           << arrivalTime << std::endl;
    }
}
