#include "RSUControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include<bits/stdc++.h>
#include <ctime>    // Add include for std::time
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
        map<int, int> vehicleIdMapping = {
            {0, 16},
            {1, 22},
            {2, 28},
            {3, 34},
            {4, 40},
            {5, 46},
            {6, 52},
            {7, 58},
            {8, 64},
            {9, 70},
            {10, 76},
            {11, 82},
            {12, 88},
            {13, 94},
            {14, 100},
            {15, 106},
            {16, 112},
            {17, 118},
            {18, 124},
            {19, 130},
            {20, 136},
            {21, 142},
            {22, 148},
            {23, 154},
            {24, 160},
            {25, 166},
            {26, 172},
            {27, 178},
            {28, 184},
            {29, 190},
            {30, 196},
            {31, 202},
            {32, 208},
            {33, 214},
            {34, 220},
            {35, 226},
            {36, 232},
            {37, 238}
        };
        for (const auto& mapping : vehicleIdMapping) {
            int internalId = mapping.first;
            int simulationId = mapping.second;
            vehicleDataMap[internalId].simulationId = simulationId;
            EV << "[RSU] Pre-mapped internal vehicle " << internalId
               << " to simulation ID " << simulationId << std::endl;
        }
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
                    vector<VehicleInfo> vehicles = xmlProcessor->getVehicles();
                    printVehicleRouteInfo(vehicles);
                } else {
                    EV << "[RSU] Failed to load route file" << endl;
                }
            }
        }
    }
}

void RSUControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* enc = wsm->getEncapsulatedPacket();
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(enc);
    if (!msg) return;
    LAddress::L2Type senderId = msg->getSenderAddress();
    string data = msg->getDemoData();
    vehicleDataMap[senderId].lastMessageTime = simTime();
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
    // Store the simulation ID in the vehicle data
    vehicleDataMap[vehicleAddress].simulationId = simulationId;

    // Update the reverse mapping
    simulationIdToAddressMap[simulationId] = vehicleAddress;
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
    // Handle generate destinations request
    else if (message.find("GENERATE_DESTINATIONS:") == 0 && taskGenerator) {
        EV << "[RSU] DEBUG: Received GENERATE_DESTINATIONS request: " << message << std::endl;
        
        try {
            int count = std::stoi(message.substr(22)); // Skip "GENERATE_DESTINATIONS:"
            EV << "[RSU] Generating " << count << " random destinations" << std::endl;
            
            auto destinations = taskGenerator->generateDestinations(count);

            // Convert destinations to a string format
            std::vector<std::string> destStrings;
            for (const auto& dest : destinations) {
                std::string destStr = dest.nodeId + "," +
                                     std::to_string(dest.timeWindow.earliness) + "," +
                                     std::to_string(dest.timeWindow.tardiness);
                destStrings.push_back(destStr);
                EV << "[RSU] DEBUG: Generated destination: " << destStr << std::endl;
            }
            
            EV << "[RSU] Sending " << destStrings.size() << " destinations to vehicle" << std::endl;
            sendRoadListMessage(vehicleId, destStrings);
        }
        catch (const std::exception& e) {
            EV << "[RSU] ERROR in GENERATE_DESTINATIONS: " << e.what() << std::endl;
            std::vector<std::string> errorMsg = {"ERROR: Failed to generate destinations"};
            sendRoadListMessage(vehicleId, errorMsg);
        }
    }
    // Handle K paths request
    else if (message.find("FIND_K_PATHS:") == 0 && graphProcessor && taskGenerator) {
        EV << "[RSU] DEBUG: Received FIND_K_PATHS request: " << message << std::endl;
        try {
            std::string params = message.substr(13);
            size_t firstComma = params.find(',');
            size_t secondComma = params.find(',', firstComma + 1);
            
            if (firstComma != std::string::npos && secondComma != std::string::npos) {
                std::string sourceId = params.substr(0, firstComma);
                std::string targetId = params.substr(firstComma + 1, secondComma - firstComma - 1);
                int k = std::stoi(params.substr(secondComma + 1));
                
                EV << "[RSU] Finding " << k << " paths from " << sourceId << " to " << targetId << std::endl;
                
                auto paths = taskGenerator->findKPaths(sourceId, targetId, k);
                
                EV << "[RSU] Found " << paths.size() << " paths" << std::endl;
                std::vector<std::string> flattenedPaths;
                for (size_t i = 0; i < paths.size(); i++) {
                    const auto& path = paths[i];
                    
                    // Log each path for debugging
                    EV << "[RSU] DEBUG: Path " << (i+1) << " (length " << path.size() << "): ";
                    for (size_t j = 0; j < path.size(); j++) {
                        EV << path[j];
                        if (j < path.size() - 1) EV << " -> ";
                    }
                    EV << std::endl;
                    
                    for (const auto& roadId : path) {
                        flattenedPaths.push_back(roadId);
                    }
                    flattenedPaths.push_back("PATH_SEPARATOR");
                }
                
                sendRoadListMessage(vehicleId, flattenedPaths);
            } else {
                EV << "[RSU] ERROR: Malformed K paths request" << std::endl;
                std::vector<std::string> errorMsg = {"ERROR: Malformed K paths request"};
                sendRoadListMessage(vehicleId, errorMsg);
            }
        }
        catch (const std::exception& e) {
            EV << "[RSU] ERROR in FIND_K_PATHS: " << e.what() << std::endl;
            std::vector<std::string> errorMsg = {"ERROR: Failed to find paths"};
            sendRoadListMessage(vehicleId, errorMsg);
        }
    }
    // Handle valid assignment check
    else if (message.find("EXISTS_VALID_ASSIGNMENT:") == 0 && taskGenerator) {
        EV << "[RSU] DEBUG: Received EXISTS_VALID_ASSIGNMENT request: " << message << std::endl;
        
        string params = message.substr(24); // Skip "EXISTS_VALID_ASSIGNMENT:"
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
               << ", path size: " << pair.second.assignedPath.size() << std::endl;
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
        VehicleData& vehicleData = vehiclePair.second;
        
        EV << "[RSU] Processing vehicle " << vehicleId << ":" << std::endl;
        std::cout << "Processing vehicle " << vehicleId << std::endl;
        
        // Find the path from the vehicle's current road to its assigned destination
        vector<string> edgePath;
        
        // If we already stored the path during assignment, use that
        if (!vehicleData.assignedPath.empty()) {
            edgePath = vehicleData.assignedPath;
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
            std::string destEdge = vehicleData.assignedDestination.nodeId;

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
                vehicleData.lastSentPath = edgePath;
            }
        }
        else if (!edgePath.empty()) {
            // Just send the destination (single edge)
            sendRerouteMessage(vehicleId, edgePath);
            vehiclesRerouted++;

            // Store the path we sent
            vehicleData.lastSentPath = edgePath;
        }
        else {
            EV << "[RSU] No path found for vehicle " << vehicleId << " to destination "
               << vehicleData.assignedDestination.nodeId << std::endl;
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

        std::cout << "\nRSU SENDING BROADCAST: Target vehicle ID: " << simId << std::endl;
        std::cout << "RSU SENDING: Message content: " << message << std::endl;
        std::cout << "RSU SENDING: System time: " << simTime() << std::endl;

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
    SimulationLogger::getInstance().saveToCSV("simulation_results.csv");
    
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

void RSUControlApp::printVehicleRouteInfo(const std::vector<VehicleInfo>& vehicles) {
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

void RSUControlApp::generateAndAssignDestinations(const std::vector<VehicleInfo>& vehicles) {
    if (!taskGenerator || !graphProcessor || vehicles.empty()) {
        EV << "[RSU] ERROR: Cannot generate destinations - required components not initialized" << std::endl;
        return;
    }

    EV << "\n=============== VEHICLE DESTINATION ASSIGNMENT ===============" << std::endl;
    
    // Store vehicles and their source roads
    std::vector<std::string> sourceRoads;
    
    for (const auto& vehicle : vehicles) {
        sourceRoads.push_back(vehicle.from);
        
        // Record vehicle start using the vehicle's index
        double startTime = vehicle.depart;
        try {
            recordVehicleStart(vehicle.index, vehicle.from, startTime);
            
            EV << "[RSU] Vehicle " << vehicle.id << " (index: " << vehicle.index 
               << ") starting at road " << vehicle.from << " at time " << startTime << std::endl;
        } catch (const std::exception& e) {
            EV << "[RSU] Error recording vehicle start: " << e.what() << std::endl;
        }
    }

    EV << "[RSU] Generating optimal destinations for " << vehicles.size() << " vehicles" << std::endl;
    int destsToGenerate = vehicles.size();
    // Add a unique seed based on current simulation time to ensure different random destinations each run
    unsigned seedValue = static_cast<unsigned>(simTime().raw() + std::time(nullptr)) % UINT_MAX;
    auto destinations = taskGenerator->getPotentialDestinationEdges(destsToGenerate, sourceRoads, seedValue);

    // Create Destination objects from the edge IDs
    std::vector<Destination> destObjects;

    // First create destination objects without time windows
    for (const auto& edgeId : destinations) {
        destObjects.emplace_back(edgeId, TimeWindow(0, 0)); // Placeholder time windows
    }

    // Calculate travel times and set time windows based on that
    const Graph& graph = graphProcessor->getGraph();
    
    // Create vectors for the Hungarian algorithm
    std::vector<std::string> destEdges;
    for (const auto& dest : destObjects) {
        destEdges.push_back(dest.nodeId);
    }

    // Prepare cost matrix for analysis and total distance calculation
    int numVehicles = sourceRoads.size();
    int numDestinations = destEdges.size();
    int n = std::max(numVehicles, numDestinations);
    std::vector<std::vector<double>> costMatrix(n, std::vector<double>(n, 0));
    const double NO_PATH_PENALTY = 9999999.0;
    for (int i = 0; i < numVehicles; ++i) {
        const std::string& sourceEdgeId = sourceRoads[i];
        for (int j = 0; j < numDestinations; ++j) {
            const std::string& destEdgeId = destEdges[j];
            auto path = graphProcessor->findEdgeShortestPath(sourceEdgeId, destEdgeId);
            double pathLength = 0.0;

            if (!path.empty()) {
                // Calculate the length of the path
                for (const auto& edgeId : path) {
                    bool edgeFound = false;
                    for (const auto& nodePair : graph.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                pathLength += edge.getLength();
                                edgeFound = true;
                                break;
                            }
                        }
                        if (edgeFound) break;
                    }
                }
                costMatrix[i][j] = pathLength;
            } else {
                costMatrix[i][j] = NO_PATH_PENALTY;
            }
        }
    }
    
    // Get optimal assignment using the Hungarian algorithm
    EV << "[RSU] Computing optimal vehicle-destination assignment using Hungarian algorithm" << std::endl;
    std::vector<int> assignment = graphProcessor->getOptimalVehicleAssignment(sourceRoads, destEdges);
    
    // Print the assignment result
    EV << "[RSU] Assignment results:" << std::endl;
    for (int i = 0; i < assignment.size(); i++) {
        int destIndex = assignment[i];
        if (destIndex != -1 && destIndex < destObjects.size()) {
            EV << "  Vehicle " << i << " (road: " << sourceRoads[i] 
               << ") assigned to destination " << destObjects[destIndex].nodeId << std::endl;
        } else {
            EV << "  Vehicle " << i << " (road: " << sourceRoads[i]
               << ") could not be assigned a destination" << std::endl;
        }
    }

    // Calculate and print total distance
    double totalDistance = 0.0;
    int assignedCount = 0;
    
    EV << "\n[RSU] ====== DISTANCE ANALYSIS OF HUNGARIAN ASSIGNMENT ======" << std::endl;
    std::cout << "\n====== DISTANCE ANALYSIS OF HUNGARIAN ASSIGNMENT ======" << std::endl;
    
    for (int i = 0; i < assignment.size(); i++) {
        int destIndex = assignment[i];
        if (destIndex != -1 && destIndex < numDestinations) {
            double distance = costMatrix[i][destIndex];
            if (distance < NO_PATH_PENALTY) {
                totalDistance += distance;
                assignedCount++;
                
                EV << "  Vehicle " << i << " (from " << sourceRoads[i] << ") to destination " 
                   << destEdges[destIndex] << ": " << distance << " meters" << std::endl;
                std::cout << "  Vehicle " << i << " (from " << sourceRoads[i] << ") to destination " 
                         << destEdges[destIndex] << ": " << distance << " meters" << std::endl;
            }
        }
    }
    
    EV << "\n[RSU] Total distance for all vehicles: " << totalDistance << " meters" << std::endl;
    EV << "[RSU] Average distance per vehicle: " << (assignedCount > 0 ? totalDistance / assignedCount : 0) 
       << " meters" << std::endl;
    std::cout << "\nTotal distance for all vehicles: " << totalDistance << " meters" << std::endl;
    std::cout << "Average distance per vehicle: " << (assignedCount > 0 ? totalDistance / assignedCount : 0) 
             << " meters" << std::endl;
    std::cout << "Assigned vehicles: " << assignedCount << " out of " << numVehicles << std::endl;
    std::cout << "=====================================================" << std::endl;

    // Calculate paths and time windows based on the assignment
    for (size_t i = 0; i < assignment.size() && i < vehicles.size(); ++i) {
        int destIndex = assignment[i];
        if (destIndex == -1 || destIndex >= destObjects.size()) {
            continue;  // Skip unassigned vehicles
        }
        
        const std::string& sourceRoad = sourceRoads[i];
        const std::string& targetRoad = destObjects[destIndex].nodeId;
        
        // Bắt đầu đo thời gian thuật toán
        auto startAlgorithm = std::chrono::high_resolution_clock::now();
        
        // Calculate path
        auto path = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
        
        // Kết thúc đo thời gian thuật toán
        auto endAlgorithm = std::chrono::high_resolution_clock::now();
        double algorithmTime = std::chrono::duration<double>(endAlgorithm - startAlgorithm).count();
        
        // Calculate total distance and travel time based on segment speeds
        double totalDistance = 0.0;
        double estimatedTime = 0.0;
        
        for (const auto& edgeId : path) {
            bool edgeFound = false;
            double edgeLength = 0.0;
            double edgeSpeed = 13.89; // Default speed if not found
            
            for (const auto& nodePair : graph.getAdjList()) {
                for (const auto& edge : nodePair.second) {
                    if (edge.getId() == edgeId) {
                        edgeLength = edge.getLength();
                        
                        // Use the Edge's getMaxSpeed method
                        edgeSpeed = edge.getMaxSpeed();
                        
                        totalDistance += edgeLength;
                        // Calculate time to traverse this segment and add to total
                        double segmentTime = edgeLength / edgeSpeed;
                        estimatedTime += segmentTime;
                        
                        edgeFound = true;
                        break;
                    }
                }
                if (edgeFound) break;
            }
            
            if (!edgeFound) {
                totalDistance += 100.0;
                estimatedTime += 100.0 / 13.89; // Default values
            }
        }
        
        // Set time window based on estimated travel time
        double earliness = 1.3 * estimatedTime;
        double tardiness = 1.7 * estimatedTime;
        destObjects[destIndex].timeWindow.earliness = earliness;
        destObjects[destIndex].timeWindow.tardiness = tardiness;
        
        EV << "[RSU] Vehicle " << vehicles[i].id << " (index: " << vehicles[i].index << ") route:" << std::endl;
        EV << "    From: Road " << sourceRoad << std::endl;
        EV << "    To: Road " << targetRoad << std::endl;
        EV << "    Path length: " << totalDistance << " m" << std::endl;
        EV << "    Estimated travel time: " << estimatedTime << " s" << std::endl;
        EV << "    Time window: " << earliness << " - " << tardiness 
           << " (approx: " << (earliness / estimatedTime) << "x - " 
           << (tardiness / estimatedTime) << "x of minimal travel time)" << std::endl;
        
        if (path.size() <= 20) {
            EV << "    Path: ";
            for (size_t j = 0; j < path.size(); j++) {
                EV << path[j];
                if (j < path.size() - 1) {
                    EV << " -> ";
                }
            }
            EV << std::endl;
        } else {
            EV << "    Path: " << path[0] << " -> ... -> " << path[path.size()-1]
               << " (" << path.size() << " segments)" << std::endl;
        }
        
        // Ghi nhận thông tin đích và khung thời gian cho xe
        try {
            recordVehicleDestination(vehicles[i].index, targetRoad, 
                                    earliness, tardiness,
                                    path, totalDistance);
        } catch (const std::exception& e) {
            EV << "[RSU] Error recording vehicle destination: " << e.what() << std::endl;
        }
        
        // Record algorithm time
        if (i < vehicles.size()) {
            try {
                recordAlgorithmTime(vehicles[i].index, algorithmTime);
            } catch (const std::exception& e) {
                EV << "[RSU] Error recording algorithm time: " << e.what() << std::endl;
            }
        }
    }
    
    // Update vehicleDataMap with assignment information
    for (size_t i = 0; i < vehicles.size(); i++) {
        // Get the vehicle's index from VehicleInfo
        int vehicleIndex = vehicles[i].index;
        
        // Get the corresponding simulation ID from the mapping
        auto it = vehicleDataMap.find(vehicleIndex);
        if (it == vehicleDataMap.end() || it->second.simulationId == -1) {
            EV << "[RSU] WARNING: Vehicle with index " << vehicleIndex << " not found in vehicleDataMap" << std::endl;
            continue;
        }
        
        int simulationId = it->second.simulationId;
        
        // Store destination and path data in vehicleDataMap
        int destIndex = (i < assignment.size()) ? assignment[i] : -1;
        if (destIndex != -1 && destIndex < destObjects.size()) {
            vehicleDataMap[vehicleIndex].assignedDestination = destObjects[destIndex];
            
            // Store the assigned path
            const std::string& sourceRoad = sourceRoads[i];
            const std::string& targetRoad = destObjects[destIndex].nodeId;
            auto path = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
            vehicleDataMap[vehicleIndex].assignedPath = path;
            
            // Lưu khung thời gian cho xe
            vehicleDataMap[vehicleIndex].earliestArrival = destObjects[destIndex].timeWindow.earliness;
            vehicleDataMap[vehicleIndex].latestArrival = destObjects[destIndex].timeWindow.tardiness;
            
            // Calculate path length and estimated travel time
            double pathLength = 0.0;
            double estimatedTravelTime = 0.0;
            for (const auto& edgeId : path) {
                // Find edge in graph
                for (const auto& nodePair : graph.getAdjList()) {
                    for (const auto& edge : nodePair.second) {
                        if (edge.getId() == edgeId) {
                            double edgeLength = edge.getLength();
                            pathLength += edgeLength;
                            
                            // Use edge's getMaxSpeed method
                            double edgeSpeed = edge.getMaxSpeed();
                            estimatedTravelTime += edgeLength / edgeSpeed;
                            
                            break;
                        }
                    }
                }
            }
            
            vehicleDataMap[vehicleIndex].pathLength = pathLength;
            vehicleDataMap[vehicleIndex].estimatedTravelTime = estimatedTravelTime;
            
            // Lưu thời gian bắt đầu của xe (từ dữ liệu SUMO)
            vehicleDataMap[vehicleIndex].startTime = vehicles[i].depart;
            
            EV << "[RSU] Vehicle with index " << vehicleIndex << " mapped to simulation ID " 
               << simulationId << " with destination " << targetRoad 
               << " and time window [" << destObjects[destIndex].timeWindow.earliness 
               << ", " << destObjects[destIndex].timeWindow.tardiness << "]" << std::endl;
            
            std::cout << "Vehicle " << simulationId << " assigned to destination " << targetRoad 
                     << " with time window [" << destObjects[destIndex].timeWindow.earliness 
                     << ", " << destObjects[destIndex].timeWindow.tardiness << "]" << std::endl;
        }
    }

    EV << "\n[RSU] Assignment completed for " << vehicles.size() << " vehicles" << std::endl;

    // Print a summary of vehicle data map contents
    std::cout << "Vehicle data map after assignment:" << std::endl;
    for (const auto& pair : vehicleDataMap) {
        std::cout << "  Vehicle ID: " << pair.first
                  << ", simulation ID: " << pair.second.simulationId
                  << ", destination: " << pair.second.assignedDestination.nodeId
                  << ", path size: " << pair.second.assignedPath.size() 
                  << ", time window: [" << pair.second.earliestArrival 
                  << ", " << pair.second.latestArrival << "]" << std::endl;
    }

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
    // Convertir l'index interne en ID de simulation en utilisant le mapping existant
    int simulationId = -1;
    
    // Utiliser le mapping existant de RSUControlApp
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    } else {
        // Véhicule non trouvé dans le mapping, utiliser l'ID original comme fallback
        simulationId = vehicleId;
        std::cout << "WARNING: Vehicle " << vehicleId << " not found in pre-defined mapping, using original ID" << std::endl;
    }
    
    // Enregistrer avec l'ID de simulation
    SimulationLogger::getInstance().recordVehicleStart(simulationId, startRoad, startTime);
    
    // Log
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
    }
    SimulationLogger::getInstance().updateVehicleDestination(
        simulationId, targetRoad, earliestArrival, latestArrival, path, pathLength);
}

void RSUControlApp::recordAlgorithmTime(int vehicleId, double algorithmTime) {
    int simulationId = -1;
    auto it = vehicleDataMap.find(vehicleId);
    if (it != vehicleDataMap.end() && it->second.simulationId != -1) {
        simulationId = it->second.simulationId;
    }
    SimulationLogger::getInstance().recordAlgorithmTime(simulationId, algorithmTime);
    EV << "[RSU] Vehicle " << vehicleId << " (sim ID: " << simulationId << ") routing algorithm took " 
       << algorithmTime << " seconds" << std::endl;
}

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

