#include "RSUControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <sstream>
#include <iostream>
#include <cstring>
#include <algorithm> // Add include for std::min

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
        
        // Create a fixed mapping between RSU internal IDs and simulation IDs from TreeView
        // This is the mapping from the TreeView: node[0] = Car id=14, node[1] = Car id=20, etc.
        std::map<int, int> vehicleIdMapping = {
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
        };
        
        // Pre-populate the vehicle data map with correct simulation IDs
        for (const auto& mapping : vehicleIdMapping) {
            int internalId = mapping.first;
            int simulationId = mapping.second;
            
            // Create an entry for this vehicle in our data structure
            vehicleDataMap[internalId].simulationId = simulationId;
            
            EV << "[RSU] Pre-mapped internal vehicle " << internalId 
               << " to simulation ID " << simulationId << std::endl;
            std::cout << "RSU: Pre-mapped internal vehicle " << internalId 
                      << " to simulation ID " << simulationId << std::endl;
        }
        
        statusCheckMsg = new cMessage("checkStatus");
        rerouteMsg = new cMessage("rerouteVehicles");
        scheduleAt(simTime() + 1.0, statusCheckMsg);
        scheduleAt(simTime() + 1.5, rerouteMsg); // Schedule rerouting at t=1.5s
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
                printRoadNetworkInfo();
                printNodeInfo();
                testTaskGenerator();
                
                // Load and print vehicle information from route file
                EV << "[RSU] Attempting to load route information" << std::endl;
                if (xmlProcessor->loadRouteFile("./erlangen.rou.xml")) {
                    std::vector<VehicleInfo> vehicles = xmlProcessor->getVehicles();
                    printVehicleRouteInfo(vehicles);
                } else {
                    EV << "[RSU] Failed to load route file" << std::endl;
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
    std::string data = msg->getDemoData();
    vehicleDataMap[senderId].lastMessageTime = simTime();
    
    // Check if this is a status message containing the simulation ID
    if (data.find("STATUS:") == 0) {
        // Extract simulation ID if present (new format uses simId= instead of id=)
        size_t idPos = data.find("simId=");
        if (idPos != std::string::npos) {
            size_t idEnd = data.find(';', idPos);
            if (idEnd != std::string::npos) {
                std::string idStr = data.substr(idPos + 6, idEnd - idPos - 6); // +6 for "simId="
                try {
                    int simId = std::stoi(idStr);
                    updateVehicleIdMapping(senderId, simId);
                    
                    // Debug output
                    EV << "[RSU] Updated vehicle ID mapping from STATUS message: Address " << senderId 
                       << " -> Simulation ID " << simId << std::endl;
                    std::cout << "RSU: Updated vehicle ID mapping from STATUS: Address " << senderId 
                              << " -> Simulation ID " << simId << std::endl;
                } catch (const std::exception& e) {
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
    // Handle K paths request
    else if (message.find("FIND_K_PATHS:") == 0 && graphProcessor && taskGenerator) {
        EV << "[RSU] DEBUG: Received FIND_K_PATHS request: " << message << std::endl;
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
            std::cout << "No path found for vehicle " << vehicleId << " to destination "
                      << vehicleData.assignedDestination.nodeId << std::endl;
        }
    }
    
    EV << "[RSU] Rerouted " << vehiclesRerouted << " vehicles out of " 
       << vehicleDataMap.size() << " total vehicles" << std::endl;
    std::cout << "Rerouted " << vehiclesRerouted << " vehicles out of "
              << vehicleDataMap.size() << " total vehicles" << std::endl;
    
    EV << "\n[RSU] ===== REROUTING COMPLETE =====\n" << std::endl;
}

void RSUControlApp::sendRerouteMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& edgePath) {
    // Get the correct simulation ID for this vehicle
    int simId = vehicleDataMap[vehicleId].simulationId;
    if (simId == -1) {
        // If no mapping exists, use the original ID (shouldn't happen with pre-mapping)
        simId = vehicleId;
        EV << "[RSU] WARNING: No simulation ID mapping for vehicle " << vehicleId << std::endl;
        std::cout << "RSU WARNING: No simulation ID mapping for vehicle " << vehicleId << std::endl;
    }
    
    // Build the CHANGE_ROUTE message with the complete path
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
    cancelAndDelete(statusCheckMsg);
    cancelAndDelete(rerouteMsg); // Clean up the reroute message
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

// Implement the method to find the shortest path
vector<string> RSUControlApp::findShortestPath(const string& sourceId, const string& targetId) const {
    if (!graphProcessor) {
        return vector<string>();
    }
    
    return graphProcessor->findShortestPath(sourceId, targetId);
}

// Implement the method to get the length of the shortest path
double RSUControlApp::getShortestPathLength(const string& sourceId, const string& targetId) const {
    if (!graphProcessor) {
        return -1.0;
    }
    
    return graphProcessor->getShortestPathLength(sourceId, targetId);
}

// Implement method to print road information
void RSUControlApp::printRoadNetworkInfo() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] No road network loaded" << endl;
        return;
    }
    
    const vector<string> allRoads = getAllRoads();
    EV << "\n[RSU] ========== ROAD NETWORK INFORMATION ==========\n" << std::endl;
    EV << "[RSU] Total number of roads: " << allRoads.size() << std::endl;
    
    int maxRoadsToShow = min(static_cast<int>(allRoads.size()), 20);
    for (int i = 0; i < maxRoadsToShow; i++) {
        const string& roadId = allRoads[i];
        EV << "[RSU] Road ID: " << roadId << endl;
        
        const Graph& graph = xmlProcessor->getGraph();
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == roadId) {
                    EV << "  From: " << edge.getFrom() << " To: " << edge.getTo() << std::endl;
                    EV << "  Length: " << edge.getLength() << std::endl;
                    const auto& lanes = edge.getLanes();
                    EV << "  Lanes: " << lanes.size() << std::endl;
                    for (const auto& lane : lanes) {
                        EV << "    Lane ID: " << lane.id << " Index: " << lane.index 
                                  << " Speed: " << lane.speed << " Length: " << lane.length << std::endl;
                    }
                    break;
                }
            }
        }
    }
    
    if (allRoads.size() > maxRoadsToShow) {
        EV << "[RSU] ... and " << (allRoads.size() - maxRoadsToShow) << " more roads" << endl;
    }
    
    EV << "\n[RSU] ========== SHORTEST PATH TESTS ==========\n" << std::endl;
    if (!graphProcessor) {
        EV << "[RSU] GraphProcessor not initialized, cannot perform path finding tests" << endl;
        return;
    }
    if (allRoads.size() >= 2) {
        string sourceRoad = "42";
        string targetRoad = "1945";
        
        EV << "[RSU] Path Test: Finding path from " << sourceRoad << " to " << targetRoad << endl;
        vector<string> path = findShortestPath(sourceRoad, targetRoad);
        double pathLength = getShortestPathLength(sourceRoad, targetRoad);
        
        if (!path.empty() && pathLength > 0) {
            EV << "[RSU] SUCCESS: Path found with length " << pathLength << " units" << endl;
            EV << "[RSU] Path segments (" << path.size() << "): ";
            for (size_t i = 0; i < path.size(); i++) {
                EV << path[i];
                if (i < path.size() - 1) {
                    EV << " -> ";
                }
            }
            EV << endl;
        } else {
            EV << "[RSU] No path found between " << sourceRoad << " and " << targetRoad << endl;

            const Graph& graph = xmlProcessor->getGraph();
            const auto& nodes = graph.getNodes();
            if (nodes.size() >= 2) {
                auto it = nodes.begin();
                string sourceNode = it->first;
                advance(it, nodes.size() - 1);
                string targetNode = it->first;
                
                EV << "[RSU] Trying path between nodes instead: " << sourceNode << " to " << targetNode << endl;
                path = findShortestPath(sourceNode, targetNode);
                pathLength = getShortestPathLength(sourceNode, targetNode);
                
                if (!path.empty() && pathLength > 0) {
                    EV << "[RSU] SUCCESS: Node path found with length " << pathLength << " units" << endl;
                    EV << "[RSU] Path segments (" << path.size() << "): ";
                    for (size_t i = 0; i < path.size(); i++) {
                        EV << path[i];
                        if (i < path.size() - 1) {
                            EV << " -> ";
                        }
                    }
                    EV << endl;
                } else {
                    EV << "[RSU] NO PATH found between nodes " << sourceNode << " and " << targetNode << std::endl;
                }
            }
        }
    } else {
        EV << "[RSU] Not enough roads to perform path finding tests" << std::endl;
    }

    EV << "\n[RSU] Graph Check: " << endl;
    const Graph& graph = xmlProcessor->getGraph();
    EV << "  - Node count: " << graph.getNodeCount() << endl;
    EV << "  - Edge count: " << graph.getEdgeCount() << endl;
    EV << "\n[RSU] ==========================================\n" << endl;
}

void RSUControlApp::printNodeInfo() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] No road network loaded" << endl;
        return;
    }
    
    const Graph& graph = xmlProcessor->getGraph();
    const auto& nodes = graph.getNodes();
    
    EV << "\n[RSU] ========== NODE INFORMATION ==========\n" << endl;
    EV << "[RSU] Total number of nodes: " << nodes.size() << endl;

    int count = 0;
    for (const auto& nodePair : nodes) {
        if (count >= 20) break;
        
        const Node& node = nodePair.second;
        EV << "[RSU] Node ID: " << node.getId() 
           << " X: " << node.getX() 
           << " Y: " << node.getY() << endl;
        count++;
    }
    
    if (nodes.size() > 20) {
        EV << "[RSU] ... and " << (nodes.size() - 20) << " more nodes" << endl;
    }
    
    EV << "\n[RSU] =======================================\n" << endl;
}

void RSUControlApp::testTaskGenerator() {
    if (!taskGenerator) {
        EV << "[RSU] ERROR: Cannot test TaskGenerator - not initialized" << endl;
        return;
    }
    
    EV << "\n[RSU] ========== TASK GENERATOR TEST ==========\n" << endl;
    
    // Test 1: Generate random destinations
    EV << "[RSU] Test 1: Generating 3 random destinations" << std::endl;
    auto destinations = taskGenerator->generateDestinations(3);
    
    EV << "[RSU] Generated " << destinations.size() << " destinations:" << std::endl;
    for (const auto& dest : destinations) {
        EV << "  - Node " << dest.nodeId << " (time window: "
           << dest.timeWindow.earliness << " - " << dest.timeWindow.tardiness << ")" << std::endl;
    }

    // Test 2: Find k shortest paths
    if (graphProcessor) {
        const auto& roads = getAllRoads();
        if (roads.size() >= 10) {
            std::string sourceId = "830";
            std::string targetId = "1914";

            EV << "\n[RSU] Test 2: Finding 16 shortest paths from " << sourceId << " to " << targetId << std::endl;

            auto paths = taskGenerator->findKPaths(sourceId, targetId, 16);

            EV << "[RSU] Found " << paths.size() << " paths:" << std::endl;
            for (size_t i = 0; i < paths.size(); i++) {
                const auto& path = paths[i];
                double pathLength = 0.0;
                for (const auto& edgeId : path) {
                    const Graph& graph = xmlProcessor->getGraph();
                    for (const auto& nodePair : graph.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                pathLength += edge.getLength();
                                break;
                            }
                        }
                    }
                }

                EV << "  - Path " << (i+1) << " (segments: " << path.size() << ", length: " << pathLength << "): ";
                for (size_t j = 0; j < path.size(); j++) {
                    EV << path[j];
                    if (j < path.size() - 1) {
                        EV << " -> ";
                    }
                }
                EV << std::endl;
            }
        } else {
            EV << "\n[RSU] Test 2: Not enough roads for path finding test" << std::endl;
        }
    }
    
    // Test 3: Check valid assignment
    //    EV << "\n[RSU] Test 3: Testing valid assignment" << std::endl;
    //
    //    std::vector<std::string> sources = {"1024", "213", "337"};
    //    std::vector<std::string> targets = {"1985", "853", "205"};
    //
    //    bool validAssignment = taskGenerator->existsValidAssignment(sources, targets);

    //    EV << "[RSU] Valid assignment exists: " << (validAssignment ? "YES" : "NO") << std::endl;

    EV << "\n[RSU] Test 3: Tìm đường đi giữa các edge" << std::endl;
    findEdgePathAndPrint("-184", "1939");

    EV << "\n[RSU] =========================================\n" << std::endl;
}

void RSUControlApp::findLanePathAndPrint(std::string sourceLaneId, std::string targetLaneId) const {
    if (!graphProcessor) {
        return;
    }
    
    EV << "[RSU] Tìm đường đi ngắn nhất từ làn " << sourceLaneId << " đến làn " << targetLaneId << std::endl;
    
    auto lanePath = graphProcessor->findLaneShortestPath(sourceLaneId, targetLaneId);
    
    if (lanePath.empty()) {
        EV << "[RSU] Không tìm thấy đường đi giữa hai làn đường" << std::endl;
        return;
    }
    
    double totalCost = 0.0;
    for (const auto& segment : lanePath) {
        totalCost += segment.cost;
    }
    
    EV << "[RSU] Đã tìm thấy đường đi với " << lanePath.size() << " phân đoạn, tổng chi phí: " << totalCost << std::endl;
    
    for (size_t i = 0; i < lanePath.size(); i++) {
        const auto& segment = lanePath[i];
        EV << "  - Edge: " << segment.edgeId << ", Lane: " << segment.laneIndex 
           << ", Chi phí: " << segment.cost << std::endl;
    }
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
    EV << "\n=============== VEHICLE ROUTE INFORMATION ===============" << std::endl;
    EV << "Loaded " << vehicles.size() << " vehicles from route file:" << std::endl;
    
    int maxVehiclesToDisplay = std::min(static_cast<size_t>(20), vehicles.size());
    for (int i = 0; i < maxVehiclesToDisplay; i++) {
        EV << "  - Vehicle ID: " << vehicles[i].id 
           << ", Depart: " << vehicles[i].depart 
           << ", From: " << vehicles[i].from 
           << ", To: " << vehicles[i].to;
        
        if (!vehicles[i].via.empty()) {
            EV << ", Via: " << vehicles[i].via;
        }
        
        EV << std::endl;
    }
    
    if (vehicles.size() > maxVehiclesToDisplay) {
        EV << "  ... and " << (vehicles.size() - maxVehiclesToDisplay) << " more vehicles" << std::endl;
    }
    
    EV << "=========================================================" << std::endl;
    
    // Generate destinations for vehicles
    EV << "[RSU] Generating random destinations for vehicles..." << std::endl;
    generateAndAssignDestinations(vehicles);
}

void RSUControlApp::generateAndAssignDestinations(const std::vector<VehicleInfo>& vehicles) {
    if (!taskGenerator || !graphProcessor || vehicles.empty()) {
        EV << "[RSU] ERROR: Cannot generate destinations - required components not initialized" << std::endl;
        return;
    }
    
    EV << "\n=============== VEHICLE DESTINATION ASSIGNMENT ===============" << std::endl;
    
    std::vector<std::string> sourceRoads;
    for (const auto& vehicle : vehicles) {
        sourceRoads.push_back(vehicle.from);
        EV << "[RSU] Vehicle " << vehicle.id << " starting at road " << vehicle.from << std::endl;
    }
    
    EV << "[RSU] Generating optimal destinations for " << sourceRoads.size() << " vehicles" << std::endl;
    int destsToGenerate = sourceRoads.size();
    auto destinations = taskGenerator->getPotentialDestinationEdges(destsToGenerate, sourceRoads);
    
    // Create Destination objects from the edge IDs
    std::vector<Destination> destObjects;
    
    // First create destination objects without time windows
    for (const auto& edgeId : destinations) {
        destObjects.emplace_back(edgeId, TimeWindow(0, 0)); // Placeholder time windows
    }
    
    // Calculate travel times and set time windows based on that
    const Graph& graph = graphProcessor->getGraph();
    
    for (size_t i = 0; i < destObjects.size() && i < sourceRoads.size(); ++i) {
        // Calculate path
        auto path = graphProcessor->findEdgeShortestPath(sourceRoads[i], destObjects[i].nodeId);
        
        // Calculate total distance
        double totalDistance = 0.0;
        for (const auto& edgeId : path) {
            bool edgeFound = false;
            for (const auto& nodePair : graph.getAdjList()) {
                for (const auto& edge : nodePair.second) {
                    if (edge.getId() == edgeId) {
                        totalDistance += edge.getLength();
                        edgeFound = true;
                        break;
                    }
                }
                if (edgeFound) break;
            }
            if (!edgeFound) {
                totalDistance += 100.0;
            }
        }
        
        // Calculate estimated travel time with an average speed
        double averageSpeed = 13.89; // m/s
        double estimatedTime = totalDistance / averageSpeed;
        if (estimatedTime <= 0) {
            estimatedTime = 1.0; // Set a minimum travel time
        }

        // Set time window to 1.5-2.5 times the minimum travel time
        double earliness = 1.5 * estimatedTime;
        double tardiness = 2.5 * estimatedTime;

        // Update the time window for this destination
        destObjects[i].timeWindow.earliness = earliness;
        destObjects[i].timeWindow.tardiness = tardiness;
    }

    if (destObjects.size() < vehicles.size()) {
        EV << "[RSU] WARNING: Not enough destinations generated (" << destObjects.size()
           << " for " << vehicles.size() << " vehicles)" << std::endl;
    }

    EV << "[RSU] Generated " << destObjects.size() << " destinations with time windows:" << std::endl;
    for (size_t i = 0; i < destObjects.size(); ++i) {
        const auto& dest = destObjects[i];
        EV << "  - Destination for Vehicle " << i << " (Source: " << (i < sourceRoads.size() ? sourceRoads[i] : "N/A") << "): Edge " << dest.nodeId
           << " (Time window: " << dest.timeWindow.earliness
           << " - " << dest.timeWindow.tardiness << ")" << std::endl;
    }

    // Print the effective Cost Matrix based on assignments
    EV << "\n[RSU] Effective Cost Matrix (Path Lengths for Assigned Routes):" << std::endl;
    EV << "       "; // Header space for vehicle column
    for(size_t j=0; j < destObjects.size(); ++j) {
        std::string destHeader = "D_" + destObjects[j].nodeId.substr(0, 4);
        EV << destHeader << "\t";
    }
    EV << std::endl;

    for (size_t i = 0; i < sourceRoads.size(); ++i) {
        std::string rowStr = "V_" + vehicles[i].id.substr(0,4) + "(E:" + sourceRoads[i].substr(0,4) + ")\t";
        for (size_t j = 0; j < destObjects.size(); ++j) {
            // Find the path length for this specific source vehicle to this specific destination edge
            // This might not be the globally optimal if the number of dests < vehicles
            // but it shows the cost for the assignment made if vehicle i got dest j
            double pathLength = -1.0;
            if (i < destObjects.size() && sourceRoads[i] == vehicles[i].from ) { // Ensure we are matching correctly
                 // Dùng findEdgeShortestPath để có kết quả chính xác và nhất quán với hiển thị đường đi
                 auto path = graphProcessor->findEdgeShortestPath(sourceRoads[i], destObjects[j].nodeId);
                 pathLength = 0.0;

                 // Tính độ dài đường đi bằng cách cộng chiều dài mỗi edge
                 if (!path.empty()) {
                    const Graph& graph = graphProcessor->getGraph();
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
                        if (!edgeFound) {
                            pathLength += 100.0; // Default length
                        }
                    }
                 } else {
                    pathLength = -1.0; // No path found
                 }
            }

            if (pathLength > 0 && pathLength < 9999999.0) {
                rowStr += std::to_string(static_cast<double>(pathLength)) + "\t";
            } else {
                rowStr += "INF\t";
            }
        }
        EV << rowStr << std::endl;
    }
    
    EV << "\n[RSU] Computing routes for each vehicle to its assigned destination:" << std::endl;
    
//    const Graph& graph = graphProcessor->getGraph();
    int processedVehicles = std::min(static_cast<int>(vehicles.size()), static_cast<int>(destObjects.size()));

    for (int i = 0; i < processedVehicles; ++i) {
        const std::string& sourceRoad = sourceRoads[i];
        const std::string& targetRoad = destObjects[i].nodeId;
        
        auto path = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
        
        EV << "  - Vehicle " << vehicles[i].id << " route:" << std::endl;
        EV << "    From: Road " << sourceRoad << std::endl;
        EV << "    To: Road " << targetRoad << std::endl;
        
        if (path.empty()) {
            EV << "    WARNING: No path found!" << std::endl;
            continue;
        }
        
        double totalDistance = 0.0;
        for (const auto& edgeId : path) {
                            bool edgeFound = false;
                            for (const auto& nodePair : graph.getAdjList()) {
                                    for (const auto& edge : nodePair.second) {
                    if (edge.getId() == edgeId) {
                        totalDistance += edge.getLength();
                                            edgeFound = true;
                                            break;
                                    }
                                }
                                if (edgeFound) break;
                            }
                            if (!edgeFound) {
                totalDistance += 100.0;
            }
        }
        
        double averageSpeed = 13.89;
        double estimatedTime = totalDistance / averageSpeed;
        if (estimatedTime <= 0) {
            estimatedTime = 1.0;
        }
        
        EV << "    Path length: " << totalDistance << " m" << std::endl;
        EV << "    Estimated travel time: " << estimatedTime << " s" << std::endl;
        EV << "    Time window: " << destObjects[i].timeWindow.earliness
           << " - " << destObjects[i].timeWindow.tardiness
           << " (approx: " << (destObjects[i].timeWindow.earliness / estimatedTime)
           << "x - " << (destObjects[i].timeWindow.tardiness / estimatedTime) << "x of minimal travel time)" << std::endl;
        
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
    }

    for (size_t i = 0; i < processedVehicles; i++) {
        LAddress::L2Type vehicleId = i;
        vehicleDataMap[vehicleId].assignedDestination = destObjects[i];
        
        // Store the assigned path
        const std::string& sourceRoad = sourceRoads[i];
        const std::string& targetRoad = destObjects[i].nodeId;
        auto path = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
        vehicleDataMap[vehicleId].assignedPath = path;
        
        // Print detailed path information for debugging
        std::cout << "Stored path for vehicle " << vehicleId << " from " << sourceRoad << " to " << targetRoad << ":" << std::endl;
        std::cout << "  Path has " << path.size() << " edges: ";
        for (const auto& edge : path) {
            std::cout << edge << " ";
        }
        std::cout << std::endl;
    }

    EV << "\n[RSU] Assignment completed for " << processedVehicles << " vehicles" << std::endl;
    
    // Print a summary of vehicle data map contents
    std::cout << "Vehicle data map after assignment:" << std::endl;
    for (const auto& pair : vehicleDataMap) {
        std::cout << "  Vehicle ID: " << pair.first 
                  << ", destination: " << pair.second.assignedDestination.nodeId
                  << ", path size: " << pair.second.assignedPath.size() << std::endl;
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
