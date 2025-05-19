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
        statusCheckMsg = new cMessage("checkStatus");
        scheduleAt(simTime() + 1.0, statusCheckMsg);
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
    handleVehicleMessage(data, senderId);
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
    // Handle generate optimal destinations request
    else if (message.find("GENERATE_OPTIMAL_DESTINATIONS:") == 0 && taskGenerator) {
        EV << "[RSU] DEBUG: Received GENERATE_OPTIMAL_DESTINATIONS request: " << message << std::endl;
        
        std::string paramsStr = message.substr(29); // Skip "GENERATE_OPTIMAL_DESTINATIONS:"
        size_t lastCommaPos = paramsStr.find_last_of(',');
        
        if (lastCommaPos != std::string::npos) {
            std::string sourceNodesStr = paramsStr.substr(0, lastCommaPos);
            int count = std::stoi(paramsStr.substr(lastCommaPos + 1));
            
            // Parse source nodes (vehicle positions)
            std::vector<std::string> sourceNodes;
            size_t pos = 0;
            std::string token;
            while ((pos = sourceNodesStr.find(';')) != std::string::npos) {
                token = sourceNodesStr.substr(0, pos);
                if (!token.empty()) {
                    sourceNodes.push_back(token);
                }
                sourceNodesStr.erase(0, pos + 1);
            }
            if (!sourceNodesStr.empty()) {
                sourceNodes.push_back(sourceNodesStr);
            }
            
            EV << "[RSU] Generating " << count << " optimal destinations for " 
               << sourceNodes.size() << " vehicles" << std::endl;
            
            // Get the current vehicle's source node
            std::string currentVehicleNode = "";
            if (!sourceNodes.empty()) {
                currentVehicleNode = sourceNodes[0]; // Use the first source as the current vehicle
            } else {
                // If no source nodes provided, just skip this step
                // We don't have road information stored in VehicleData
            }
            
            // Generate optimal destinations
            auto destinations = taskGenerator->generateOptimalDestinations(sourceNodes, count);
            
            // Convert destinations to a string format
            std::vector<std::string> destStrings;
            for (const auto& dest : destinations) {
                std::string destStr = dest.nodeId + "," + 
                                     std::to_string(dest.timeWindow.earliness) + "," + 
                                     std::to_string(dest.timeWindow.tardiness);
                destStrings.push_back(destStr);
                EV << "[RSU] DEBUG: Generated optimal destination: " << destStr << std::endl;
            }
            
            EV << "[RSU] Sending " << destStrings.size() << " optimal destinations to vehicle" << std::endl;
            sendRoadListMessage(vehicleId, destStrings);
        } else {
            EV << "[RSU] ERROR: Malformed optimal destinations request" << std::endl;
        }
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
    } else {
        TraCIDemoRSU11p::handleSelfMsg(msg);
    }
}

void RSUControlApp::finish() {
    cancelAndDelete(statusCheckMsg);
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
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        return vector<string>();
    }
    return xmlProcessor->getAllRoads();
}

vector<string> RSUControlApp::getAllNodes() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        return vector<string>();
    }
    
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
    
    // Get all roads from the graph to use as potential destinations
    std::vector<std::string> allRoads = getAllRoads();
    
    // Filter roads to use as destinations (remove negative road IDs)
    std::vector<std::string> potentialDestRoads;
    for (const auto& roadId : allRoads) {
        // Only consider positive road IDs (not starting with '-')
        if (!roadId.empty() && roadId[0] != '-') {
            potentialDestRoads.push_back(roadId);
        }
    }
    
    // Print some debug info
    EV << "[RSU] Found " << potentialDestRoads.size() << " potential destination roads" << std::endl;
    
    // Check if we have enough potential destinations
    if (potentialDestRoads.size() < vehicles.size()) {
        EV << "[RSU] WARNING: Not enough potential destinations in the network" << std::endl;
        return;
    }
    
    // Get the source roads for each vehicle
    std::vector<std::string> sourceRoads;
    for (const auto& vehicle : vehicles) {
        sourceRoads.push_back(vehicle.from);
    }
    
    // Get graph for reference
    const Graph& graph = graphProcessor->getGraph();
    
    // Build a map from edge ID to "from" and "to" junctions
    std::map<std::string, std::pair<std::string, std::string>> edgeToJunctions;
    
    for (const auto& nodePair : graph.getAdjList()) {
        const std::string& fromJunction = nodePair.first;
        for (const auto& edge : nodePair.second) {
            const std::string& edgeId = edge.getId();
            const std::string& toJunction = edge.getTo();
            edgeToJunctions[edgeId] = std::make_pair(fromJunction, toJunction);
        }
    }
    
    // Debug: Print some edge-junction relationships
    int debugCount = 0;
    for (const auto& entry : edgeToJunctions) {
        if (debugCount < 5) {
            EV << "[RSU] Edge " << entry.first << ": from junction " << entry.second.first 
               << " to junction " << entry.second.second << std::endl;
            debugCount++;
        }
    }
    
    // Assign destinations and find paths
    std::vector<std::string> targetRoads;
    std::vector<Destination> destinations;
    std::vector<bool> roadUsed(potentialDestRoads.size(), false);
    
    // Track found paths for each vehicle
    std::vector<std::vector<std::string>> vehiclePaths;
    
    // For each vehicle, assign a destination road and find a path
    for (size_t i = 0; i < sourceRoads.size(); i++) {
        std::string sourceRoad = sourceRoads[i];
        
        // First try to find a valid path to some destination
        bool pathFound = false;
        std::vector<std::string> bestPath;
        double bestPathLength = 0;
        std::string bestDestRoad;
        
        // Try a limited number of potential destinations
        int startIdx = rand() % potentialDestRoads.size();
        
        for (size_t attempt = 0; attempt < std::min(size_t(100), potentialDestRoads.size()); attempt++) {
            size_t destIdx = (startIdx + attempt) % potentialDestRoads.size();
            
            // Skip if this road is already used or same as source
            if (roadUsed[destIdx] || potentialDestRoads[destIdx] == sourceRoad) {
                continue;
            }
            
            std::string destRoad = potentialDestRoads[destIdx];
            
            // Get the junctions for source and destination roads
            auto sourceIt = edgeToJunctions.find(sourceRoad);
            auto destIt = edgeToJunctions.find(destRoad);
            
            if (sourceIt != edgeToJunctions.end() && destIt != edgeToJunctions.end()) {
                const std::string& sourceFromJunction = sourceIt->second.first;
                const std::string& sourceToJunction = sourceIt->second.second;
                const std::string& destFromJunction = destIt->second.first;
                const std::string& destToJunction = destIt->second.second;
                
                // Find a path between the junctions
                // Try from source.to -> dest.from (most natural path)
                std::vector<std::string> junctionPath = graphProcessor->findShortestPath(sourceToJunction, destFromJunction);
                double pathLength = graphProcessor->getShortestPathLength(sourceToJunction, destFromJunction);
                
                if (!junctionPath.empty() && pathLength > 0 && std::isfinite(pathLength)) {
                    // We found a valid path between junctions
                    // Build a complete path: sourceRoad -> junction path -> destRoad
                    std::vector<std::string> fullPath;
                    fullPath.push_back(sourceRoad);
                    
                    // Add all intermediate roads from junction path
                    // Each pair of consecutive junctions in the path corresponds to an edge
                    for (size_t j = 0; j < junctionPath.size() - 1; j++) {
                        const std::string& fromJunction = junctionPath[j];
                        const std::string& toJunction = junctionPath[j + 1];
                        
                        // Find the edge that connects these junctions
                        bool edgeFound = false;
                        for (const auto& nodePair : graph.getAdjList()) {
                            if (nodePair.first == fromJunction) {
                                for (const auto& edge : nodePair.second) {
                                    if (edge.getTo() == toJunction) {
                                        fullPath.push_back(edge.getId());
                                        edgeFound = true;
                                        break;
                                    }
                                }
                            }
                            if (edgeFound) break;
                        }
                        
                        // If no edge found between these junctions, add a placeholder
                        if (!edgeFound) {
                            fullPath.push_back("(junction " + fromJunction + " to " + toJunction + ")");
                        }
                    }
                    
                    // Add the destination road
                    fullPath.push_back(destRoad);
                    
                    bestPath = fullPath;
                    bestPathLength = pathLength;
                    bestDestRoad = destRoad;
                    pathFound = true;
                    break;
                }
                
                // If that didn't work, try other junction combinations
                // Try source.from -> dest.from
                if (!pathFound) {
                    junctionPath = graphProcessor->findShortestPath(sourceFromJunction, destFromJunction);
                    pathLength = graphProcessor->getShortestPathLength(sourceFromJunction, destFromJunction);
                    
                    if (!junctionPath.empty() && pathLength > 0 && std::isfinite(pathLength)) {
                        // Build complete path: sourceRoad -> junction path -> destRoad
                        std::vector<std::string> fullPath;
                        fullPath.push_back(sourceRoad);
                        
                        // Add all intermediate roads from junction path
                        // Each pair of consecutive junctions in the path corresponds to an edge
                        for (size_t j = 0; j < junctionPath.size() - 1; j++) {
                            const std::string& fromJunction = junctionPath[j];
                            const std::string& toJunction = junctionPath[j + 1];
                            
                            // Find the edge that connects these junctions
                            bool edgeFound = false;
                            for (const auto& nodePair : graph.getAdjList()) {
                                if (nodePair.first == fromJunction) {
                                    for (const auto& edge : nodePair.second) {
                                        if (edge.getTo() == toJunction) {
                                            fullPath.push_back(edge.getId());
                                            edgeFound = true;
                                            break;
                                        }
                                    }
                                }
                                if (edgeFound) break;
                            }
                            
                            // If no edge found between these junctions, add a placeholder
                            if (!edgeFound) {
                                fullPath.push_back("(junction " + fromJunction + " to " + toJunction + ")");
                            }
                        }
                        
                        fullPath.push_back(destRoad);
                        
                        bestPath = fullPath;
                        bestPathLength = pathLength;
                        bestDestRoad = destRoad;
                        pathFound = true;
                        break;
                    }
                }
                
                // Try source.from -> dest.to
                if (!pathFound) {
                    junctionPath = graphProcessor->findShortestPath(sourceFromJunction, destToJunction);
                    pathLength = graphProcessor->getShortestPathLength(sourceFromJunction, destToJunction);
                    
                    if (!junctionPath.empty() && pathLength > 0 && std::isfinite(pathLength)) {
                        // Build complete path: sourceRoad -> junction path -> destRoad
                        std::vector<std::string> fullPath;
                        fullPath.push_back(sourceRoad);
                        
                        // Add all intermediate roads from junction path
                        // Each pair of consecutive junctions in the path corresponds to an edge
                        for (size_t j = 0; j < junctionPath.size() - 1; j++) {
                            const std::string& fromJunction = junctionPath[j];
                            const std::string& toJunction = junctionPath[j + 1];
                            
                            // Find the edge that connects these junctions
                            bool edgeFound = false;
                            for (const auto& nodePair : graph.getAdjList()) {
                                if (nodePair.first == fromJunction) {
                                    for (const auto& edge : nodePair.second) {
                                        if (edge.getTo() == toJunction) {
                                            fullPath.push_back(edge.getId());
                                            edgeFound = true;
                                            break;
                                        }
                                    }
                                }
                                if (edgeFound) break;
                            }
                            
                            // If no edge found between these junctions, add a placeholder
                            if (!edgeFound) {
                                fullPath.push_back("(junction " + fromJunction + " to " + toJunction + ")");
                            }
                        }
                        
                        fullPath.push_back(destRoad);
                        
                        bestPath = fullPath;
                        bestPathLength = pathLength;
                        bestDestRoad = destRoad;
                        pathFound = true;
                        break;
                    }
                }
                
                // Try source.to -> dest.to
                if (!pathFound) {
                    junctionPath = graphProcessor->findShortestPath(sourceToJunction, destToJunction);
                    pathLength = graphProcessor->getShortestPathLength(sourceToJunction, destToJunction);
                    
                    if (!junctionPath.empty() && pathLength > 0 && std::isfinite(pathLength)) {
                        // Build complete path: sourceRoad -> junction path -> destRoad
                        std::vector<std::string> fullPath;
                        fullPath.push_back(sourceRoad);
                        
                        // Add all intermediate roads from junction path
                        // Each pair of consecutive junctions in the path corresponds to an edge
                        for (size_t j = 0; j < junctionPath.size() - 1; j++) {
                            const std::string& fromJunction = junctionPath[j];
                            const std::string& toJunction = junctionPath[j + 1];
                            
                            // Find the edge that connects these junctions
                            bool edgeFound = false;
                            for (const auto& nodePair : graph.getAdjList()) {
                                if (nodePair.first == fromJunction) {
                                    for (const auto& edge : nodePair.second) {
                                        if (edge.getTo() == toJunction) {
                                            fullPath.push_back(edge.getId());
                                            edgeFound = true;
                                            break;
                                        }
                                    }
                                }
                                if (edgeFound) break;
                            }
                            
                            // If no edge found between these junctions, add a placeholder
                            if (!edgeFound) {
                                fullPath.push_back("(junction " + fromJunction + " to " + toJunction + ")");
                            }
                        }
                        
                        fullPath.push_back(destRoad);
                        
                        bestPath = fullPath;
                        bestPathLength = pathLength;
                        bestDestRoad = destRoad;
                        pathFound = true;
                        break;
                    }
                }
            }
        }
        
        if (pathFound) {
            // Find the index of the destination road in the potentialDestRoads array
            size_t bestDestIdx = std::find(potentialDestRoads.begin(), potentialDestRoads.end(), bestDestRoad) - potentialDestRoads.begin();
            
            // Mark the destination as used
            if (bestDestIdx < roadUsed.size()) {
                roadUsed[bestDestIdx] = true;
            }
            
            // Store the destination information
            targetRoads.push_back(bestDestRoad);
            
            // Calculate travel time based on actual road lengths and speeds
            double estimatedTime = 0.0;
            
            // If we have a full path with intermediate roads
            if (!bestPath.empty()) {
                // Calculate time based on each road segment's length and speed
                for (const auto& roadId : bestPath) {
                    // Skip junction placeholders
                    if (roadId.find("(junction") == std::string::npos && 
                        roadId.find("(path from") == std::string::npos) {
                        
                        // Find the edge object for this road
                        for (const auto& nodePair : graph.getAdjList()) {
                            for (const auto& edge : nodePair.second) {
                                if (edge.getId() == roadId) {
                                    // Get actual length and maximum speed
                                    double length = edge.getLength();
                                    double maxSpeed = edge.getMaxSpeed();
                                    
                                    // Use a speed that's 80% of the maximum (typical actual speed)
                                    double speed = maxSpeed * 0.8;
                                    
                                    // Add time for this road segment
                                    if (speed > 0.0) {
                                        estimatedTime += length / speed;
                                    } else {
                                        // Fallback if speed is zero or invalid
                                        estimatedTime += length / 13.89; // 50 km/h
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            } else {
                // Fallback to simple estimate if we don't have a detailed path
                estimatedTime = bestPathLength / 13.89; // 50 km/h
            }
            
            // Create time window with realistic values based on the actual path
            double initialDelay = 20.0; // seconds
            // More realistic time window: from 1.5x travel time to 2.5x travel time
            double earliness = initialDelay + (estimatedTime * 1.5); 
            double tardiness = initialDelay + (estimatedTime * 2.5);
            
            destinations.push_back(Destination(bestDestRoad, TimeWindow(earliness, tardiness)));
            
            // Store the path
            vehiclePaths.push_back(bestPath);
            
            EV << "[RSU] Found path from " << sourceRoad << " to " << bestDestRoad << std::endl;
        } else {
            // If we couldn't find a valid path, assign any unused destination
            for (size_t j = 0; j < potentialDestRoads.size(); j++) {
                if (!roadUsed[j] && potentialDestRoads[j] != sourceRoad) {
                    std::string destRoad = potentialDestRoads[j];
                    targetRoads.push_back(destRoad);
                    roadUsed[j] = true;
                    
                    // Calculate travel time based on actual road lengths and speeds
                    double estimatedTime = 0.0;
                    
                    // If we have a full path with intermediate roads
                    if (!bestPath.empty()) {
                        // Get source and target roads
                        const std::string& sourceRoad = bestPath.front();
                        const std::string& targetRoad = bestPath.back();
                        
                        // Try to get a more detailed path between source and destination
                        std::vector<std::string> detailedPath = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
                        
                        // Use the detailed path if available, otherwise use original path
                        std::vector<std::string> pathToCalculate = (detailedPath.size() > 2) ? detailedPath : bestPath;
                        
                        // Calculate time based on each road segment's length and speed
                        for (const auto& roadId : pathToCalculate) {
                            // Skip junction placeholders
                            if (roadId.find("(junction") == std::string::npos && 
                                roadId.find("(path from") == std::string::npos) {
                                
                                // Find the edge object for this road
                                for (const auto& nodePair : graph.getAdjList()) {
                                    for (const auto& edge : nodePair.second) {
                                        if (edge.getId() == roadId) {
                                            // Get actual length and maximum speed
                                            double length = edge.getLength();
                                            double maxSpeed = edge.getMaxSpeed();
                                            
                                            // Use a speed that's 80% of the maximum (typical actual speed)
                                            double speed = maxSpeed * 0.8;
                                            
                                            // Add time for this road segment
                                            if (speed > 0.0) {
                                                estimatedTime += length / speed;
                                            } else {
                                                // Fallback if speed is zero or invalid
                                                estimatedTime += length / 13.89; // 50 km/h
                                            }
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    } else {
                        // Fallback to simple estimate if we don't have a detailed path
                        estimatedTime = bestPathLength / 13.89; // 50 km/h
                    }
                    
                    // Create time window with realistic values based on the actual path
                    double initialDelay = 20.0; // seconds
                    // More realistic time window: from 1.5x travel time to 2.5x travel time
                    double earliness = initialDelay + (estimatedTime * 1.5);
                    double tardiness = initialDelay + (estimatedTime * 2.5);
                    
                    destinations.push_back(Destination(destRoad, TimeWindow(earliness, tardiness)));
                    
                    // Create empty path (will show warning)
                    vehiclePaths.push_back(std::vector<std::string>());
                    
                    EV << "[RSU] WARNING: Assigned road " << destRoad 
                       << " to vehicle with source " << sourceRoad << " without a validated path" << std::endl;
                    break;
                }
            }
        }
    }
    
    // Check if we assigned a destination to each vehicle
    if (targetRoads.size() != sourceRoads.size()) {
        EV << "[RSU] ERROR: Could not assign destinations to all vehicles" << std::endl;
        EV << "[RSU] Only assigned " << targetRoads.size() << " destinations for " << sourceRoads.size() << " vehicles" << std::endl;
        return;
    }
    
    EV << "[RSU] Successfully created destination assignments" << std::endl;
    
    // Step 5: Print the destinations with their time windows
    EV << "[RSU] Generated " << destinations.size() << " destinations with time windows:" << std::endl;
    
    int maxDestToDisplay = std::min(static_cast<size_t>(20), destinations.size());
    for (int i = 0; i < maxDestToDisplay; i++) {
        EV << "  - Destination " << i + 1 << ": Road " << destinations[i].nodeId 
           << " (Time window: " << destinations[i].timeWindow.earliness 
           << " - " << destinations[i].timeWindow.tardiness << ")" << std::endl;
    }
    
    if (destinations.size() > maxDestToDisplay) {
        EV << "  ... and " << (destinations.size() - maxDestToDisplay) << " more destinations" << std::endl;
    }
    
    // Step 6: Compute and display routes for each vehicle to its assigned destination
    EV << "\n[RSU] Computing routes for each vehicle to its assigned destination:" << std::endl;
    
    int maxRoutesToDisplay = std::min(static_cast<size_t>(10), vehicles.size());
    for (int i = 0; i < maxRoutesToDisplay; i++) {
        const std::string& sourceRoad = sourceRoads[i];
        const std::string& targetRoad = targetRoads[i];
        const std::vector<std::string>& path = vehiclePaths[i];
        
        // Get junction info for display
        std::string sourceFromJunction = "unknown";
        std::string sourceToJunction = "unknown";
        std::string targetFromJunction = "unknown";
        std::string targetToJunction = "unknown";
        
        // Look up junctions
        auto sourceIt = edgeToJunctions.find(sourceRoad);
        if (sourceIt != edgeToJunctions.end()) {
            sourceFromJunction = sourceIt->second.first;
            sourceToJunction = sourceIt->second.second;
        }
        
        auto targetIt = edgeToJunctions.find(targetRoad);
        if (targetIt != edgeToJunctions.end()) {
            targetFromJunction = targetIt->second.first;
            targetToJunction = targetIt->second.second;
        }
        
        // Calculate path length and travel time
        double pathLength = 0;
        if (!path.empty()) {
            // For the same vehicle index i, get the path length directly from bestPathLength
            // This avoids the need for iterator search that was causing type mismatch errors
            
            // Try to get accurate path length for this exact path
            for (size_t vIdx = 0; vIdx < vehiclePaths.size(); vIdx++) {
                if (&vehiclePaths[vIdx] == &path) {
                    // This is the same path object, we found the right index
                    if (vIdx < sourceRoads.size()) {
                        // Get the actual accurate path length from GraphProcessor
                        std::string srcRoad = sourceRoads[vIdx];
                        std::string tgtRoad = targetRoads[vIdx];
                        
                        // Try to get a more accurate length from the graph
                        auto srcIt = edgeToJunctions.find(srcRoad);
                        auto tgtIt = edgeToJunctions.find(tgtRoad);
                        
                        if (srcIt != edgeToJunctions.end() && tgtIt != edgeToJunctions.end()) {
                            const std::string& srcToJunction = srcIt->second.second;
                            const std::string& tgtFromJunction = tgtIt->second.first;
                            
                            // Get the accurate path length
                            double accurateLength = graphProcessor->getShortestPathLength(srcToJunction, tgtFromJunction);
                            if (accurateLength > 0 && std::isfinite(accurateLength)) {
                                // Add length of source and target roads
                                double srcLength = getEdgeLength(srcRoad);
                                double tgtLength = getEdgeLength(tgtRoad);
                                pathLength = accurateLength + srcLength + tgtLength;
                            }
                        }
                    }
                    break; // We found the matching path, no need to continue loop
                }
            }
            
            // If we still don't have a valid path length, calculate based on edges
            if (pathLength <= 0) {
                // Calculate length by summing up edge lengths
                for (const auto& edgeId : path) {
                    // Skip junction placeholders
                    if (edgeId.find("(junction") == std::string::npos && 
                        edgeId.find("(path from") == std::string::npos) {
                        double edgeLength = getEdgeLength(edgeId);
                        if (edgeLength > 0) {
                            pathLength += edgeLength;
                        } else {
                            // Use default length if not found
                            pathLength += 100.0;
                        }
                    }
                }
            }
            
            // Sanity check - if still no valid length, use a reasonable default
            if (pathLength <= 0) {
                pathLength = (path.size() - 1) * 100.0 + 50.0; // Rough estimate with some variability
            }
        }
        
        // Calculate estimated travel time based on actual road lengths and speeds
        double estimatedTime = 0.0;
        double totalLength = 0.0;
        double totalTravelTime = 0.0;
        
        // For debugging
        EV << "    Detailed travel time calculation:" << std::endl;
        
        if (!path.empty()) {
            // Get the detailed path with all intermediate segments
            const std::string& sourceRoad = path.front();
            const std::string& targetRoad = path.back();
            
            // Try to get a more detailed path between source and destination
            std::vector<std::string> detailedPath = graphProcessor->findEdgeShortestPath(sourceRoad, targetRoad);
            
            // Use the detailed path if available, otherwise use original path
            std::vector<std::string> pathToCalculate = (detailedPath.size() > 2) ? detailedPath : path;
                
            // Calculate time based on EACH road segment's length and speed
            for (const auto& roadId : pathToCalculate) {
                // Skip junction placeholders
                if (roadId.find("(junction") == std::string::npos && 
                    roadId.find("(path from") == std::string::npos) {
                    
                    bool foundEdge = false;
                    // Find the edge object for this road
                    for (const auto& nodePair : graph.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == roadId) {
                                foundEdge = true;
                                // Get actual length and maximum speed
                                double length = edge.getLength();
                                totalLength += length;
                                
                                double maxSpeed = edge.getMaxSpeed();
                                if (maxSpeed <= 0.0) {
                                    maxSpeed = 13.89; // Default to 50 km/h if no valid speed
                                }
                                
                                // Use a realistic speed (80% of max speed)
                                double speed = maxSpeed * 0.8;
                                
                                // Calculate segment travel time
                                double segmentTime = length / speed;
                                totalTravelTime += segmentTime;
                                
                                EV << "      Road " << roadId << ": length=" << length 
                                   << "m, speed=" << speed << "m/s, time=" << segmentTime << "s" << std::endl;
                                break;
                            }
                        }
                        if (foundEdge) break;
                    }
                    
                    if (!foundEdge) {
                        EV << "      Road " << roadId << ": not found in graph" << std::endl;
                    }
                }
            }
            
            // Set the final estimated time
            estimatedTime = totalTravelTime;
            
            EV << "      Total path length: " << totalLength << "m, Total travel time: " 
               << totalTravelTime << "s" << std::endl;
            
        } else if (pathLength > 0 && std::isfinite(pathLength)) {
            // Fallback to simple estimate if we don't have a detailed path
            estimatedTime = pathLength / 13.89; // 50 km/h
            EV << "      Using fallback calculation: length=" << pathLength 
               << "m / speed=13.89m/s = " << estimatedTime << "s" << std::endl;
        }
        
        // Update time window with final accurate travel time calculation
        double initialDelay = 20.0; // seconds
        double newEarliness = initialDelay + (estimatedTime * 1.5);
        double newTardiness = initialDelay + (estimatedTime * 2.5);
        
        // Update the destination time window with new values
        destinations[i].timeWindow.earliness = newEarliness;
        destinations[i].timeWindow.tardiness = newTardiness;
        
        // Print route information
        EV << "  - Vehicle " << vehicles[i].id << " route:" << std::endl;
        EV << "    From: Road " << sourceRoad << " (Junction " << sourceFromJunction 
           << " -> " << sourceToJunction << ")" << std::endl;
        EV << "    To: Road " << targetRoad << " (Junction " << targetFromJunction 
           << " -> " << targetToJunction << ")" << std::endl;
        EV << "    Path length: " << (std::isfinite(pathLength) ? std::to_string(pathLength) : "unknown") << " m" << std::endl;
        EV << "    Estimated travel time: " << (std::isfinite(estimatedTime) ? std::to_string(estimatedTime) : "unknown") << " s" << std::endl;
        EV << "    Time window: " << destinations[i].timeWindow.earliness 
           << " - " << destinations[i].timeWindow.tardiness << std::endl;
        
        // Print the path (showing all road IDs in the path, including intermediate roads)
        if (!path.empty()) {
            EV << "    Path: ";
            
            // Always try to get a more detailed path between source and destination roads
            const std::string& sourceRoad = path.front();
            const std::string& targetRoad = path.back();
            
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
                    intermediateRoads = path;
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
            EV << "    WARNING: No path found!" << std::endl;
        }
        
        EV << std::endl;
    }
    
    if (vehicles.size() > maxRoutesToDisplay) {
        EV << "  ... and routes for " << (vehicles.size() - maxRoutesToDisplay) << " more vehicles" << std::endl;
    }
    
    EV << "==============================================================\n" << std::endl;
    
    // Store the destinations in the vehicleDataMap for later reference
    for (size_t i = 0; i < std::min(vehicles.size(), destinations.size()); i++) {
        LAddress::L2Type vehicleId = i; // Simplified - in a real system, this would be the actual vehicle ID
        vehicleDataMap[vehicleId].assignedDestination = destinations[i];
    }
}

double RSUControlApp::getEdgeLength(const std::string& edgeId) const {
    // Search for the edge in the graph and return its length
    if (graphProcessor) {
        const Graph& graph = graphProcessor->getGraph();
        
        // Check in all nodes' adjacency lists
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    return edge.getLength();
                }
            }
        }
    }
    
    return 100.0;
}
