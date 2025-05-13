#include "RSUControlApp.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <sstream>
#include <iostream>
#include <cstring>

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
    // Handle K paths request
    else if (message.find("FIND_K_PATHS:") == 0 && graphProcessor && taskGenerator) {
        EV << "[RSU] DEBUG: Received FIND_K_PATHS request: " << message << std::endl;
        
        // Extract source, target, and k
        std::string params = message.substr(13); // Skip "FIND_K_PATHS:"
        size_t firstComma = params.find(',');
        size_t secondComma = params.find(',', firstComma + 1);
        
        if (firstComma != std::string::npos && secondComma != std::string::npos) {
            std::string sourceId = params.substr(0, firstComma);
            std::string targetId = params.substr(firstComma + 1, secondComma - firstComma - 1);
            int k = std::stoi(params.substr(secondComma + 1));
            
            EV << "[RSU] Finding " << k << " paths from " << sourceId << " to " << targetId << std::endl;
            
            auto paths = taskGenerator->findKPaths(sourceId, targetId, k);
            
            EV << "[RSU] Found " << paths.size() << " paths" << std::endl;
            
            // Flatten the paths into a single list with separators
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

    const Graph& graph = graphProcessor->getGraph();
    bool sourceEdgeFound = false;
    bool targetEdgeFound = false;
    std::string sourceNodeId, targetNodeId;
    for (const auto& nodePair : graph.getAdjList()) {
        for (const auto& edge : nodePair.second) {
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
    
    // Print the path
    EV << "[RSU] Đường đi: ";
    for (size_t i = 0; i < edgePath.size(); i++) {
        EV << edgePath[i];
        if (i < edgePath.size() - 1) {
            EV << " -> ";
        }
    }
    EV << std::endl;
}
