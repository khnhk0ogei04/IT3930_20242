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

Define_Module(RSUControlApp);

void RSUControlApp::initialize(int stage) {
    TraCIDemoRSU11p::initialize(stage);
    EV << "[RSU] Initialize called with stage " << stage << std::endl;
    if (stage == 0) {
        vehicleDataMap.clear();

        statusCheckMsg = new cMessage("checkStatus");
        scheduleAt(simTime() + 1.0, statusCheckMsg);

        // Initialize the road network from configuration
        networkFilePath = par("netFile").stdstringValue();
        EV << "[RSU] Network file path parameter: " << networkFilePath << std::endl;
        
        if (!networkFilePath.empty()) {
            // Initialize processors
            xmlProcessor.reset(new XMLProcessor());
            
            EV << "[RSU] Attempting to load network file from: " << networkFilePath << std::endl;
            
            if (xmlProcessor->loadNetworkFile(networkFilePath)) {
                EV << "[RSU] Successfully loaded road network from " << networkFilePath << std::endl;
                
                // Initialize GraphProcessor
                graphProcessor.reset(new GraphProcessor(xmlProcessor->getGraph()));
                EV << "[RSU] Successfully initialized GraphProcessor" << std::endl;
                
                // Initialize TaskGenerator
                taskGenerator.reset(new TaskGenerator(*graphProcessor));
                EV << "[RSU] Successfully initialized TaskGenerator" << std::endl;
                
                // Print network information using the new methods
                printRoadNetworkInfo();
                printNodeInfo();
                
                // Test TaskGenerator initial functionality
                testTaskGenerator();
            } else {
                EV << "[RSU] Failed to load road network from " << networkFilePath << std::endl;
                
                // Try several different path variations
                std::vector<std::string> alternativePaths = {
                    "testw8.net.xml",
                    "./testw8.net.xml",
                    "../testw8.net.xml",
                    "simulations/veins/testw8.net.xml",
                    "../../simulations/veins/testw8.net.xml",
                    "erlangen.net.xml",
                    "./erlangen.net.xml",
                    "../erlangen.net.xml",
                    "simulations/veins/erlangen.net.xml",
                    "../../simulations/veins/erlangen.net.xml"
                };
                
                EV << "[RSU] Current working directory information:" << std::endl;
                // Try to get current working directory (this won't work in all environments)
                char cwd[FILENAME_MAX];
                if (GetCurrentDir(cwd, sizeof(cwd)) != NULL) {
                    EV << "[RSU] Current working directory: " << cwd << std::endl;
                } else {
                    EV << "[RSU] Could not determine current working directory" << std::endl;
                }
                
                bool loaded = false;
                for (const auto& path : alternativePaths) {
                    EV << "[RSU] Trying alternative path: " << path << std::endl;
                    if (xmlProcessor->loadNetworkFile(path)) {
                        EV << "[RSU] Successfully loaded with path: " << path << std::endl;
                        graphProcessor.reset(new GraphProcessor(xmlProcessor->getGraph()));
                        taskGenerator.reset(new TaskGenerator(*graphProcessor));
                        
                        // Get all roads
                        std::vector<std::string> allRoads = xmlProcessor->getAllRoads();
                        EV << "[RSU] Total number of roads: " << allRoads.size() << std::endl;
                        
                        // Print nodes information
                        const Graph& graph = xmlProcessor->getGraph();
                        const auto& nodes = graph.getNodes();
                        EV << "\n[RSU] ========== NODE INFORMATION ==========\n" << std::endl;
                        EV << "[RSU] Total number of nodes: " << nodes.size() << std::endl;
                        
                        // Print first 20 nodes or all if less than 20
                        int nodeCount = 0;
                        for (const auto& nodePair : nodes) {
                            const Node& node = nodePair.second;
                            EV << "[RSU] Node: id=\"" << node.getId() 
                               << "\" x=\"" << node.getX() 
                               << "\" y=\"" << node.getY() 
                               << "\"" << std::endl;
                            nodeCount++;
                        }
                        
                        // Print road information with lanes
                        EV << "\n[RSU] ========== ROAD INFORMATION ==========\n" << std::endl;
                        
                        // Print first 20 roads with detailed attributes
                        int roadCount = 0;
                        for (const auto& roadId : allRoads) {
                            // Tìm edge trong graph để lấy thuộc tính đầy đủ
                            for (const auto& nodePair : graph.getAdjList()) {
                                for (const auto& edge : nodePair.second) {
                                    if (edge.getId() == roadId) {
                                        // Print edge information in XML format
                                        EV << "[RSU] <edge id=\"" << edge.getId() 
                                           << "\" from=\"" << edge.getFrom() 
                                           << "\" to=\"" << edge.getTo() 
                                           << "\" priority=\"-1\""
                                           << " length=\"" << edge.getLength() << "\">"
                                           << std::endl;
                                        
                                        // Get the lanes for this edge directly from the edge object
                                        const auto& lanes = edge.getLanes();
                                        
                                        // Print lane count
                                        EV << "[RSU]   Lane count: " << lanes.size() << std::endl;
                                        EV << "In RSUControlApp: Edge " << edge.getId() << " has " << lanes.size() << " lanes" << std::endl;
                                        
                                        // Print lane details
                                        for (const auto& lane : lanes) {
                                            EV << "[RSU]   <lane id=\"" << lane.id 
                                               << "\" index=\"" << lane.index 
                                               << "\" speed=\"" << lane.speed 
                                               << "\" length=\"" << lane.length 
                                               << "\" shape=\"" << lane.shape << "\"/>" 
                                               << std::endl;
                                        }
                                        
                                        EV << "[RSU] </edge>" << std::endl;
                                        break;
                                    }
                                }
                            }
                            
                            roadCount++;
                        }
                        
                        loaded = true;
                        break;
                    }
                }
            }
        } else {
            EV << "[RSU] No network file path specified in configuration" << std::endl;
        }
    }
}

void RSUControlApp::onWSM(BaseFrame1609_4* wsm) {
    auto* enc = wsm->getEncapsulatedPacket();
    auto* msg = dynamic_cast<TraCIDemo11pMessage*>(enc);
    if (!msg) return;

    LAddress::L2Type senderId = msg->getSenderAddress();
    std::string data = msg->getDemoData();
    
    // Update vehicle last message time
    vehicleDataMap[senderId].lastMessageTime = simTime();
    
    // Process vehicle message
    handleVehicleMessage(data, senderId);
}

void RSUControlApp::handleVehicleMessage(const std::string& message, LAddress::L2Type vehicleId) {
    // Check if the XML processor is initialized and network is loaded
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] XML processor not initialized or network not loaded" << std::endl;
        std::vector<std::string> errorMsg = {"ERROR: Network not loaded"};
        sendRoadListMessage(vehicleId, errorMsg);
        return;
    }

    // Debug message received
    EV << "[RSU] Received message from vehicle: '" << message << "'" << std::endl;

    // Handle request for all roads
    if (message == "GET_ALL_ROADS") {
        auto roads = xmlProcessor->getAllRoads();
        EV << "[RSU] Responding with " << roads.size() << " roads for GET_ALL_ROADS request" << std::endl;
        sendRoadListMessage(vehicleId, roads);
    }
    // Handle request for accessible roads
    else if (message.find("GET_ACCESSIBLE_ROADS:") == 0) {
        std::string roadId = message.substr(21); // Skip "GET_ACCESSIBLE_ROADS:"
        
        auto accessibleRoads = xmlProcessor->getAccessibleRoads(roadId);
        
        // In kết quả số đường tìm thấy và danh sách các đường đó trên một hàng
        EV << "[RSU] Found " << accessibleRoads.size() << " accessible roads from '" << roadId << "': ";
        
        // In danh sách các đường có thể truy cập trên cùng một hàng
        for (size_t i = 0; i < accessibleRoads.size(); ++i) {
            EV << accessibleRoads[i];
            if (i < accessibleRoads.size() - 1) {
                EV << ", ";
            }
        }
        EV << std::endl;
        
        // Also log detailed information about the source road for debugging
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
        std::string roadId = message.substr(19); // Skip "GET_INCOMING_ROADS:"
        EV << "[RSU] Looking for incoming roads to '" << roadId << "'" << std::endl;
        auto incomingRoads = xmlProcessor->getIncomingRoads(roadId);
        EV << "[RSU] Found " << incomingRoads.size() << " incoming roads to '" << roadId << "'" << std::endl;
        sendRoadListMessage(vehicleId, incomingRoads);
    }
    // Handle request for road attributes
    else if (message.find("GET_ROAD_ATTRIBUTES:") == 0) {
        std::string roadId = message.substr(20); // Skip "GET_ROAD_ATTRIBUTES:"
        
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
        
        // Convert attributes to string format
        std::string attrStr = roadId + ":";
        for (const auto& attr : attrs) {
            attrStr += attr.first + "=" + attr.second + ";";
        }
        
        // Create and send response
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
        std::string params = message.substr(19); // Skip "FIND_SHORTEST_PATH:"
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
                // No path exists
                response.push_back("NO_PATH_EXISTS");
            } else {
                // Add path length as first element
                response.push_back("LENGTH:" + std::to_string(pathLength));
                
                // Add all road segments to the response
                for (const auto& roadId : path) {
                    response.push_back(roadId);
                }
            }
            
            // Send response back to the vehicle
            sendRoadListMessage(vehicleId, response);
            
            // Log path information
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
            
            // Log each destination for debugging
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
        
        std::string params = message.substr(24); // Skip "EXISTS_VALID_ASSIGNMENT:"
        size_t separatorPos = params.find('|');
        
        if (separatorPos != std::string::npos) {
            std::string sourcesStr = params.substr(0, separatorPos);
            std::string destsStr = params.substr(separatorPos + 1);
            
            // Parse sources and destinations
            std::vector<std::string> sources;
            std::vector<std::string> dests;
            
            size_t pos = 0;
            while ((pos = sourcesStr.find(',')) != std::string::npos) {
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
            
            // Log sources and destinations for debugging
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
            EV << std::endl;
            
            bool validAssignment = taskGenerator->existsValidAssignment(sources, dests);
            EV << "[RSU] Valid assignment: " << (validAssignment ? "TRUE" : "FALSE") << std::endl;
            
            sendRoadListMessage(vehicleId, {validAssignment ? "TRUE" : "FALSE"});
        } else {
            EV << "[RSU] ERROR: Malformed valid assignment request" << std::endl;
        }
    }
}

void RSUControlApp::sendRoadListMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& roadList) {
    // Compose message with the list of roads
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
        // Clean up vehicle data that hasn't been updated in a while
        cleanupVehicleData();

        // Print network file status
        if (!networkFilePath.empty()) {
            EV << "Current network file: " << networkFilePath << std::endl;
            if (xmlProcessor && xmlProcessor->isNetworkLoaded()) {
                EV << "Network successfully loaded with "
                         << xmlProcessor->getGraph().getNodeCount() << " nodes and "
                         << xmlProcessor->getGraph().getEdgeCount() << " edges" << std::endl;
                
                // Print first few roads for debugging
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
        
        // Schedule next check
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
    // Cleanup old vehicle data
    simtime_t now = simTime();
    const double CLEANUP_TIMEOUT = 10.0; // 10 seconds
    
    for (auto it = vehicleDataMap.begin(); it != vehicleDataMap.end();) {
        if (now - it->second.lastMessageTime > CLEANUP_TIMEOUT) {
            it = vehicleDataMap.erase(it);
        } else {
            ++it;
        }
    }
}

void RSUControlApp::onWSA(DemoServiceAdvertisment* wsa) {
    // Phương thức này phải được triển khai nhưng không cần thực hiện gì trong trường hợp này
    // Chỉ gọi phương thức cơ sở
    TraCIDemoRSU11p::onWSA(wsa);
}

// Implement the method to get all roads
std::vector<std::string> RSUControlApp::getAllRoads() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        return std::vector<std::string>();
    }
    return xmlProcessor->getAllRoads();
}

// Implement the method to get all nodes
std::vector<std::string> RSUControlApp::getAllNodes() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        return std::vector<std::string>();
    }
    
    std::vector<std::string> result;
    const Graph& graph = xmlProcessor->getGraph();
    const auto& nodes = graph.getNodes();
    
    for (const auto& nodePair : nodes) {
        result.push_back(nodePair.first);
    }
    
    return result;
}

// Implement the method to find the shortest path
std::vector<std::string> RSUControlApp::findShortestPath(const std::string& sourceId, const std::string& targetId) const {
    if (!graphProcessor) {
        return std::vector<std::string>();
    }
    
    return graphProcessor->findShortestPath(sourceId, targetId);
}

// Implement the method to get the length of the shortest path
double RSUControlApp::getShortestPathLength(const std::string& sourceId, const std::string& targetId) const {
    if (!graphProcessor) {
        return -1.0;
    }
    
    return graphProcessor->getShortestPathLength(sourceId, targetId);
}

// Implement method to print road information
void RSUControlApp::printRoadNetworkInfo() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] No road network loaded" << std::endl;
        return;
    }
    
    const std::vector<std::string> allRoads = getAllRoads();
    EV << "\n[RSU] ========== ROAD NETWORK INFORMATION ==========\n" << std::endl;
    EV << "[RSU] Total number of roads: " << allRoads.size() << std::endl;
    
    // Print first 20 roads with detailed info
    int maxRoadsToShow = std::min(static_cast<int>(allRoads.size()), 20);
    for (int i = 0; i < maxRoadsToShow; i++) {
        const std::string& roadId = allRoads[i];
        EV << "[RSU] Road ID: " << roadId << std::endl;
        
        // Find the edge in the graph to get connectivity info
        const Graph& graph = xmlProcessor->getGraph();
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == roadId) {
                    EV << "  From: " << edge.getFrom() << " To: " << edge.getTo() << std::endl;
                    EV << "  Length: " << edge.getLength() << std::endl;
                    
                    // Hiển thị thông tin làn đường
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
        EV << "[RSU] ... and " << (allRoads.size() - maxRoadsToShow) << " more roads" << std::endl;
    }
    
    // TÌM VÀ IN ĐƯỜNG ĐI NGẮN NHẤT NGAY SAU KHI HIỂN THỊ DANH SÁCH ĐƯỜNG
    EV << "\n[RSU] ========== SHORTEST PATH TESTS ==========\n" << std::endl;
    
    // Đảm bảo chúng ta có GraphProcessor (sử dụng qua kết tập)
    if (!graphProcessor) {
        EV << "[RSU] GraphProcessor not initialized, cannot perform path finding tests" << std::endl;
        return;
    }
    
    // Chọn một số đường để kiểm tra đường đi
    if (allRoads.size() >= 2) {
        // Kiểm tra đường đi đầu tiên: từ đường đầu tiên đến đường cuối cùng
        std::string sourceRoad = "1024";
        std::string targetRoad = "1959";
        
        EV << "[RSU] Path Test 1: Finding path from " << sourceRoad << " to " << targetRoad << std::endl;
        std::vector<std::string> path = findShortestPath(sourceRoad, targetRoad);
        double pathLength = getShortestPathLength(sourceRoad, targetRoad);
        
        if (!path.empty() && pathLength > 0) {
            EV << "[RSU] SUCCESS: Path found with length " << pathLength << " units" << std::endl;
            // In danh sách đường đi trên cùng một dòng
            EV << "[RSU] Path segments (" << path.size() << "): ";
            for (size_t i = 0; i < path.size(); i++) {
                EV << path[i];
                if (i < path.size() - 1) {
                    EV << " -> "; // Sử dụng mũi tên để phân tách
                }
            }
            EV << std::endl;
        } else {
            EV << "[RSU] NO PATH found between " << sourceRoad << " and " << targetRoad << std::endl;
            
            // Nếu không tìm thấy đường đi giữa các đường, thử tìm đường đi giữa các nút
            const Graph& graph = xmlProcessor->getGraph();
            const auto& nodes = graph.getNodes();
            if (nodes.size() >= 2) {
                auto it = nodes.begin();
                std::string sourceNode = it->first;
                std::advance(it, nodes.size() - 1);
                std::string targetNode = it->first;
                
                EV << "[RSU] Trying path between nodes instead: " << sourceNode << " to " << targetNode << std::endl;
                path = findShortestPath(sourceNode, targetNode);
                pathLength = getShortestPathLength(sourceNode, targetNode);
                
                if (!path.empty() && pathLength > 0) {
                    EV << "[RSU] SUCCESS: Node path found with length " << pathLength << " units" << std::endl;
                    // In danh sách đường đi trên cùng một dòng
                    EV << "[RSU] Path segments (" << path.size() << "): ";
                    for (size_t i = 0; i < path.size(); i++) {
                        EV << path[i];
                        if (i < path.size() - 1) {
                            EV << " -> "; // Sử dụng mũi tên để phân tách
                        }
                    }
                    EV << std::endl;
                } else {
                    EV << "[RSU] NO PATH found between nodes " << sourceNode << " and " << targetNode << std::endl;
                }
            }
        }
        
        // Kiểm tra đường đi thứ hai: từ đường đầu tiên đến đường ở giữa
        if (allRoads.size() >= 3) {
            std::string middleRoad = allRoads[allRoads.size() / 2];
            
            EV << "\n[RSU] Path Test 2: Finding path from " << sourceRoad << " to " << middleRoad << std::endl;
            path = findShortestPath(sourceRoad, middleRoad);
            pathLength = getShortestPathLength(sourceRoad, middleRoad);
            
            if (!path.empty() && pathLength > 0) {
                EV << "[RSU] SUCCESS: Path found with length " << pathLength << " units" << std::endl;
                // In danh sách đường đi trên cùng một dòng
                EV << "[RSU] Path segments (" << path.size() << "): ";
                for (size_t i = 0; i < path.size(); i++) {
                    EV << path[i];
                    if (i < path.size() - 1) {
                        EV << " -> "; // Sử dụng mũi tên để phân tách
                    }
                }
                EV << std::endl;
            } else {
                EV << "[RSU] NO PATH found between " << sourceRoad << " and " << middleRoad << std::endl;
            }
        }
    } else {
        EV << "[RSU] Not enough roads to perform path finding tests" << std::endl;
    }
    
    // Kiểm tra tương thích đồ thị
    EV << "\n[RSU] Graph Check: " << std::endl;
    const Graph& graph = xmlProcessor->getGraph();
    EV << "  - Node count: " << graph.getNodeCount() << std::endl;
    EV << "  - Edge count: " << graph.getEdgeCount() << std::endl;
    
    EV << "\n[RSU] ==========================================\n" << std::endl;
}

// Implement method to print node information
void RSUControlApp::printNodeInfo() const {
    if (!xmlProcessor || !xmlProcessor->isNetworkLoaded()) {
        EV << "[RSU] No road network loaded" << std::endl;
        return;
    }
    
    const Graph& graph = xmlProcessor->getGraph();
    const auto& nodes = graph.getNodes();
    
    EV << "\n[RSU] ========== NODE INFORMATION ==========\n" << std::endl;
    EV << "[RSU] Total number of nodes: " << nodes.size() << std::endl;
    
    // Print first 20 nodes
    int count = 0;
    for (const auto& nodePair : nodes) {
        if (count >= 20) break;
        
        const Node& node = nodePair.second;
        EV << "[RSU] Node ID: " << node.getId() 
           << " X: " << node.getX() 
           << " Y: " << node.getY() << std::endl;
        count++;
    }
    
    if (nodes.size() > 20) {
        EV << "[RSU] ... and " << (nodes.size() - 20) << " more nodes" << std::endl;
    }
    
    EV << "\n[RSU] =======================================\n" << std::endl;
}

// Triển khai hàm test TaskGenerator
void RSUControlApp::testTaskGenerator() {
    if (!taskGenerator) {
        EV << "[RSU] ERROR: Cannot test TaskGenerator - not initialized" << std::endl;
        return;
    }
    
    EV << "\n[RSU] ========== TASK GENERATOR TEST ==========\n" << std::endl;
    
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
        if (roads.size() >= 4) {
            std::string sourceId = "1153";  // Using specific node ID
            std::string targetId = "473";  // Using specific node ID
            
            EV << "\n[RSU] Test 2: Finding 2 shortest paths from " << sourceId << " to " << targetId << std::endl;
            
            auto paths = taskGenerator->findKPaths(sourceId, targetId, 4);
            
            EV << "[RSU] Found " << paths.size() << " paths:" << std::endl;
            for (size_t i = 0; i < paths.size(); i++) {
                const auto& path = paths[i];
                
                // Tính độ dài đường đi
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
    EV << "\n[RSU] Test 3: Testing valid assignment" << std::endl;
    
    std::vector<std::string> sources = {"1024", "213", "337"};
    std::vector<std::string> targets = {"1985", "853", "205"};
    
    bool validAssignment = taskGenerator->existsValidAssignment(sources, targets);
    
    EV << "[RSU] Valid assignment exists: " << (validAssignment ? "YES" : "NO") << std::endl;
    
    EV << "\n[RSU] =========================================\n" << std::endl;
}
