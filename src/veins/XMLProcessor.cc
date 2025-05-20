#include "XMLProcessor.h"
#include <algorithm>
#include <iostream>

namespace veins {

// New static method to extract roads from XML
std::vector<Edge> XMLProcessor::getRoadsFromXml(const std::string& filePath) {
    std::vector<Edge> roads;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not open XML file \"" << filePath << "\".\n";
        return roads;
    }
    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        std::cerr << "Error: XML file missing root element <net>.\n";
        return roads;
    }

    std::cout << "XMLProcessor: Parsing road network from " << filePath << std::endl;
    int edgeCount = 0;
    int laneCount = 0;
    int edgesWithNoLanes = 0;

    for (tinyxml2::XMLElement* edgeElem = netElement->FirstChildElement("edge");
        edgeElem != nullptr;
        edgeElem = edgeElem->NextSiblingElement("edge")) {
        
        const char* edgeId = edgeElem->Attribute("id");
        const char* fromNode = edgeElem->Attribute("from");
        const char* toNode = edgeElem->Attribute("to");
        
        if (edgeId && fromNode && toNode) {
            // Count lanes for this edge
            int edgeLaneCount = 0;
            std::vector<Lane> lanes;
            
            // Iterate through lanes and extract their attributes
            for (tinyxml2::XMLElement* laneElem = edgeElem->FirstChildElement("lane");
                 laneElem != nullptr;
                 laneElem = laneElem->NextSiblingElement("lane")) {
                
                const char* laneId = laneElem->Attribute("id");
                const char* laneIndex = laneElem->Attribute("index");
                const char* laneSpeed = laneElem->Attribute("speed");
                const char* laneLength = laneElem->Attribute("length");
                const char* laneShape = laneElem->Attribute("shape");
                
                if (laneId && laneIndex && laneSpeed && laneLength) {
                    Lane lane;
                    lane.id = laneId;
                    lane.index = atoi(laneIndex);
                    lane.speed = atof(laneSpeed);
                    lane.length = atof(laneLength);
                    if (laneShape) {
                        lane.shape = laneShape;
                    }
                    
                    lanes.push_back(lane);
                    laneCount++;
                    edgeLaneCount++;
                    
                    std::cout << "  Added lane " << laneId << " to edge " << edgeId 
                              << " (index: " << lane.index << ", speed: " << lane.speed 
                              << ", length: " << lane.length << ")" << std::endl;
                }
            }
            
            std::cout << "XMLProcessor: Edge " << edgeId << " has " << edgeLaneCount << " lanes" << std::endl;
            if (edgeLaneCount == 0) {
                edgesWithNoLanes++;
            }

            // Create the edge with its lanes
            Edge edge(edgeId, fromNode, toNode, lanes);
            
            if (edge.getLanes().size() != edgeLaneCount) {
                std::cout << "WARNING: Edge " << edgeId << " has " << edgeLaneCount 
                          << " lanes in XML but " << edge.getLanes().size() 
                          << " lanes after Edge construction" << std::endl;
            }
            roads.push_back(edge);
            edgeCount++;
        }
    }
    
    std::cout << "XMLProcessor: Parsed " << edgeCount << " edges and " << laneCount << " lanes in total" << std::endl;
    std::cout << "XMLProcessor: Found " << edgesWithNoLanes << " edges with no lanes" << std::endl;
    
    return roads;
}

// Implement only the parseNetXml method
bool XMLProcessor::parseNetXml(const std::string& filePath, Graph& graph) {
    // Use the global parseNetXml function defined in Graph.cc
    return ::parseNetXml(filePath, graph);
}

XMLProcessor::XMLProcessor() : networkLoaded(false) {
    vehicleData.clear();
}

bool XMLProcessor::loadNetworkFile(const std::string& filename) {
    // Clear any existing data
    incomingRoadsMap.clear();
    roadAttributes.clear();
    networkLoaded = false;
    vehicleData.clear();
    std::cout << "XMLProcessor: Attempting to load network file from: " << filename << std::endl;

    // Use the existing parseNetXml to load the graph
    bool success = parseNetXml(filename, roadNetwork);
    if (!success) {
        std::cout << "Failed to load network from " << filename << std::endl;
        return false;
    }

    // Process the XML file to extract road attributes
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filename.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        std::cout << "Error: Could not open XML file \"" << filename << "\" (Error code: "
                  << static_cast<int>(result) << ")." << std::endl;
        return false;
    }

    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        std::cout << "Error: XML file missing root element <net>." << std::endl;
        return false;
    }

    // Count edges and lanes for debugging
    int edgeCount = 0;
    int laneCount = 0;

    // Process all edge elements to extract attributes
    for (tinyxml2::XMLElement* edgeElem = netElement->FirstChildElement("edge");
         edgeElem != nullptr;
         edgeElem = edgeElem->NextSiblingElement("edge")) {

        const char* idAttr = edgeElem->Attribute("id");
        if (!idAttr) continue;

        edgeCount++;
        std::string edgeId(idAttr);
        roadAttributes[edgeId] = extractAttributes(edgeElem);

        // Detailed debug information for the first 5 edges
        if (edgeCount <= 5) {
            std::cout << "DEBUG: Edge " << edgeId;
            const char* fromAttr = edgeElem->Attribute("from");
            const char* toAttr = edgeElem->Attribute("to");
            if (fromAttr && toAttr) {
                std::cout << " connects from " << fromAttr << " to " << toAttr;
            }
            std::cout << std::endl;
        }

        // Process lanes for this edge
        int edgeLaneCount = 0;

        for (tinyxml2::XMLElement* laneElem = edgeElem->FirstChildElement("lane");
             laneElem != nullptr;
             laneElem = laneElem->NextSiblingElement("lane")) {

            const char* laneId = laneElem->Attribute("id");
            const char* laneIndex = laneElem->Attribute("index");
            const char* laneSpeed = laneElem->Attribute("speed");
            const char* laneLength = laneElem->Attribute("length");

            if (laneId && laneIndex && laneSpeed && laneLength) {
                std::string prefix = "lane" + std::string(laneIndex) + "_";
                roadAttributes[edgeId][prefix + "id"] = laneId;
                roadAttributes[edgeId][prefix + "speed"] = laneSpeed;
                roadAttributes[edgeId][prefix + "length"] = laneLength;
                laneCount++;
                edgeLaneCount++;
            }
        }
        
        // Add lane count to attributes
        if (edgeLaneCount > 0) {
            roadAttributes[edgeId]["laneCount"] = std::to_string(edgeLaneCount);
        }
        
        std::cout << "XML Edge: " << edgeId << " has " << edgeLaneCount << " lanes in XML" << std::endl;
    }

    std::cout << "XML Processing found " << edgeCount << " edges and " << laneCount << " lanes" << std::endl;

    // Build the map of incoming roads
    buildIncomingRoadsMap();

    // Verify the graph structure
    std::cout << "\nDEBUG: Verifying graph structure..." << std::endl;
    const auto& adjList = roadNetwork.getAdjList();
    int nodeCount = 0, connectedNodeCount = 0;
    for (const auto& nodePair : adjList) {
        nodeCount++;
        if (!nodePair.second.empty()) {
            connectedNodeCount++;
            if (nodeCount <= 5) {
                std::cout << "DEBUG: Node " << nodePair.first << " connects to ";
                for (const auto& edge : nodePair.second) {
                    std::cout << edge.getTo() << " via edge " << edge.getId() << ", ";
                }
                std::cout << std::endl;
            }
        } else if (nodeCount <= 5) {
            std::cout << "DEBUG: Node " << nodePair.first << " has no outgoing connections" << std::endl;
        }
    }
    std::cout << "DEBUG: Graph has " << nodeCount << " nodes, " << connectedNodeCount 
              << " have outgoing connections" << std::endl;

    // Verify lanes in the graph
    int graphLaneCount = 0;
    for (const auto& nodePair : adjList) {
        for (const auto& edge : nodePair.second) {
            const auto& lanes = edge.getLanes();
            graphLaneCount += lanes.size();
            if (graphLaneCount <= 10) {
                std::cout << "Graph Edge: " << edge.getId() << " has " << lanes.size() 
                          << " lanes, connects from " << nodePair.first << " to " << edge.getTo() << std::endl;
            }
        }
    }
    
    std::cout << "Graph has " << graphLaneCount << " total lanes" << std::endl;

    networkLoaded = true;
    std::cout << "Loaded network from " << filename << std::endl;
    std::cout << "Network contains " << roadNetwork.getNodeCount() << " nodes and "
              << roadNetwork.getEdgeCount() << " edges" << std::endl;

    return true;
}

std::vector<std::string> XMLProcessor::getAllRoads() const {
    std::vector<std::string> result;
    if (!networkLoaded) return result;
    
    // Extract all edge IDs from the road network
    const auto& adjList = roadNetwork.getAdjList();
    for (const auto& pair : adjList) {
        const auto& edges = pair.second;
        for (const auto& edge : edges) {
            result.push_back(edge.getId());
        }
    }
    
    // Remove duplicates
    std::sort(result.begin(), result.end());
    auto last = std::unique(result.begin(), result.end());
    result.erase(last, result.end());
    
    return result;
}

std::vector<std::string> XMLProcessor::getAccessibleRoads(const std::string& roadId) const {
    std::vector<std::string> result;
    if (!networkLoaded) return result;
    
    // Find the road in the graph
    std::string toNodeId;
    
    // Search for the road and get its target node
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == roadId) {
                toNodeId = edge.getTo();
                
                // Find all roads starting from the target node
                auto it = roadNetwork.getAdjList().find(toNodeId);
                if (it != roadNetwork.getAdjList().end()) {
                    for (const auto& outEdge : it->second) {
                        if (outEdge.getId() != roadId) {
                            result.push_back(outEdge.getId());
                        }
                    }
                }
                
                return result;  // Return immediately after finding and processing
            }
        }
    }
    
    return result;
}

std::vector<std::string> XMLProcessor::getIncomingRoads(const std::string& roadId) const {
    if (!networkLoaded) return std::vector<std::string>();
    
    auto it = incomingRoadsMap.find(roadId);
    if (it != incomingRoadsMap.end()) {
        return it->second;
    }
    
    return std::vector<std::string>();
}

std::map<std::string, std::string> XMLProcessor::getRoadAttributes(const std::string& roadId) const {
    if (!networkLoaded) return std::map<std::string, std::string>();
    
    auto it = roadAttributes.find(roadId);
    if (it != roadAttributes.end()) {
        return it->second;
    }
    
    return std::map<std::string, std::string>();
}

std::map<std::string, std::string> XMLProcessor::extractAttributes(const tinyxml2::XMLElement* element) const {
    std::map<std::string, std::string> attrs;
    if (!element) return attrs;
    
    const tinyxml2::XMLAttribute* attr = element->FirstAttribute();
    while (attr) {
        attrs[attr->Name()] = attr->Value();
        attr = attr->Next();
    }
    
    return attrs;
}

void XMLProcessor::buildIncomingRoadsMap() {
    if (!networkLoaded) return;
    
    incomingRoadsMap.clear();
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            const std::string& targetNodeId = edge.getTo();
            const std::string& edgeId = edge.getId();
            
            // Find all outgoing edges from the target node
            auto it = roadNetwork.getAdjList().find(targetNodeId);
            if (it != roadNetwork.getAdjList().end()) {
                const auto& outEdges = it->second;
                for (const auto& outEdge : outEdges) {
                    incomingRoadsMap[outEdge.getId()].push_back(edgeId);
                }
            }
        }
    }
}

bool XMLProcessor::loadRouteFile(const std::string& filePath) {
    vehicleData.clear();

    std::cout << "XMLProcessor: Attempting to load route file from: " << filePath << std::endl;

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        std::cout << "Error: Could not open XML file \"" << filePath << "\" (Error code: "
                  << static_cast<int>(result) << ")." << std::endl;
        return false;
    }

    tinyxml2::XMLElement* routesElement = doc.FirstChildElement("routes");
    if (!routesElement) {
        std::cout << "Error: XML file missing root element <routes>." << std::endl;
        return false;
    }

    // Process all trip elements to extract vehicle information
    for (tinyxml2::XMLElement* tripElem = routesElement->FirstChildElement("trip");
         tripElem != nullptr;
         tripElem = tripElem->NextSiblingElement("trip")) {

        VehicleInfo vehicle;
        vehicle.id = tripElem->Attribute("id");

        const char* departAttr = tripElem->Attribute("depart");
        if (departAttr) {
            vehicle.depart = std::stod(departAttr);
        } else {
            vehicle.depart = 0.0; // Default if not specified
        }

        vehicle.from = tripElem->Attribute("from");
        vehicle.to = tripElem->Attribute("to");

        const char* viaAttr = tripElem->Attribute("via");
        if (viaAttr) {
            vehicle.via = viaAttr;
        } else {
            vehicle.via = ""; // Empty if not specified
        }

        vehicleData.push_back(vehicle);
    }

    std::cout << "XMLProcessor: Loaded " << vehicleData.size() << " vehicles from " << filePath << std::endl;
    return true;
}

std::vector<VehicleInfo> XMLProcessor::getVehicles() const { return vehicleData; }

// New static method to extract junctions from XML
std::unordered_map<std::string, Node> XMLProcessor::getJunctionsFromXml(const std::string& filePath) {
    std::unordered_map<std::string, Node> junctions;

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error: Could not open XML file \"" << filePath << "\".\n";
        return junctions;
    }

    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        std::cerr << "Error: XML file missing root element <net>.\n";
        return junctions;
    }

    std::cout << "XMLProcessor: Parsing junctions from " << filePath << std::endl;
    int junctionCount = 0;

    for (tinyxml2::XMLElement* juncElem = netElement->FirstChildElement("junction");
         juncElem != nullptr;
         juncElem = juncElem->NextSiblingElement("junction")) {

        const char* idAttr = juncElem->Attribute("id");
        if (!idAttr || idAttr[0] == ':') continue;  // Skip internal junctions

        double x = juncElem->DoubleAttribute("x");
        double y = juncElem->DoubleAttribute("y");
        std::string junctionId(idAttr);

        junctions.emplace(junctionId, Node(junctionId, x, y));
        junctionCount++;
    }

    std::cout << "XMLProcessor: Parsed " << junctionCount << " junctions" << std::endl;
    return junctions;
}

} // namespace veins
