#include "XMLProcessor.h"
#include <algorithm>
#include <iostream>

namespace veins {

// Implement only the parseNetXml method
bool XMLProcessor::parseNetXml(const std::string& filePath, Graph& graph) {
    // Use the global parseNetXml function defined in Graph.cc
    return ::parseNetXml(filePath, graph);
}

XMLProcessor::XMLProcessor() : networkLoaded(false) {
    // Constructor initialization
}

bool XMLProcessor::loadNetworkFile(const std::string& filename) {
    // Clear any existing data
    incomingRoadsMap.clear();
    roadAttributes.clear();
    networkLoaded = false;
    
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
        
        // Process lanes for this edge
        int laneCount = 0;
        
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
            }
        }
        
        // Add lane count to attributes
        if (laneCount > 0) {
            roadAttributes[edgeId]["laneCount"] = std::to_string(laneCount);
        }
        
        std::cout << "XML Edge: " << edgeId << " has " << laneCount << " lanes in XML" << std::endl;
    }
    
    std::cout << "XML Processing found " << edgeCount << " edges and " << laneCount << " lanes" << std::endl;
    
    // Build the map of incoming roads
    buildIncomingRoadsMap();
    
    // Verify lanes in the graph
    int graphLaneCount = 0;
    const auto& adjList = roadNetwork.getAdjList();
    for (const auto& nodePair : adjList) {
        for (const auto& edge : nodePair.second) {
            const auto& lanes = edge.getLanes();
            graphLaneCount += lanes.size();
            std::cout << "Graph Edge: " << edge.getId() << " has " << lanes.size() << " lanes in Graph" << std::endl;
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
    
    // Optimized algorithm: For each edge, add it as an incoming edge to all
    // edges that start from its target node
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

} // namespace veins
