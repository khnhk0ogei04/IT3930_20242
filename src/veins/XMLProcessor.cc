#include "XMLProcessor.h"
#include <algorithm>
#include <iostream>

using namespace std;

namespace veins {
vector<Edge> XMLProcessor::getRoadsFromXml(const string& filePath) {
    vector<Edge> roads;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
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
            int edgeLaneCount = 0;
            vector<Lane> lanes;
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
                    }
                }
            if (edgeLaneCount == 0) edgesWithNoLanes += 1;
            Edge edge(edgeId, fromNode, toNode, lanes);
            roads.push_back(edge);
            edgeCount++;
        }
    }
//    cout << "XMLProcessor: Parsed " << edgeCount << " edges and " << laneCount << " lanes in total" << endl;
//    cout << "XMLProcessor: Found " << edgesWithNoLanes << " edges with no lanes" << endl;
    return roads;
}

bool XMLProcessor::parseNetXml(const string& filePath, Graph& graph) {
    return ::parseNetXml(filePath, graph);
}

XMLProcessor::XMLProcessor() : networkLoaded(false) {
    vehicleData.clear();
}

bool XMLProcessor::loadNetworkFile(const string& filename) {
    incomingRoadsMap.clear();
    roadAttributes.clear();
    networkLoaded = false;
    vehicleData.clear();
    bool success = parseNetXml(filename, roadNetwork);
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filename.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        return false;
    }

    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        return false;
    }
    int edgeCount = 0;
    int laneCount = 0;
    for (tinyxml2::XMLElement* edgeElem = netElement->FirstChildElement("edge");
         edgeElem != nullptr;
         edgeElem = edgeElem->NextSiblingElement("edge")) {
            const char* idAttr = edgeElem->Attribute("id");
            if (!idAttr) continue;
            edgeCount++;
            string edgeId(idAttr);
            roadAttributes[edgeId] = extractAttributes(edgeElem);

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
                if (edgeLaneCount > 0) {
                    roadAttributes[edgeId]["laneCount"] = std::to_string(edgeLaneCount);
                }
    }
    buildIncomingRoadsMap();
    cout << "\ngraph structure..." << std::endl;
    const auto& adjList = roadNetwork.getAdjList();
    int nodeCount = 0, connectedNodeCount = 0;
    for (const auto& nodePair : adjList) {
        nodeCount++;
        if (!nodePair.second.empty()) {
            connectedNodeCount++;
        } else {
            std::cout << "DEBUG: Node " << nodePair.first << " has no outgoing connections" << std::endl;
        }
    }
//    cout << "DEBUG: Graph has " << nodeCount << " nodes, " << connectedNodeCount
//              << " have outgoing connections" << endl;
    int graphLaneCount = 0;
    for (const auto& nodePair : adjList) {
        for (const auto& edge : nodePair.second) {
            const auto& lanes = edge.getLanes();
            graphLaneCount += lanes.size();
        }
    }
    networkLoaded = true;
    return true;
}

vector<string> XMLProcessor::getAllRoads() const {
    vector<string> result;
    if (!networkLoaded) return result;
    const auto& adjList = roadNetwork.getAdjList();
    for (const auto& pair : adjList) {
        const auto& edges = pair.second;
        for (const auto& edge : edges) {
            result.push_back(edge.getId());
        }
    }
    sort(result.begin(), result.end());
    auto last = unique(result.begin(), result.end());
    result.erase(last, result.end());
    return result;
}

vector<string> XMLProcessor::getAccessibleRoads(const string& roadId) const {
    vector<string> result;
    if (!networkLoaded) return result;
    string toNodeId;

    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == roadId) {
                toNodeId = edge.getTo();
                auto it = roadNetwork.getAdjList().find(toNodeId);
                if (it != roadNetwork.getAdjList().end()) {
                    for (const auto& outEdge : it->second) {
                        if (outEdge.getId() != roadId) {
                            result.push_back(outEdge.getId());
                        }
                    }
                }
                return result;
            }
        }
    }
    return result;
}

vector<string> XMLProcessor::getIncomingRoads(const string& roadId) const {
    if (!networkLoaded) return vector<string>();
    auto it = incomingRoadsMap.find(roadId);
    if (it != incomingRoadsMap.end()) {
        return it->second;
    }
    
    return vector<string>();
}

map<string, string> XMLProcessor::getRoadAttributes(const string& roadId) const {
    if (!networkLoaded) return map<string, string>();
    
    auto it = roadAttributes.find(roadId);
    if (it != roadAttributes.end()) {
        return it->second;
    }
    
    return map<string, string>();
}

map<string, string> XMLProcessor::extractAttributes(const tinyxml2::XMLElement* element) const {
    map<string, string> attrs;
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
            const string& targetNodeId = edge.getTo();
            const string& edgeId = edge.getId();
            
            // find all outgoing edges from the target node
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

bool XMLProcessor::loadRouteFile(const string& filePath) {
    vehicleData.clear();
    cout << "Load routes from file: " << filePath << endl;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        cout << "Couldn't load file";
        return false;
    }
    tinyxml2::XMLElement* routesElement = doc.FirstChildElement("routes");
    int index = 0; // Sequential index for each vehicle
    for (tinyxml2::XMLElement* tripElem = routesElement->FirstChildElement("trip");
         tripElem != nullptr;
         tripElem = tripElem->NextSiblingElement("trip")) {

         Vehicle vehicle(index, tripElem->Attribute("id"), index); // vehicleId, sumoId, index
         index++; // Assign sequential index

         const char* departAttr = tripElem->Attribute("depart");
         double departTime = 0.0;
         if (departAttr) {
            departTime = std::stod(departAttr);
         }

         string fromRoad = tripElem->Attribute("from");
         string toRoad = tripElem->Attribute("to");
         string viaRoad = "";
         
         const char* viaAttr = tripElem->Attribute("via");
         if (viaAttr) {
            viaRoad = viaAttr;
         }
         
         vehicle.setRouteInfo(fromRoad, toRoad, departTime, viaRoad);
         vehicleData.push_back(vehicle);
    }
    cout << "XMLProcessor: Loaded " << vehicleData.size() << " vehicles from " << filePath << endl;
    return true;
}

vector<Vehicle> XMLProcessor::getVehicles() const {
    return vehicleData;
}

unordered_map<string, Node> XMLProcessor::getJunctionsFromXml(const string& filePath) {
    unordered_map<string, Node> junctions;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(filePath.c_str());
    tinyxml2::XMLElement* netElement = doc.FirstChildElement("net");
    cout << "XMLProcessor: Parsing junctions from " << filePath << endl;
    int junctionCount = 0;
    for (tinyxml2::XMLElement* juncElem = netElement->FirstChildElement("junction");
         juncElem != nullptr;
         juncElem = juncElem->NextSiblingElement("junction")) {
        const char* idAttr = juncElem->Attribute("id");
        if (!idAttr || idAttr[0] == ':') continue;
        double x = juncElem->DoubleAttribute("x");
        double y = juncElem->DoubleAttribute("y");
        string junctionId(idAttr);
        junctions.emplace(junctionId, Node(junctionId, x, y));
        junctionCount++;
    }
    cout << "XMLProcessor: Parsed " << junctionCount << " junctions" << endl;
    return junctions;
}
} // namespace veins
