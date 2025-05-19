#include "Graph.h"
#include "tinyxml2.h"
#include <iostream>
#include <cstdlib>

using namespace tinyxml2;
using namespace std;

void Graph::addNode(string id, double x, double y) {
    if (nodes.find(id) == nodes.end()) {
        nodes.emplace(id, Node(id, x, y));
        adjList.emplace(id, vector<Edge>());
    }
}

void Graph::addEdge(string from, string to, double length, string id) {
    if (from.empty() || to.empty()) {
        return;
    }
    Edge edge(id, from, to, length, "");
    addEdge(edge);
}

void Graph::addEdge(const Edge& edge) {
    string from = edge.getFrom();
    string to = edge.getTo();
    string id = edge.getId();
    if (from.empty() || to.empty()) {
        return;
    }
    if (nodes.find(from) == nodes.end()) {
        nodes[from] = Node(from);
    }
    if (nodes.find(to) == nodes.end()) {
        nodes[to] = Node(to);
    }
    adjList[from].push_back(edge);
    edges[id] = edge;

    // debug
    cout << "Graph Edge: " << id << " has " << edge.getLanes().size() << " lanes in Graph" << std::endl;
    // count number of edges in graph
    edgeCount++;
}

size_t Graph::getNodeCount() const {
    return nodes.size();
}

size_t Graph::getEdgeCount() const {
    return edgeCount;
}

void Graph::printNeighbors() const {
    for (const auto& nodePair : adjList) {
        string fromId = nodePair.first;
        const vector<Edge> edges = nodePair.second;
        cout << "Node " << fromId << " adjacent to:\n";
        for (const auto& edge : edges) {
            cout << "  - Node " << edge.getTo() << " (Distance: " << edge.getLength() << ")\n";
        }
    }
}


// Extracts all road from .net.xml file:
vector<Edge> Graph::getRoadsFromXml(string filePath) {
    vector<Edge> roads;
    XMLDocument doc;
    XMLError result = doc.LoadFile(filePath.c_str());
    if (result != XML_SUCCESS) {
        cerr << "Error: Could not open XML file \"" << filePath << "\".\n";
        return roads;
    }
    XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        cerr << "Error: XML file missing root element <net>.\n";
        return roads;
    }

    std::cout << "Parsing road network from " << filePath << std::endl;
    int edgeCount = 0;
    int laneCount = 0;
    int edgesWithNoLanes = 0;

    for (XMLElement* edgeElem = netElement->FirstChildElement("edge");
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
            for (XMLElement* laneElem = edgeElem->FirstChildElement("lane");
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
            
            std::cout << "Edge " << edgeId << " has " << edgeLaneCount << " lanes" << std::endl;
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
    
    std::cout << "Parsed " << edgeCount << " edges and " << laneCount << " lanes in total" << std::endl;
    std::cout << "Found " << edgesWithNoLanes << " edges with no lanes" << std::endl;
    
    return roads;
}

unordered_map<string, Node> Graph::getJunctionsFromXml(string filePath) {
    unordered_map<string, Node> junctions;

    XMLDocument doc;
    XMLError result = doc.LoadFile(filePath.c_str());
    if (result != XML_SUCCESS) {
        cerr << "Error: Could not open XML file \"" << filePath << "\".\n";
        return junctions;
    }

    XMLElement* netElement = doc.FirstChildElement("net");
    if (!netElement) {
        cerr << "Error: XML file missing root element <net>.\n";
        return junctions;
    }

    for (XMLElement* juncElem = netElement->FirstChildElement("junction");
         juncElem != nullptr;
         juncElem = juncElem->NextSiblingElement("junction")) {

        const char* idAttr = juncElem->Attribute("id");
        if (!idAttr || idAttr[0] == ':') continue;

        double x = juncElem->DoubleAttribute("x");
        double y = juncElem->DoubleAttribute("y");
        string junctionId(idAttr);

        junctions.emplace(junctionId, Node(junctionId, x, y));
    }

    return junctions;
}


bool parseNetXml(string filePath, Graph& graph) {

    unordered_map<string, Node> junctions = Graph::getJunctionsFromXml(filePath);
    for (const auto &junctionInfo : junctions) {
        string id = junctionInfo.first;
        Node node = junctionInfo.second;
        graph.addNode(id, node.getX(), node.getY());
    }

    // get roads from xml
    vector<Edge> roads = Graph::getRoadsFromXml(filePath);

    for (Edge edge: roads){
        graph.addEdge(edge);
    }

    return (!junctions.empty() || !roads.empty());
}

Edge Graph::getEdge(string edgeId) const {
    auto it = edges.find(edgeId);
    if (it != edges.end()) {
        return it->second;
    }
    return Edge();
}

