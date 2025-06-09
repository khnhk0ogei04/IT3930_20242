#include "Graph.h"
#include "tinyxml2.h"
#include "XMLProcessor.h"
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

Edge Graph::getEdge(string edgeId) const {
    auto it = edges.find(edgeId);
    if (it != edges.end()) {
        return it->second;
    }
    return Edge();
}

void Graph::addEdge(string from, string to, double length, string id) {
    Edge edge(id, from, to, length, "");
    if (nodes.find(from) == nodes.end()) {
        nodes[from] = Node(from);
    }
    if (nodes.find(to) == nodes.end()) {
        nodes[to] = Node(to);
    }
    adjList[from].push_back(edge);
    edges[id] = edge;
    edgeCount++;
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
    edgeCount++;
}

size_t Graph::getNodeCount() const {
    return nodes.size();
}

size_t Graph::getEdgeCount() const {
    return edgeCount;
}

vector<Edge> Graph::getRoadsFromXml(string filePath) {
    return veins::XMLProcessor::getRoadsFromXml(filePath);
}

unordered_map<string, Node> Graph::getJunctionsFromXml(string filePath) {
    return veins::XMLProcessor::getJunctionsFromXml(filePath);
}

bool parseNetXml(string filePath, Graph& graph) {
    unordered_map<string, Node> junctions = Graph::getJunctionsFromXml(filePath);
    for (const auto &junctionInfo : junctions) {
        string id = junctionInfo.first;
        Node node = junctionInfo.second;
        graph.addNode(id, node.getX(), node.getY());
    }
    vector<Edge> roads = veins::XMLProcessor::getRoadsFromXml(filePath);
    for (Edge edge: roads){
        graph.addEdge(edge);
    }
    return (!junctions.empty() || !roads.empty());
}

