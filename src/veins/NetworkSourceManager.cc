#include "NetworkSourceManager.h"
#include <algorithm>
#include <iostream>

using namespace std;

namespace veins {

NetworkSourceManager::NetworkSourceManager() : networkLoaded(false) {
    // Constructor
}

bool NetworkSourceManager::loadNetwork(string filename) {
    bool success = parseNetXml(filename, roadNetwork);
    if (success) {
        networkLoaded = true;
        cout << "Loaded network from " << filename << endl;
        cout << "Network contains " << roadNetwork.getNodeCount() << " nodes and "
                  << roadNetwork.getEdgeCount() << " edges" << endl;
    } else {
        cout << "Failed to load network from " << filename << endl;
    }
    return networkLoaded;
}

vector<string> NetworkSourceManager::getAllNodes() const {
    vector<string> result;
    if (!networkLoaded) return result;
    
    const auto& nodes = roadNetwork.getNodes();
    for (const auto& pair : nodes) {
        result.push_back(pair.first);
    }
    return result;
}

vector<string> NetworkSourceManager::getAllEdges() const {
    vector<string> result;
    if (!networkLoaded) return result;
    
    const auto& adjList = roadNetwork.getAdjList();
    for (const auto& pair : adjList) {
        const auto& edges = pair.second;
        for (const auto& edge : edges) {
            result.push_back(edge.getId());
        }
    }
    
    // Remove duplicates
    sort(result.begin(), result.end());
    auto last = unique(result.begin(), result.end());
    result.erase(last, result.end());
    
    return result;
}

vector<string> NetworkSourceManager::getEdgesFromNode(string nodeId) const {
    vector<string> result;
    if (!networkLoaded) return result;
    
    const auto& adjList = roadNetwork.getAdjList();
    auto it = adjList.find(nodeId);
    if (it != adjList.end()) {
        const auto& edges = it->second;
        for (const auto& edge : edges) {
            result.push_back(edge.getId());
        }
    }
    
    return result;
}

vector<string> NetworkSourceManager::getConnectedEdges(string edgeId) const {
    vector<string> result;
    if (!networkLoaded) return result;
    
    const Edge* edge = findEdge(edgeId);
    if (!edge) return result;
    
    const string& targetNode = edge->getTo();

    // Get all edges starting from the target node
    const auto& adjList = roadNetwork.getAdjList();
    auto it = adjList.find(targetNode);
    if (it != adjList.end()) {
        const auto& edges = it->second;
        for (const auto& connEdge : edges) {
            if (connEdge.getId() != edgeId) {
                result.push_back(connEdge.getId());
            }
        }
    }
    
    return result;
}

string NetworkSourceManager::getEdgeSource(string edgeId) const {
    if (!networkLoaded) return "";

    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getFrom();
    }
    return "";
}

string NetworkSourceManager::getEdgeTarget(string edgeId) const {
    if (!networkLoaded) return "";

    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getTo();
    }
    return "";
}

double NetworkSourceManager::getEdgeLength(string edgeId) const {
    if (!networkLoaded) return 0.0;

    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getLength();
    }
    return 0.0;
}

const Edge* NetworkSourceManager::findEdge(string edgeId) const {
    if (!networkLoaded) return nullptr;
    
    const auto& adjList = roadNetwork.getAdjList();
    for (const auto& pair : adjList) {
        const auto& edges = pair.second;
        for (const auto& edge : edges) {
            if (edge.getId() == edgeId) {
                return &edge;
            }
        }
    }

    return nullptr;
}

} // namespace veins
