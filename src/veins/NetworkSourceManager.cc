#include "NetworkSourceManager.h"
#include <algorithm>
#include <iostream>

namespace veins {

NetworkSourceManager::NetworkSourceManager() : networkLoaded(false) {
    // Constructor
}

bool NetworkSourceManager::loadNetwork(string filename) {
    bool success = parseNetXml(filename, roadNetwork);
    if (success) {
        networkLoaded = true;
        std::cout << "Loaded network from " << filename << std::endl;
        std::cout << "Network contains " << roadNetwork.getNodeCount() << " nodes and " 
                  << roadNetwork.getEdgeCount() << " edges" << std::endl;
    } else {
        std::cout << "Failed to load network from " << filename << std::endl;
    }
    return networkLoaded;
}

std::vector<std::string> NetworkSourceManager::getAllNodes() const {
    std::vector<std::string> result;
    if (!networkLoaded) return result;
    
    const auto& nodes = roadNetwork.getNodes();
    for (const auto& pair : nodes) {
        result.push_back(pair.first);
    }
    
    return result;
}

std::vector<std::string> NetworkSourceManager::getAllEdges() const {
    std::vector<std::string> result;
    if (!networkLoaded) return result;
    
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

std::vector<std::string> NetworkSourceManager::getEdgesFromNode(string nodeId) const {
    std::vector<std::string> result;
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

std::vector<std::string> NetworkSourceManager::getConnectedEdges(string edgeId) const {
    std::vector<std::string> result;
    if (!networkLoaded) return result;
    
    const Edge* edge = findEdge(edgeId);
    if (!edge) return result;
    
    const std::string& targetNode = edge->getTo();
    
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

std::string NetworkSourceManager::getEdgeSource(string edgeId) const {
    if (!networkLoaded) return "";
    
    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getFrom();
    }
    return "";
}

std::string NetworkSourceManager::getEdgeTarget(string edgeId) const {
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
