#include "GraphProcessor.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <set>
#include <limits>

namespace veins {

GraphProcessor::GraphProcessor(const Graph& graph) : roadNetwork(graph) {
    // Constructor
}

std::vector<std::string> GraphProcessor::findShortestPath(const std::string& sourceId, const std::string& targetId) const {
    // Run Dijkstra's algorithm from the source node
    auto dijkstraResult = dijkstra(sourceId);
    
    // Reconstruct the path from the dijkstra results
    return reconstructPath(dijkstraResult, sourceId, targetId);
}

double GraphProcessor::getShortestPathLength(const std::string& sourceId, const std::string& targetId) const {
    // Run Dijkstra's algorithm from the source node
    auto dijkstraResult = dijkstra(sourceId);
    
    // Check if the target node was reached
    auto it = dijkstraResult.find(targetId);
    if (it == dijkstraResult.end()) {
        return -1.0;  // No path exists
    }
    
    // Return the distance to the target node
    return it->second.first;
}

std::vector<std::vector<std::string>> GraphProcessor::findKShortestPaths(
    const std::string& sourceId, const std::string& targetId, int k) const {
    
    std::vector<std::vector<std::string>> result;
    
    // Find the shortest path first
    auto shortestPath = findShortestPath(sourceId, targetId);
    if (shortestPath.empty()) {
        return result;  // No path exists
    }
    
    result.push_back(shortestPath);
    
    // If only one path is requested, return it
    if (k <= 1) {
        return result;
    }
    
    // Improved approach to find alternative paths by modifying one edge at a time
    
    // Extract all edges from the shortest path
    std::set<std::string> shortestPathEdges(shortestPath.begin(), shortestPath.end());
    
    // For storing path info with length for sorting
    struct PathInfo {
        std::vector<std::string> path;
        double length;
    };
    std::vector<PathInfo> alternativePaths;
    
    // For each edge in the shortest path, try to find an alternative path that doesn't use it
    for (const auto& edgeToAvoid : shortestPathEdges) {
        // Run a modified Dijkstra's algorithm that avoids this edge
        std::map<std::string, double> distances;
        std::map<std::string, std::string> previous;
        std::set<std::string> visited;
        
        // Initialize distances
        for (const auto& nodePair : roadNetwork.getNodes()) {
            distances[nodePair.first] = std::numeric_limits<double>::infinity();
        }
        distances[sourceId] = 0.0;
        
        // Priority queue for Dijkstra's algorithm
        std::set<std::pair<double, std::string>> pq;
        pq.insert(std::make_pair(0.0, sourceId));
        
        // Dijkstra's algorithm
        while (!pq.empty()) {
            // Extract min
            auto it = pq.begin();
            double dist = it->first;
            std::string current = it->second;
            pq.erase(it);
            
            // Check if we reached the target
            if (current == targetId) {
                break;
            }
            
            // Skip if already visited
            if (visited.find(current) != visited.end()) {
                continue;
            }
            visited.insert(current);
            
            // Process neighbors
            auto adjIt = roadNetwork.getAdjList().find(current);
            if (adjIt != roadNetwork.getAdjList().end()) {
                for (const auto& edge : adjIt->second) {
                    // Skip the edge we want to avoid
                    if (edge.getId() == edgeToAvoid) {
                        continue;
                    }
                    
                    std::string neighbor = edge.getTo();
                    double newDist = dist + edge.getLength();
                    
                    if (newDist < distances[neighbor]) {
                        distances[neighbor] = newDist;
                        previous[neighbor] = edge.getId();
                        pq.insert(std::make_pair(newDist, neighbor));
                    }
                }
            }
        }
        
        // Reconstruct the path if we found one
        if (distances[targetId] != std::numeric_limits<double>::infinity()) {
            std::vector<std::string> path;
            std::string current = targetId;
            
            while (current != sourceId) {
                auto prevIt = previous.find(current);
                if (prevIt == previous.end()) {
                    // No way to reach from source to here
                    path.clear();
                    break;
                }
                
                std::string edgeId = prevIt->second;
                path.push_back(edgeId);
                
                // Find where this edge comes from
                bool found = false;
                for (const auto& nodePair : roadNetwork.getAdjList()) {
                    for (const auto& edge : nodePair.second) {
                        if (edge.getId() == edgeId) {
                            current = nodePair.first;  // This is the edge's source node
                            found = true;
                            break;
                        }
                    }
                    if (found) break;
                }
                
                if (!found) {
                    // Something went wrong
                    path.clear();
                    break;
                }
            }
            
            // Reverse the path (we built it backwards)
            std::reverse(path.begin(), path.end());
            
            // Check if this is a valid path and differs from the shortest path
            if (!path.empty() && path != shortestPath) {
                // Calculate the path length
                double pathLength = 0.0;
                for (const auto& edgeId : path) {
                    for (const auto& nodePair : roadNetwork.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                pathLength += edge.getLength();
                                break;
                            }
                        }
                    }
                }
                
                PathInfo info;
                info.path = path;
                info.length = pathLength;
                alternativePaths.push_back(info);
            }
        }
    }
    
    // Sort the alternative paths by length
    std::sort(alternativePaths.begin(), alternativePaths.end(), 
        [](const PathInfo& a, const PathInfo& b) {
            return a.length < b.length;
        }
    );
    
    // Add the best alternative paths to the result
    for (const auto& pathInfo : alternativePaths) {
        // Check if this path is different from all existing paths
        bool isDifferent = true;
        for (const auto& existingPath : result) {
            if (pathInfo.path == existingPath) {
                isDifferent = false;
                break;
            }
        }
        
        if (isDifferent) {
            result.push_back(pathInfo.path);
            if (result.size() >= k) {
                break;
            }
        }
    }
    
    return result;
}

bool GraphProcessor::existsValidAssignment(
    const std::vector<std::string>& sources, 
    const std::vector<std::string>& targets) const {
    
    // Check if the number of sources and targets match
    if (sources.size() != targets.size()) {
        return false;
    }
    
    // Create a cost matrix for the assignment problem
    std::vector<std::vector<double>> costMatrix(sources.size(), std::vector<double>(targets.size()));
    
    // Fill the cost matrix with path lengths
    for (size_t i = 0; i < sources.size(); ++i) {
        for (size_t j = 0; j < targets.size(); ++j) {
            double pathLength = getShortestPathLength(sources[i], targets[j]);
            
            if (pathLength < 0) {
                // No path exists, set to a very large value
                costMatrix[i][j] = std::numeric_limits<double>::max();
            } else {
                costMatrix[i][j] = pathLength;
            }
        }
    }
    
    // Apply the Hungarian algorithm to check if a valid assignment exists
    return hungarianAlgorithm(costMatrix);
}

std::map<std::string, std::pair<double, std::string>> GraphProcessor::dijkstra(const std::string& sourceId) const {
    // Map to store the distance and previous node for each node (node_id -> {distance, previous_node})
    std::map<std::string, std::pair<double, std::string>> result;
    
    // Priority queue for Dijkstra's algorithm
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    
    // Set of visited nodes
    std::set<std::string> visited;
    
    // Initialize distances to infinity
    for (const auto& nodePair : roadNetwork.getNodes()) {
        result[nodePair.first] = {std::numeric_limits<double>::infinity(), ""};
    }
    
    // Distance to the source node is 0
    result[sourceId] = {0.0, ""};
    
    // Add the source node to the priority queue
    pq.push({sourceId, 0.0});
    
    // Dijkstra's algorithm
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        
        // Skip if this node has already been processed
        if (visited.find(current.id) != visited.end()) {
            continue;
        }
        
        // Mark the node as visited
        visited.insert(current.id);
        
        // Process all outgoing edges from the current node
        auto it = roadNetwork.getAdjList().find(current.id);
        if (it != roadNetwork.getAdjList().end()) {
            const auto& edges = it->second;
            
            for (const auto& edge : edges) {
                std::string neighborId = edge.getTo();
                double weight = edge.getLength();
                
                // Skip if the neighbor has already been visited
                if (visited.find(neighborId) != visited.end()) {
                    continue;
                }
                
                // Relaxation step
                double newDistance = result[current.id].first + weight;
                if (newDistance < result[neighborId].first) {
                    result[neighborId] = {newDistance, edge.getId()};
                    pq.push({neighborId, newDistance});
                }
            }
        }
    }
    
    return result;
}

std::vector<std::string> GraphProcessor::reconstructPath(
    const std::map<std::string, std::pair<double, std::string>>& dijkstraResult,
    const std::string& sourceId, 
    const std::string& targetId) const {
    
    std::vector<std::string> path;
    
    // Check if the target node was reached
    auto it = dijkstraResult.find(targetId);
    if (it == dijkstraResult.end() || it->second.first == std::numeric_limits<double>::infinity()) {
        return path;  // No path exists
    }
    
    // Reconstruct the path by following the previous node pointers
    std::string currentNodeId = targetId;
    std::string currentEdgeId;
    
    while (currentNodeId != sourceId) {
        currentEdgeId = dijkstraResult.at(currentNodeId).second;
        
        if (currentEdgeId.empty()) {
            break;  // No previous edge, probably reached the source
        }
        
        path.push_back(currentEdgeId);
        
        // Find the edge to get its source node
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == currentEdgeId) {
                    currentNodeId = nodePair.first;
                    break;
                }
            }
        }
    }
    
    // Reverse the path since we reconstructed it backward
    std::reverse(path.begin(), path.end());
    
    return path;
}

bool GraphProcessor::hungarianAlgorithm(const std::vector<std::vector<double>>& costMatrix) const {
    // Simplified implementation that checks if each source can reach at least one target
    // and each target can be reached by at least one source
    
    const size_t n = costMatrix.size();
    
    // Check if each source can reach at least one target
    for (size_t i = 0; i < n; ++i) {
        bool canReachAnyTarget = false;
        
        for (size_t j = 0; j < n; ++j) {
            if (costMatrix[i][j] != std::numeric_limits<double>::max()) {
                canReachAnyTarget = true;
                break;
            }
        }
        
        if (!canReachAnyTarget) {
            return false;  // This source can't reach any target
        }
    }
    
    // Check if each target can be reached by at least one source
    for (size_t j = 0; j < n; ++j) {
        bool canBeReachedByAnySource = false;
        
        for (size_t i = 0; i < n; ++i) {
            if (costMatrix[i][j] != std::numeric_limits<double>::max()) {
                canBeReachedByAnySource = true;
                break;
            }
        }
        
        if (!canBeReachedByAnySource) {
            return false;  // This target can't be reached by any source
        }
    }
    
    return true;
}

} // namespace veins 