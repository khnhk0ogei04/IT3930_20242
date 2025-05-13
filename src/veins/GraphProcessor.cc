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

std::vector<std::string> GraphProcessor::findShortestPath(string sourceId, string targetId) const {
    // Run Dijkstra's algorithm from the source node
    auto dijkstraResult = dijkstra(sourceId);
    
    // Reconstruct the path from the dijkstra results
    return reconstructPath(dijkstraResult, sourceId, targetId);
}

double GraphProcessor::getShortestPathLength(string sourceId, string targetId) const {
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
    string sourceId, string targetId, int k) const {
    
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

    // Implement Yen's algorithm for k-shortest paths
    // Data structures for storing potential paths and their lengths
    std::vector<std::pair<std::vector<std::string>, double>> potentialPaths;
    std::set<std::vector<std::string>> addedPaths;  // To track paths we've already found
    addedPaths.insert(shortestPath);

    // Track path lengths for efficient sorting
    auto calculatePathLength = [this](const std::vector<std::string>& path) {
        double length = 0.0;
        for (const auto& edgeId : path) {
            for (const auto& nodePair : roadNetwork.getAdjList()) {
                for (const auto& edge : nodePair.second) {
                    if (edge.getId() == edgeId) {
                        length += edge.getLength();
                        break;
                    }
                }
            }
        }
        return length;
    };
    
    // For each path already found, find potential next-best paths
    for (int i = 0; result.size() < k && i < result.size(); ++i) {
        const auto& path = result[i];
        
        // For each node in the path (except the last), try alternative routes
        string lastNodeId = sourceId;
        std::vector<string> rootPath;
        
        for (size_t j = 0; j < path.size(); ++j) {
            // Get the current edge and its destination node
            string currentEdgeId = path[j];
            string currentNodeId = "";
            
            // Find where this edge goes to
            for (const auto& nodePair : roadNetwork.getAdjList()) {
                for (const auto& edge : nodePair.second) {
                    if (edge.getId() == currentEdgeId) {
                        currentNodeId = edge.getTo();
                        break;
                    }
                }
                if (!currentNodeId.empty()) break;
            }
            
            if (currentNodeId.empty()) continue;
            
            // The root path up to current deviation point
            std::vector<string> spurRootPath = rootPath;
            
            // Remove the links that were in previous k-shortest paths
            std::map<std::pair<string, string>, double> removedEdges;
            
            for (const auto& prevPath : result) {
                if (j < prevPath.size() && spurRootPath.size() <= prevPath.size() &&
                    std::equal(spurRootPath.begin(), spurRootPath.end(), prevPath.begin())) {
                    
                    // Find the next edge in the previous path
                    if (j < prevPath.size()) {
                        string edgeToRemove = prevPath[j];
                        
                        // Find the source node of this edge
                        for (const auto& nodePair : roadNetwork.getAdjList()) {
                            for (const auto& edge : nodePair.second) {
                                if (edge.getId() == edgeToRemove) {
                                    removedEdges[{nodePair.first, edge.getTo()}] = edge.getLength();
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            
            // Create a temporary modified graph excluding removed edges
            Graph tempGraph = roadNetwork;  // Make a copy (this is a deep copy in the original code)
            for (const auto& edgePair : removedEdges) {
                // We need to modify the real graph - but since we're in a const method, 
                // we can't modify the original graph
                // For simplicity, we'll implement this by using Dijkstra with avoided edges
                // ...
            }
            
            // Find shortest path from deviation node to target with modified graph
            // Since we can't easily modify the graph in a const method, we'll use a modified
            // version of dijkstra that avoids certain edges
            
            // Run a modified Dijkstra's algorithm
            std::map<std::string, double> distances;
            std::map<std::string, std::string> previous;
            std::set<std::string> visited;
            
            // Initialize distances
            for (const auto& nodePair : roadNetwork.getNodes()) {
                distances[nodePair.first] = std::numeric_limits<double>::infinity();
            }
            distances[lastNodeId] = 0.0;
            
            // Priority queue for Dijkstra's algorithm
            std::set<std::pair<double, std::string>> pq;
            pq.insert(std::make_pair(0.0, lastNodeId));
            
            // Dijkstra's algorithm that avoids specific edges
            while (!pq.empty()) {
                auto it = pq.begin();
                double dist = it->first;
                std::string current = it->second;
                pq.erase(it);
                
                if (current == targetId) {
                    break;
                }
                
                if (visited.find(current) != visited.end()) {
                    continue;
                }
                visited.insert(current);
                
                auto adjIt = roadNetwork.getAdjList().find(current);
                if (adjIt != roadNetwork.getAdjList().end()) {
                    for (const auto& edge : adjIt->second) {
                        std::string neighbor = edge.getTo();
                        
                        // Skip edges that were removed
                        if (removedEdges.find({current, neighbor}) != removedEdges.end()) {
                            continue;
                        }
                        
                        // Also check if this node is in the root path to avoid loops
                        bool inRootPath = false;
                        for (const auto& rootEdgeId : spurRootPath) {
                            for (const auto& nodePair : roadNetwork.getAdjList()) {
                                for (const auto& pathEdge : nodePair.second) {
                                    if (pathEdge.getId() == rootEdgeId && 
                                        (pathEdge.getTo() == neighbor || nodePair.first == neighbor)) {
                                        inRootPath = true;
                                        break;
                                    }
                                }
                                if (inRootPath) break;
                            }
                            if (inRootPath) break;
                        }
                        
                        if (inRootPath) continue;
                        
                        double newDist = dist + edge.getLength();
                        if (newDist < distances[neighbor]) {
                            distances[neighbor] = newDist;
                            previous[neighbor] = edge.getId();
                            pq.insert(std::make_pair(newDist, neighbor));
                        }
                    }
                }
            }
            
            // If we found a path from deviation to target
            if (distances[targetId] != std::numeric_limits<double>::infinity()) {
                // Reconstruct the path
                std::vector<std::string> spurPath;
                std::string current = targetId;
                
                while (current != lastNodeId) {
                    auto prevIt = previous.find(current);
                    if (prevIt == previous.end()) {
                        spurPath.clear();
                        break;
                    }
                    
                    std::string edgeId = prevIt->second;
                    spurPath.push_back(edgeId);
                    
                    // Find where this edge comes from
                    bool found = false;
                    for (const auto& nodePair : roadNetwork.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                current = nodePair.first;
                                found = true;
                                break;
                            }
                        }
                        if (found) break;
                    }
                    
                    if (!found) {
                        spurPath.clear();
                        break;
                    }
                }
                
                // Reverse the spur path (we built it backwards)
                std::reverse(spurPath.begin(), spurPath.end());
                
                // Combine root path and spur path
                std::vector<std::string> totalPath = spurRootPath;
                totalPath.insert(totalPath.end(), spurPath.begin(), spurPath.end());
                
                // If this path is new, add it to potential paths
                if (addedPaths.find(totalPath) == addedPaths.end()) {
                    double pathLength = calculatePathLength(totalPath);
                    potentialPaths.push_back({totalPath, pathLength});
                }
            }
            
            // Update the root path to include the current edge
            rootPath.push_back(currentEdgeId);
            lastNodeId = currentNodeId;
        }
        
        // Sort potential paths by length
        std::sort(potentialPaths.begin(), potentialPaths.end(),
            [](const std::pair<std::vector<std::string>, double>& a, 
               const std::pair<std::vector<std::string>, double>& b) {
                return a.second < b.second;
            });
        
        // Add the next best path to the result
        while (!potentialPaths.empty() && result.size() < k) {
            auto bestPath = potentialPaths.front();
            potentialPaths.erase(potentialPaths.begin());
            
            if (addedPaths.find(bestPath.first) == addedPaths.end()) {
                addedPaths.insert(bestPath.first);
                result.push_back(bestPath.first);
                break;
            }
        }
    }
    
    // If we didn't find enough paths, try the alternative approach from the original implementation
    if (result.size() < k) {
        // Extract all edges from the shortest path
        std::set<std::string> shortestPathEdges(shortestPath.begin(), shortestPath.end());
        
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
                auto it = pq.begin();
                double dist = it->first;
                std::string current = it->second;
                pq.erase(it);
                
                if (current == targetId) {
                    break;
                }
                
                if (visited.find(current) != visited.end()) {
                    continue;
                }
                visited.insert(current);
                
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
                                current = nodePair.first;
                                found = true;
                                break;
                            }
                        }
                        if (found) break;
                    }
                    
                    if (!found) {
                        path.clear();
                        break;
                    }
                }
                
                // Reverse the path (we built it backwards)
                std::reverse(path.begin(), path.end());
                
                // Check if this is a valid path and differs from paths we already have
                if (!path.empty()) {
                    if (addedPaths.find(path) == addedPaths.end()) {
                        addedPaths.insert(path);
                        result.push_back(path);
                        
                        if (result.size() >= k) {
                            break;
                        }
                    }
                }
            }
        }
    }
    
    // Final safety check to ensure we don't return more than k paths
    if (result.size() > k) {
        result.resize(k);
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

std::map<std::string, std::pair<double, std::string>> GraphProcessor::dijkstra(string sourceId) const {
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
    string sourceId,
    string targetId) const {
    
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

// Implementation for findLaneShortestPath
vector<GraphProcessor::LanePath> GraphProcessor::findLaneShortestPath(string sourceLaneId, string targetLaneId) const {
    vector<LanePath> result;
    
    // Extract edge IDs and lane indices from the lane IDs
    string sourceEdgeId = extractEdgeIdFromLane(sourceLaneId);
    string targetEdgeId = extractEdgeIdFromLane(targetLaneId);
    int sourceLaneIndex = extractLaneIndexFromLane(sourceLaneId);
    int targetLaneIndex = extractLaneIndexFromLane(targetLaneId);
    
    if (sourceEdgeId.empty() || targetEdgeId.empty() || sourceLaneIndex < 0 || targetLaneIndex < 0) {
        // Invalid lane IDs
        return result;
    }
    
    // Find the shortest path between the edges using the existing method
    vector<string> edgePath = findShortestPath(sourceEdgeId, targetEdgeId);
    
    if (edgePath.empty() && sourceEdgeId != targetEdgeId) {
        // No path exists between the edges
        return result;
    }
    
    // Special case: source and target are on the same edge
    if (sourceEdgeId == targetEdgeId) {
        // Create a single LanePath segment
        LanePath segment;
        segment.edgeId = sourceEdgeId;
        segment.laneIndex = sourceLaneIndex; // Start with source lane
        
        // Find the edge in the graph to calculate cost
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
                    // Cost is the length of the edge
                    segment.cost = edge.getLength();
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
        result.push_back(segment);
        return result;
    }
    
    // Add source edge (with source lane)
    if (!edgePath.empty()) {
        LanePath sourceSegment;
        sourceSegment.edgeId = sourceEdgeId;
        sourceSegment.laneIndex = sourceLaneIndex;
        
        // Find the source edge in the graph to calculate cost
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
                    // Cost is the length of the edge
                    sourceSegment.cost = edge.getLength();
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
        result.push_back(sourceSegment);
    }
    
    // Add intermediate edges (with best lanes)
    for (size_t i = 0; i < edgePath.size(); i++) {
        const string& edgeId = edgePath[i];
        
        // Skip if this is the source or target edge (we handle those separately)
        if (edgeId == sourceEdgeId || edgeId == targetEdgeId) {
            continue;
        }
        
        LanePath segment;
        segment.edgeId = edgeId;
        segment.laneIndex = findBestLaneForEdge(edgeId);
        
        // Find the edge in the graph to calculate cost
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    // Cost is the length of the edge
                    segment.cost = edge.getLength();
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
        result.push_back(segment);
    }
    
    // Add target edge (with target lane)
    if (!edgePath.empty() || sourceEdgeId == targetEdgeId) {
        LanePath targetSegment;
        targetSegment.edgeId = targetEdgeId;
        targetSegment.laneIndex = targetLaneIndex;
        
        // Find the target edge in the graph to calculate cost
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == targetEdgeId) {
                    // Cost is the length of the edge
                    targetSegment.cost = edge.getLength();
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        
        result.push_back(targetSegment);
    }
    
    return result;
}

string GraphProcessor::extractEdgeIdFromLane(string laneId) const {
    // In SUMO, lane IDs typically have the format "edgeId_laneIndex"
    size_t pos = laneId.find_last_of('_');
    if (pos != string::npos) {
        return laneId.substr(0, pos);
    }
    
    // If the lane ID doesn't contain an underscore, it might just be the edge ID
    // with a minus sign for direction (common in some SUMO networks)
    return laneId;
}

int GraphProcessor::extractLaneIndexFromLane(string laneId) const {
    // In SUMO, lane IDs typically have the format "edgeId_laneIndex"
    size_t pos = laneId.find_last_of('_');
    if (pos != string::npos && pos < laneId.length() - 1) {
        try {
            return stoi(laneId.substr(pos + 1));
        } catch (const std::invalid_argument&) {
            return 0; // Default to lane 0 if conversion fails
        }
    }
    
    // If no lane index is specified, default to 0
    return 0;
}

int GraphProcessor::findBestLaneForEdge(string edgeId) const {
    // Find the edge in the graph
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == edgeId) {
                const auto& lanes = edge.getLanes();
                
                if (lanes.empty()) {
                    return 0; // Default to lane 0 if no lanes are defined
                }
                
                // Find the lane with the highest speed
                int bestLaneIndex = 0;
                double highestSpeed = 0.0;
                
                for (const auto& lane : lanes) {
                    if (lane.speed > highestSpeed) {
                        highestSpeed = lane.speed;
                        bestLaneIndex = lane.index;
                    }
                }
                
                return bestLaneIndex;
            }
        }
    }
    
    // If edge not found, default to lane 0
    return 0;
}

// Implementation for findEdgeShortestPath
vector<string> GraphProcessor::findEdgeShortestPath(string sourceEdgeId, string targetEdgeId) const {
    vector<string> result;
    
    // Special case: source and target are the same edge
    if (sourceEdgeId == targetEdgeId) {
        result.push_back(sourceEdgeId);
        return result;
    }
    
    // First, find the source and target edges to get their associated nodes
    string sourceFromNode = "", sourceToNode = "";
    string targetFromNode = "", targetToNode = "";
    
    // Find the nodes for source and target edges
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        const string& fromNode = nodePair.first;
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                sourceFromNode = fromNode;
                sourceToNode = edge.getTo();
            }
            if (edge.getId() == targetEdgeId) {
                targetFromNode = fromNode;
                targetToNode = edge.getTo();
            }
        }
    }
    
    // If we couldn't find the edges, return empty path
    if (sourceFromNode.empty() || targetFromNode.empty()) {
        return result;
    }
    
    // Try different combinations of source and target nodes
    vector<vector<string>> possiblePaths;
    
    // 1. Path from source end node to target start node (most likely case)
    vector<string> path1 = findShortestPath(sourceToNode, targetFromNode);
    if (!path1.empty()) {
        // Construct the complete path: sourceEdge -> connecting edges -> targetEdge
        vector<string> completePath;
        completePath.push_back(sourceEdgeId);
        completePath.insert(completePath.end(), path1.begin(), path1.end());
        completePath.push_back(targetEdgeId);
        possiblePaths.push_back(completePath);
    }
    
    // 2. Path from source start node to target start node
    vector<string> path2 = findShortestPath(sourceFromNode, targetFromNode);
    if (!path2.empty()) {
        vector<string> completePath;
        completePath.push_back(sourceEdgeId);
        completePath.insert(completePath.end(), path2.begin(), path2.end());
        completePath.push_back(targetEdgeId);
        possiblePaths.push_back(completePath);
    }
    
    // 3. Path from source end node to target end node
    vector<string> path3 = findShortestPath(sourceToNode, targetToNode);
    if (!path3.empty()) {
        vector<string> completePath;
        completePath.push_back(sourceEdgeId);
        completePath.insert(completePath.end(), path3.begin(), path3.end());
        completePath.push_back(targetEdgeId);
        possiblePaths.push_back(completePath);
    }
    
    // 4. Path from source start node to target end node
    vector<string> path4 = findShortestPath(sourceFromNode, targetToNode);
    if (!path4.empty()) {
        vector<string> completePath;
        completePath.push_back(sourceEdgeId);
        completePath.insert(completePath.end(), path4.begin(), path4.end());
        completePath.push_back(targetEdgeId);
        possiblePaths.push_back(completePath);
    }
    
    // If we found any possible paths, choose the shortest one
    if (!possiblePaths.empty()) {
        // Find shortest path (by edge count)
        size_t shortestIndex = 0;
        size_t shortestLength = possiblePaths[0].size();
        
        for (size_t i = 1; i < possiblePaths.size(); i++) {
            if (possiblePaths[i].size() < shortestLength) {
                shortestLength = possiblePaths[i].size();
                shortestIndex = i;
            }
        }
        
        result = possiblePaths[shortestIndex];
    }
    
    // If we still have no path, try to find a direct path between the edges
    if (result.empty()) {
        vector<string> directPath = findShortestPath(sourceEdgeId, targetEdgeId);
        if (!directPath.empty()) {
            // Just use the direct path
            result = directPath;
        }
    }
    
    return result;
}

} // namespace veins 
