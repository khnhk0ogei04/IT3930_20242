#include "GraphProcessor.h"
#include <algorithm>
#include <iostream>
#include <queue>
#include <set>
#include <limits>

using namespace std;

namespace veins {
GraphProcessor::GraphProcessor(const Graph& graph) : roadNetwork(graph) {

}

GraphProcessor::~GraphProcessor() {
    // Nothing to clean up
}

vector<string> GraphProcessor::findShortestPath(string sourceId, string targetId) const {
    auto dijkstraResult = dijkstra(sourceId);
    return reconstructPath(dijkstraResult, sourceId, targetId);
}

double GraphProcessor::getShortestPathLength(string sourceId, string targetId) const {
    // Print debug information
    std::cout << "DEBUG: Finding shortest path length from " << sourceId << " to " << targetId << std::endl;

    // Run Dijkstra's algorithm from the source node
    auto dijkstraResult = dijkstra(sourceId);
    
    // Check if the target node was reached
    auto it = dijkstraResult.find(targetId);
    if (it == dijkstraResult.end()) {
        std::cout << "DEBUG: No path found from " << sourceId << " to " << targetId << std::endl;
        return -1.0;  // No path exists
    }

    // Return the distance to the target node
    std::cout << "DEBUG: Path found with length " << it->second.first << std::endl;
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
    if (k <= 1) {
        return result;
    }
    std::vector<std::pair<std::vector<std::string>, double>> potentialPaths;
    std::set<std::vector<std::string>> addedPaths;
    addedPaths.insert(shortestPath);
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

    std::cout << "DEBUG: Running Dijkstra from source node: " << sourceId << std::endl;

    // Check if source node exists in the graph
    bool sourceExists = false;
    for (const auto& nodePair : roadNetwork.getNodes()) {
        if (nodePair.first == sourceId) {
            sourceExists = true;
            break;
        }
    }

    // Also check if it exists in the adjacency list
    auto adjIt = roadNetwork.getAdjList().find(sourceId);
    bool sourceInAdj = (adjIt != roadNetwork.getAdjList().end());

    std::cout << "DEBUG: Source node " << sourceId << " exists in nodes map: "
              << (sourceExists ? "Yes" : "No") << ", in adjacency list: "
              << (sourceInAdj ? "Yes" : "No") << std::endl;

    if (!sourceExists && !sourceInAdj) {
        std::cout << "DEBUG: Source node doesn't exist in the graph!" << std::endl;
        return result;
    }

    // Count outgoing edges from the source
    int outgoingEdges = 0;
    if (sourceInAdj) {
        outgoingEdges = adjIt->second.size();
        std::cout << "DEBUG: Source has " << outgoingEdges << " outgoing edges" << std::endl;

        // Print first few outgoing edges
        int count = 0;
        for (const auto& edge : adjIt->second) {
            if (count++ < 5) {
                std::cout << "DEBUG:   Edge to " << edge.getTo() << " (ID: "
                          << edge.getId() << ", length: " << edge.getLength() << ")" << std::endl;
            }
        }
    }

    // Priority queue for Dijkstra's algorithm
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;

    // Set of visited nodes
    std::set<std::string> visited;

    // Initialize distances to infinity
    for (const auto& nodePair : roadNetwork.getNodes()) {
        result[nodePair.first] = {std::numeric_limits<double>::infinity(), ""};
    }

    // Make sure all nodes in the adjacency list are also initialized
    for (const auto& adjPair : roadNetwork.getAdjList()) {
        if (result.find(adjPair.first) == result.end()) {
            result[adjPair.first] = {std::numeric_limits<double>::infinity(), ""};
        }
    }

    // Distance to the source node is 0
    result[sourceId] = {0.0, ""};

    // Add the source node to the priority queue
    pq.push({sourceId, 0.0});

    int nodesProcessed = 0;
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
        nodesProcessed++;

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

                // Ensure the neighbor exists in the result map
                if (result.find(neighborId) == result.end()) {
                    result[neighborId] = {std::numeric_limits<double>::infinity(), ""};
                }

                if (newDistance < result[neighborId].first) {
                    result[neighborId] = {newDistance, edge.getId()};
                    pq.push({neighborId, newDistance});
                }
            }
        }
    }

    std::cout << "DEBUG: Processed " << nodesProcessed << " nodes out of "
              << roadNetwork.getNodes().size() << " total nodes" << std::endl;

    // Report on number of reachable nodes
    int reachableCount = 0;
    for (const auto& pair : result) {
        if (pair.second.first < std::numeric_limits<double>::infinity()) {
            reachableCount++;
        }
    }
    std::cout << "DEBUG: Found " << reachableCount << " reachable nodes from source " << sourceId << std::endl;

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
    
    std::cout << "\nDEBUG: Finding path from edge " << sourceEdgeId << " to edge " << targetEdgeId << std::endl;

    // Special case: source and target are the same edge
    if (sourceEdgeId == targetEdgeId) {
        result.push_back(sourceEdgeId);
        std::cout << "DEBUG: Source and target are the same edge, returning direct path" << std::endl;
        return result;
    }

    // First, find the source and target edges in the graph
    string sourceFromNode = "", sourceToNode = "";
    string targetFromNode = "", targetToNode = "";
    double sourceEdgeLength = 0.0, targetEdgeLength = 0.0;

    // Find source edge
    bool sourceFound = false;
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                sourceFromNode = nodePair.first;
                sourceToNode = edge.getTo();
                sourceEdgeLength = edge.getLength();
                sourceFound = true;
                std::cout << "DEBUG: Found source edge " << sourceEdgeId
                         << " (from=" << sourceFromNode << ", to=" << sourceToNode
                         << ", length=" << sourceEdgeLength << ")" << std::endl;
                break;
            }
        }
        if (sourceFound) break;
    }

    // Find target edge
    bool targetFound = false;
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == targetEdgeId) {
                targetFromNode = nodePair.first;
                targetToNode = edge.getTo();
                targetEdgeLength = edge.getLength();
                targetFound = true;
                std::cout << "DEBUG: Found target edge " << targetEdgeId
                         << " (from=" << targetFromNode << ", to=" << targetToNode
                         << ", length=" << targetEdgeLength << ")" << std::endl;
                break;
            }
        }
        if (targetFound) break;
    }

    if (!sourceFound || !targetFound) {
        std::cout << "DEBUG: One or both edges not found in the graph" << std::endl;
        if (!sourceFound) std::cout << "DEBUG: Source edge " << sourceEdgeId << " not found" << std::endl;
        if (!targetFound) std::cout << "DEBUG: Target edge " << targetEdgeId << " not found" << std::endl;
        return result;
    }

    // Print some graph information
    std::cout << "DEBUG: Graph has " << roadNetwork.getNodeCount() << " nodes and "
              << roadNetwork.getEdgeCount() << " edges" << std::endl;

    // Check direct connection between source and target
    if (sourceToNode == targetFromNode) {
        std::cout << "DEBUG: Direct connection exists (source.to == target.from)" << std::endl;
        result.push_back(sourceEdgeId);
        result.push_back(targetEdgeId);
        return result;
    }

    // Try all possible combinations of nodes between source and target edges
    std::vector<std::pair<std::string, std::string>> nodePairs = {
        {sourceToNode, targetFromNode},     // Source end to target start (most common)
        {sourceFromNode, targetFromNode},   // Source start to target start
        {sourceToNode, targetToNode},       // Source end to target end
        {sourceFromNode, targetToNode}      // Source start to target end
    };

    // Try each combination and pick the first successful path
    for (const auto& nodePair : nodePairs) {
        const std::string& startNode = nodePair.first;
        const std::string& endNode = nodePair.second;

        std::cout << "DEBUG: Trying to find path from node " << startNode << " to node " << endNode << std::endl;

        // Find path between nodes
        std::vector<std::string> nodePath = findShortestPath(startNode, endNode);

        if (!nodePath.empty()) {
            // Build the complete path
            std::cout << "DEBUG: Found path with " << nodePath.size() << " segments" << std::endl;

            // The path we want is: sourceEdge -> connecting edges -> targetEdge
            result.push_back(sourceEdgeId);
            result.insert(result.end(), nodePath.begin(), nodePath.end());
            result.push_back(targetEdgeId);

            std::cout << "DEBUG: Complete path: ";
            for (const auto& edgeId : result) {
                std::cout << edgeId << " -> ";
            }
            std::cout << "end" << std::endl;

            return result;
        }
    }

    // No path found between any combination of nodes
    std::cout << "DEBUG: No path found between any combination of edge nodes" << std::endl;

    // Try as a last resort if the edges themselves are directly connected as nodes
    std::cout << "DEBUG: Trying to find path treating edges as nodes..." << std::endl;
    std::vector<std::string> directPath = findShortestPath(sourceEdgeId, targetEdgeId);
    if (!directPath.empty()) {
        std::cout << "DEBUG: Found direct path treating edges as nodes" << std::endl;
        result = directPath;
        return result;
    }

    std::cout << "DEBUG: Failed to find any path between the edges" << std::endl;
    return result;
}

double GraphProcessor::getEdgeShortestPathLength(string sourceEdgeId, string targetEdgeId) const {
    // Special case: source and target are the same edge
    if (sourceEdgeId == targetEdgeId) {
        // Find edge length
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
                    std::cout << "DEBUG: Same edge " << sourceEdgeId << " has length " << edge.getLength() << std::endl;
                    return edge.getLength();
                }
            }
        }
        std::cout << "DEBUG: Same edge " << sourceEdgeId << " not found in graph, using default length 100.0" << std::endl;
        return 100.0; // Default length if edge not found
    }

    // Find source and target edge info
    string sourceFromNode = "", sourceToNode = "";
    string targetFromNode = "", targetToNode = "";
    double sourceLength = 0.0, targetLength = 0.0;

    // Find source edge
    bool sourceFound = false;
    std::cout << "\n--DEBUG: Finding path length from edge " << sourceEdgeId << " to edge " << targetEdgeId << std::endl;
    std::cout << "  DEBUG: Looking for source edge " << sourceEdgeId << " in graph..." << std::endl;

    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                sourceFromNode = nodePair.first;
                sourceToNode = edge.getTo();
                sourceLength = edge.getLength();
                sourceFound = true;
                std::cout << "  DEBUG: Found source edge " << sourceEdgeId
                         << " (from=" << sourceFromNode << ", to=" << sourceToNode
                         << ", length=" << sourceLength << ")" << std::endl;
                break;
            }
        }
        if (sourceFound) break;
    }

    // Find target edge
    bool targetFound = false;
    std::cout << "  DEBUG: Looking for target edge " << targetEdgeId << " in graph..." << std::endl;

    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == targetEdgeId) {
                targetFromNode = nodePair.first;
                targetToNode = edge.getTo();
                targetLength = edge.getLength();
                targetFound = true;
                std::cout << "  DEBUG: Found target edge " << targetEdgeId
                         << " (from=" << targetFromNode << ", to=" << targetToNode
                         << ", length=" << targetLength << ")" << std::endl;
                break;
            }
        }
        if (targetFound) break;
    }

    // If either edge not found, return -1
    if (!sourceFound || !targetFound) {
        std::cout << "  DEBUG: Failed to find edges in graph" << std::endl;
        if (!sourceFound) std::cout << "  DEBUG: Source edge " << sourceEdgeId << " not found" << std::endl;
        if (!targetFound) std::cout << "  DEBUG: Target edge " << targetEdgeId << " not found" << std::endl;
        return -1.0;
    }

    // Direct connection (end of source connects to start of target)
    if (sourceToNode == targetFromNode) {
        double totalLen = sourceLength + targetLength;
        std::cout << "  DEBUG: Direct connection found! Total length = "
                 << sourceLength << " + " << targetLength << " = " << totalLen << std::endl;
        return totalLen;
    }

    // Try to find path between nodes
    double shortestLength = std::numeric_limits<double>::max();

    // Try all combinations of source and target nodes
    std::vector<std::pair<std::string, std::string>> nodePairs = {
        {sourceToNode, targetFromNode},     // Source end to target start (most common)
        {sourceFromNode, targetFromNode},   // Source start to target start
        {sourceToNode, targetToNode},       // Source end to target end
        {sourceFromNode, targetToNode}      // Source start to target end
    };

    std::cout << "  DEBUG: Testing all combinations of nodes:" << std::endl;
    for (const auto& pair : nodePairs) {
        double pathLength = getShortestPathLength(pair.first, pair.second);
        std::cout << "  DEBUG:   Path from " << pair.first << " to " << pair.second
                 << ": Length = " << (pathLength > 0 ? std::to_string(pathLength) : "INF") << std::endl;
        if (pathLength > 0 && pathLength < shortestLength) {
            shortestLength = pathLength;
        }
    }

    // If we found a path between any of the node pairs
    if (shortestLength < std::numeric_limits<double>::max()) {
        // Add the lengths of source and target edges to the path
        double totalLength = sourceLength + shortestLength + targetLength;
        std::cout << "  DEBUG: Complete path found with length = "
                 << sourceLength << " + " << shortestLength << " + " << targetLength
                 << " = " << totalLength << std::endl;
        return totalLength;
    }

    // No path found
    std::cout << "  DEBUG: No path found between edges " << sourceEdgeId << " and " << targetEdgeId << std::endl;
    return -1.0;
}

// Implementation of the getOptimalVehicleAssignment method
std::vector<int> GraphProcessor::getOptimalVehicleAssignment(
    const std::vector<std::string>& sourceEdges,
    const std::vector<std::string>& destEdges) const {

    int numVehicles = sourceEdges.size();
    int numDestinations = destEdges.size();

    if (numVehicles == 0 || numDestinations == 0) {
        std::cout << "INFO: No vehicles or destinations to assign." << std::endl;
        return std::vector<int>();
    }

    // Create cost matrix with proper dimensions
    int n = std::max(numVehicles, numDestinations);
    std::vector<std::vector<double>> costMatrix(n, std::vector<double>(n, 0));
    const double NO_PATH_PENALTY = 9999999.0;
    const double SAME_EDGE_PENALTY = 5000.0; // High, but allows assignment if necessary

    std::cout << "INFO: Building Cost Matrix for Hungarian Assignment (" << numVehicles
              << " vehicles, " << numDestinations << " dests):" << std::endl;

    // Fill the cost matrix with edge-to-edge path costs
    for (int i = 0; i < numVehicles; ++i) {
        const std::string& sourceEdgeId = sourceEdges[i];

        for (int j = 0; j < numDestinations; ++j) {
            const std::string& destEdgeId = destEdges[j];

            // Special case: same edge assignment
            if (sourceEdgeId == destEdgeId) {
                costMatrix[i][j] = SAME_EDGE_PENALTY;
                std::cout << "DEBUG: Source " << i << " and dest " << j
                          << " are the same edge, cost = " << SAME_EDGE_PENALTY << std::endl;
                continue;
            }

            // Calculate path using findEdgeShortestPath() và tính tổng độ dài
            auto path = findEdgeShortestPath(sourceEdgeId, destEdgeId);
            double pathLength = 0.0;

            if (!path.empty()) {
                // Tính tổng chiều dài của đường đi
                for (const auto& edgeId : path) {
                    bool edgeFound = false;
                    for (const auto& nodePair : roadNetwork.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                pathLength += edge.getLength();
                                edgeFound = true;
                                break;
                            }
                        }
                        if (edgeFound) break;
                    }
                    if (!edgeFound) {
                        pathLength += 100.0; // Default length
                    }
                }

                // Cost = path length between edges
                costMatrix[i][j] = pathLength;
                std::cout << "DEBUG: Path cost from " << sourceEdgeId << " to "
                          << destEdgeId << " = " << pathLength << " (path length)" << std::endl;
            } else {
                // No path exists between these edges
                costMatrix[i][j] = NO_PATH_PENALTY;
                std::cout << "DEBUG: No path from " << sourceEdgeId << " to " << destEdgeId << std::endl;
            }
        }

        // Fill remaining elements of the row with NO_PATH_PENALTY (for rectangular matrix)
        for (int j = numDestinations; j < n; ++j) {
            costMatrix[i][j] = NO_PATH_PENALTY;
        }
    }
    
    // Fill remaining rows with NO_PATH_PENALTY (for rectangular matrix)
    for (int i = numVehicles; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            costMatrix[i][j] = NO_PATH_PENALTY;
        }
    }

    // Print the cost matrix for debugging
    std::cout << "\nINFO: Cost Matrix for Hungarian Algorithm:" << std::endl;
    std::string header = "       ";
    for (int j = 0; j < numDestinations; ++j) {
        header += "D_" + destEdges[j].substr(0, std::min((size_t)4, destEdges[j].length())) + "\t";
    }
    std::cout << header << std::endl;

    for (int i = 0; i < numVehicles; ++i) {
        std::string rowStr = "V_" + std::to_string(i) + "(" + sourceEdges[i].substr(0, std::min((size_t)4, sourceEdges[i].length())) + ")\t";
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] >= NO_PATH_PENALTY) {
                rowStr += "INF\t";
            } else {
                rowStr += std::to_string(static_cast<int>(costMatrix[i][j])) + "\t";
            }
        }
        std::cout << rowStr << std::endl;
    }

    // Hungarian algorithm implementation
    // Step 1: Subtract minimum value from each row
    for (int i = 0; i < n; ++i) {
        double minVal = *std::min_element(costMatrix[i].begin(), costMatrix[i].end());
        if (minVal < NO_PATH_PENALTY) {  // Only subtract if row has valid paths
            for (int j = 0; j < n; ++j) {
                if (costMatrix[i][j] < NO_PATH_PENALTY) {
                    costMatrix[i][j] -= minVal;
                }
            }
        }
    }

    // Step 2: Subtract minimum value from each column
    for (int j = 0; j < n; ++j) {
        double minVal = NO_PATH_PENALTY;
        for (int i = 0; i < n; ++i) {
            minVal = std::min(minVal, costMatrix[i][j]);
        }
        if (minVal < NO_PATH_PENALTY) {  // Only subtract if column has valid paths
            for (int i = 0; i < n; ++i) {
                if (costMatrix[i][j] < NO_PATH_PENALTY) {
                    costMatrix[i][j] -= minVal;
                }
            }
        }
    }

    // Print reduced cost matrix
    std::cout << "\nINFO: Reduced Cost Matrix:" << std::endl;
    for (int i = 0; i < numVehicles; ++i) {
        std::string rowStr = "V_" + std::to_string(i) + "\t";
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] >= NO_PATH_PENALTY) {
                rowStr += "INF\t";
            } else {
                rowStr += std::to_string(static_cast<int>(costMatrix[i][j])) + "\t";
            }
        }
        std::cout << rowStr << std::endl;
    }

    // For simplicity, we'll use a greedy approach on the reduced matrix
    // A full Hungarian implementation would be more complex
    std::vector<int> assignment(numVehicles, -1);
    std::vector<bool> destAssigned(numDestinations, false);
    int assignmentsMade = 0;

    // First pass: assign zeros in the reduced matrix
    for (int i = 0; i < numVehicles && assignmentsMade < std::min(numVehicles, numDestinations); ++i) {
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] == 0 && !destAssigned[j] && assignment[i] == -1) {
                assignment[i] = j;
                destAssigned[j] = true;
                assignmentsMade++;
                break;
            }
        }
    }

    // Second pass: assign remaining vehicles using minimum cost
    for (int i = 0; i < numVehicles; ++i) {
        if (assignment[i] == -1) { // Vehicle not assigned yet
            double minCost = NO_PATH_PENALTY;
            int bestDest = -1;
            for (int j = 0; j < numDestinations; ++j) {
                if (!destAssigned[j] && costMatrix[i][j] < minCost) {
                    minCost = costMatrix[i][j];
                    bestDest = j;
                }
            }
            if (bestDest != -1) {
                assignment[i] = bestDest;
                destAssigned[bestDest] = true;
                assignmentsMade++;
            }
        }
    }

    std::cout << "\nINFO: Assignment Results:" << std::endl;
    for (int i = 0; i < numVehicles; ++i) {
        if (assignment[i] != -1) {
            int j = assignment[i];
            
            // Get the accurate cost by recalculating path length using the exact same method
            double pathCost = 0;
            auto path = findEdgeShortestPath(sourceEdges[i], destEdges[j]);
            if (!path.empty()) {
                for (const auto& edgeId : path) {
                    bool edgeFound = false;
                    for (const auto& nodePair : roadNetwork.getAdjList()) {
                        for (const auto& edge : nodePair.second) {
                            if (edge.getId() == edgeId) {
                                pathCost += edge.getLength();
                                edgeFound = true;
                                break;
                            }
                        }
                        if (edgeFound) break;
                    }
                    if (!edgeFound) {
                        pathCost += 100.0; // Default length
                    }
                }
            } else {
                pathCost = NO_PATH_PENALTY;
            }
            
            std::cout << "Vehicle " << i << " (Edge " << sourceEdges[i] << ") assigned to Destination "
                      << j << " (Edge " << destEdges[j] << ") with path length " << pathCost << std::endl;
        } else {
            std::cout << "Vehicle " << i << " (Edge " << sourceEdges[i] << ") could not be assigned" << std::endl;
        }
    }
    std::cout << "Total assignments made: " << assignmentsMade << "/" << numVehicles << std::endl;

    return assignment;
}

} // namespace veins
