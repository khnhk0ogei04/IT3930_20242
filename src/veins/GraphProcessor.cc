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
}

vector<string> GraphProcessor::findShortestPath(string sourceId, string targetId) const {
    auto dijkstraResult = dijkstra(sourceId);
    return reconstructPath(dijkstraResult, sourceId, targetId);
}

double GraphProcessor::getShortestPathLength(string sourceId, string targetId) const {
    auto dijkstraResult = dijkstra(sourceId);
    auto it = dijkstraResult.find(targetId);
    if (it != dijkstraResult.end()){
        cout << "path found with length " << it->second.first << std::endl;
        return it->second.first;
    }
    return -1.0;
}

double GraphProcessor::getEdgeLength(string sourceEdgeId, string targetEdgeId) const {
    // If the edges are the same, return 0
    if (sourceEdgeId == targetEdgeId) {
        return 0.0;
    }
    
    // Look for direct edge in the graph
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                // Found the source edge, now look for the target edge
                string sourceNodeTo = edge.getTo();
                
                // Check if the target edge starts from where the source edge ends
                for (const auto& nextNodePair : roadNetwork.getAdjList()) {
                    if (nextNodePair.first == sourceNodeTo) {
                        for (const auto& nextEdge : nextNodePair.second) {
                            if (nextEdge.getId() == targetEdgeId) {
                                // Found direct connection
                                return edge.getLength();
                            }
                        }
                    }
                }
            }
        }
    }
    
    // If no direct connection, look for the individual edge lengths
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                return edge.getLength();
            }
        }
    }
    
    // If edge not found, return a default value
    return 100.0;
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
    return hungarianAlgorithm(costMatrix);
}

std::map<std::string, std::pair<double, std::string>> GraphProcessor::dijkstra(string sourceId) const {
    // Map to store the distance and previous node for each node (node_id -> {distance, previous_node})
    std::map<std::string, std::pair<double, std::string>> result;

    // Check if source node exists in the graph
    bool sourceExists = false;
    for (const auto& nodePair : roadNetwork.getNodes()) {
        if (nodePair.first == sourceId) {
            sourceExists = true;
            break;
        }
    }

    auto adjIt = roadNetwork.getAdjList().find(sourceId);
    bool sourceInAdj = (adjIt != roadNetwork.getAdjList().end());
    if (!sourceExists && !sourceInAdj) {
        return result;
    }

    // Count outgoing edges from the source
    int outgoingEdges = 0;
    if (sourceInAdj) {
        outgoingEdges = adjIt->second.size();
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
        visited.insert(current.id);
        nodesProcessed++;
        auto it = roadNetwork.getAdjList().find(current.id);
        if (it != roadNetwork.getAdjList().end()) {
            const auto& edges = it->second;

            for (const auto& edge : edges) {
                std::string neighborId = edge.getTo();
                double weight = edge.getLength();
                if (visited.find(neighborId) != visited.end()) {
                    continue;
                }
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

    // Report on number of reachable nodes
    int reachableCount = 0;
    for (const auto& pair : result) {
        if (pair.second.first < std::numeric_limits<double>::infinity()) {
            reachableCount++;
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

bool GraphProcessor::hungarianAlgorithm(const vector<vector<double>>& costMatrix) const {
    int n = costMatrix.size();
    vector<vector<double>> matrix = costMatrix;
    const double NO_PATH_PENALTY = 9999999.0;
    for (int i = 0; i < n; ++i) {
        double minVal = NO_PATH_PENALTY;
        for (int j = 0; j < n; ++j) {
            minVal = std::min(minVal, matrix[i][j]);
        }
        if (minVal < NO_PATH_PENALTY && minVal > 0) {
            for (int j = 0; j < n; ++j) {
                if (matrix[i][j] < NO_PATH_PENALTY) {
                    matrix[i][j] -= minVal;
                }
            }
        }
    }
    for (int j = 0; j < n; ++j) {
        double minVal = NO_PATH_PENALTY;
        for (int i = 0; i < n; ++i) {
            minVal = std::min(minVal, matrix[i][j]);
        }
        if (minVal < NO_PATH_PENALTY && minVal > 0) {
            for (int i = 0; i < n; ++i) {
                if (matrix[i][j] < NO_PATH_PENALTY) {
                    matrix[i][j] -= minVal;
                }
            }
        }
    }
    bool hasZeros = false;
    for (int i = 0; i < n && !hasZeros; ++i) {
        for (int j = 0; j < n && !hasZeros; ++j) {
            if (matrix[i][j] == 0) {
                hasZeros = true;
            }
        }
    }
    if (!hasZeros) {
        return false;
    }
    vector<int> rowAssignment(n, -1);
    vector<bool> colAssigned(n, false);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (matrix[i][j] == 0 && !colAssigned[j]) {
                rowAssignment[i] = j;
                colAssigned[j] = true;
                break;
            }
        }
    }
    int assignedCount = 0;
    for (int i = 0; i < n; ++i) {
        if (rowAssignment[i] != -1) {
            assignedCount++;
        }
    }
    
    // We need at least min(n, costMatrix[0].size()) assignments for a valid solution
    return (assignedCount >= std::min(n, (int)costMatrix[0].size()));
}

string GraphProcessor::extractEdgeIdFromLane(string laneId) const {
    int pos = laneId.find_last_of('_');
    if (pos != string::npos) {
        return laneId.substr(0, pos);
    }
    return laneId;
}

int GraphProcessor::extractLaneIndexFromLane(string laneId) const {
    int pos = laneId.find_last_of('_');
    if (pos != string::npos && pos < laneId.length() - 1) {
        try {
            return stoi(laneId.substr(pos + 1));
        } catch (const std::invalid_argument&) {
            return 0;
        }
    }
    return 0;
}

int GraphProcessor::findBestLaneForEdge(string edgeId) const {
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == edgeId) {
                const auto& lanes = edge.getLanes();

                if (lanes.empty()) {
                    return 0;
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
                break;
            }
        }
        if (targetFound) break;
    }

    if (!sourceFound || !targetFound) {
        return result;
    }

    // Check direct connection between source and target
    if (sourceToNode == targetFromNode) {
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
        // Find path between nodes
        std::vector<std::string> nodePath = findShortestPath(startNode, endNode);

        if (!nodePath.empty()) {

            // The path we want is: sourceEdge -> connecting edges -> targetEdge
            result.push_back(sourceEdgeId);
            result.insert(result.end(), nodePath.begin(), nodePath.end());
            result.push_back(targetEdgeId);

            return result;
        }
    }
    std::vector<std::string> directPath = findShortestPath(sourceEdgeId, targetEdgeId);
    if (!directPath.empty()) {
        result = directPath;
        return result;
    }
    return result;
}

double GraphProcessor::getEdgeShortestPathLength(string sourceEdgeId, string targetEdgeId) const {
    // Special case: source and target are the same edge
    if (sourceEdgeId == targetEdgeId) {
        // Find edge length
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
                    return edge.getLength();
                }
            }
        }
        return 100.0; // Default length if edge not found
    }

    // Find source and target edge info
    string sourceFromNode = "", sourceToNode = "";
    string targetFromNode = "", targetToNode = "";
    double sourceLength = 0.0, targetLength = 0.0;
    bool sourceFound = false;
    for (const auto& nodePair : roadNetwork.getAdjList()) {
        for (const auto& edge : nodePair.second) {
            if (edge.getId() == sourceEdgeId) {
                sourceFromNode = nodePair.first;
                sourceToNode = edge.getTo();
                sourceLength = edge.getLength();
                sourceFound = true;
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
                targetLength = edge.getLength();
                targetFound = true;
                break;
            }
        }
        if (targetFound) break;
    }

    // If either edge not found, return -1
    if (!sourceFound || !targetFound) {
        return -1.0;
    }

    // Direct connection (end of source connects to start of target)
    if (sourceToNode == targetFromNode) {
        double totalLen = sourceLength + targetLength;
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
    for (const auto& pair : nodePairs) {
        double pathLength = getShortestPathLength(pair.first, pair.second);
        if (pathLength > 0 && pathLength < shortestLength) {
            shortestLength = pathLength;
        }
    }

    // If we found a path between any of the node pairs
    if (shortestLength < std::numeric_limits<double>::max()) {
        // Add the lengths of source and target edges to the path
        double totalLength = sourceLength + shortestLength + targetLength;
        return totalLength;
    }

    // No path found
    std::cout << "  DEBUG: No path found between edges " << sourceEdgeId << " and " << targetEdgeId << std::endl;
    return -1.0;
}

// Stable Hungarian Algorithm (fix infinite loop and ensure correct line cover)
// Author: ChatGPT, for user request (no external class, core logic only)
// Paste directly into GraphProcessor::getOptimalVehicleAssignment()

vector<int> GraphProcessor::getOptimalVehicleAssignment(
    const vector<std::string>& sourceEdges,
    const vector<std::string>& destEdges) const {
    int numVehicles = sourceEdges.size();
    int numDestinations = destEdges.size();
    if (numVehicles == 0 || numDestinations == 0) return vector<int>();
    int n = max(numVehicles, numDestinations);
    vector<vector<double>> C(n, vector<double>(n, 9999999.0));
    const double NO_PATH_PENALTY = 9999999.0, SAME_EDGE_PENALTY = 5000.0;

    // Build cost matrix (unchanged)
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numDestinations; ++j) {
            if (sourceEdges[i] == destEdges[j]) {
                C[i][j] = SAME_EDGE_PENALTY;
            } else {
                auto path = findEdgeShortestPath(sourceEdges[i], destEdges[j]);
                double pathLength = 0.0;
                if (!path.empty()) {
                    for (const auto& edgeId : path) {
                        for (const auto& nodePair : roadNetwork.getAdjList()) {
                            for (const auto& edge : nodePair.second) {
                                if (edge.getId() == edgeId) pathLength += edge.getLength();
                            }
                        }
                    }
                }
                C[i][j] = path.empty() ? NO_PATH_PENALTY : pathLength;
            }
        }
    }
    for (int i = 0; i < n; ++i) for (int j = numDestinations; j < n; ++j) C[i][j] = NO_PATH_PENALTY;
    for (int i = numVehicles; i < n; ++i) for (int j = 0; j < n; ++j) C[i][j] = NO_PATH_PENALTY;

    // Step 1: Row reduction
    for (int i = 0; i < n; ++i) {
        double rowMin = *min_element(C[i].begin(), C[i].end());
        for (int j = 0; j < n; ++j) C[i][j] -= rowMin;
    }
    // Step 2: Column reduction
    for (int j = 0; j < n; ++j) {
        double colMin = C[0][j];
        for (int i = 1; i < n; ++i) colMin = min(colMin, C[i][j]);
        for (int i = 0; i < n; ++i) C[i][j] -= colMin;
    }

    // Main loop: Hungarian with real minimum line cover by alternating path
    vector<int> rowAssign(n, -1), colAssign(n, -1);
    vector<vector<bool>> starred(n, vector<bool>(n, false));
    // Initial greedy star (maximum matching of zeros)
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (C[i][j] == 0 && rowAssign[i] == -1 && colAssign[j] == -1) {
                starred[i][j] = true;
                rowAssign[i] = j;
                colAssign[j] = i;
            }
        }
    }

    while (true) {
        // 1. Cover all columns with a starred zero
        vector<bool> rowCover(n, false), colCover(n, false);
        for (int j = 0; j < n; ++j)
            if (colAssign[j] != -1) colCover[j] = true;

        // 2. If n columns are covered, optimal assignment found
        int count = 0;
        for (int j = 0; j < n; ++j) if (colCover[j]) ++count;
        if (count == n) break;

        // 3. Find a non-covered zero
        vector<vector<bool>> prime(n, vector<bool>(n, false));
        int zrow = -1, zcol = -1;
        while (true) {
            bool found = false;
            for (int i = 0; i < n && !found; ++i) {
                if (rowCover[i]) continue;
                for (int j = 0; j < n; ++j) {
                    if (!colCover[j] && C[i][j] == 0) {
                        zrow = i; zcol = j; found = true; break;
                    }
                }
            }
            if (!found) break; // Step 4: adjust matrix
            prime[zrow][zcol] = true;
            // If there's no star in this row, go to augmenting path
            int starCol = -1;
            for (int j = 0; j < n; ++j) if (starred[zrow][j]) { starCol = j; break; }
            if (starCol == -1) {
                // Augmenting path
                vector<pair<int, int>> path;
                path.emplace_back(zrow, zcol);
                while (true) {
                    // Find star in this col
                    int rowStar = -1;
                    for (int i = 0; i < n; ++i) if (starred[i][path.back().second]) { rowStar = i; break; }
                    if (rowStar == -1) break;
                    path.emplace_back(rowStar, path.back().second);
                    // Find prime in this row
                    int colPrime = -1;
                    for (int j = 0; j < n; ++j) if (prime[path.back().first][j]) { colPrime = j; break; }
                    path.emplace_back(path.back().first, colPrime);
                }
                // Invert stars/primes along path
                for (size_t k = 0; k < path.size(); ++k) {
                    int r = path[k].first, c = path[k].second;
                    starred[r][c] = !starred[r][c];
                }
                // Clear covers/primes
                fill(rowCover.begin(), rowCover.end(), false);
                fill(colCover.begin(), colCover.end(), false);
                for (auto& v : prime) fill(v.begin(), v.end(), false);
                // Rebuild rowAssign/colAssign
                fill(rowAssign.begin(), rowAssign.end(), -1);
                fill(colAssign.begin(), colAssign.end(), -1);
                for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) {
                    if (starred[i][j]) { rowAssign[i] = j; colAssign[j] = i; }
                }
                break;
            } else {
                rowCover[zrow] = true;
                colCover[starCol] = false;
            }
        }

        // Step 4: adjust matrix
        double delta = NO_PATH_PENALTY;
        for (int i = 0; i < n; ++i) if (!rowCover[i]) for (int j = 0; j < n; ++j)
            if (!colCover[j]) delta = min(delta, C[i][j]);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) {
            if (rowCover[i]) C[i][j] += delta;
            if (!colCover[j]) C[i][j] -= delta;
        }
    }
    // Extract assignment
    vector<int> result(numVehicles, -1);
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numDestinations; ++j)
            if (starred[i][j]) { result[i] = j; break; }
    }
    return result;
}


} // namespace veins
