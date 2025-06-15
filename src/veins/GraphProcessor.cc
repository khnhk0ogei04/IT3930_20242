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



bool GraphProcessor::existsValidAssignment(
    const vector<string>& sources,
    const vector<string>& targets) const {
    if (sources.size() != targets.size()) {
        return false;
    }
    vector<vector<double>> costMatrix(sources.size(), vector<double>(targets.size()));
    for (int i = 0; i < sources.size(); ++i) {
        for (int j = 0; j < targets.size(); ++j) {
            double pathLength = getShortestPathLength(sources[i], targets[j]);
            if (pathLength < 0) {
                // no path exists, set to a very large value
                costMatrix[i][j] = numeric_limits<double>::max();
            } else {
                costMatrix[i][j] = pathLength;
            }
        }
    }
    return hungarianAlgorithm(costMatrix);
}

map<string, pair<double, string>> GraphProcessor::dijkstra(string sourceId) const {
    map<string, pair<double, string>> result;
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
    int outgoingEdges = 0;
    if (sourceInAdj) {
        outgoingEdges = adjIt->second.size();
    }
    priority_queue<DijkstraNode, vector<DijkstraNode>, greater<DijkstraNode>> pq;
    set<string> visited;
    for (const auto& nodePair : roadNetwork.getNodes()) {
        result[nodePair.first] = {numeric_limits<double>::infinity(), ""};
    }
    for (const auto& adjPair : roadNetwork.getAdjList()) {
        if (result.find(adjPair.first) == result.end()) {
            result[adjPair.first] = {numeric_limits<double>::infinity(), ""};
        }
    }
    result[sourceId] = {0.0, ""};
    pq.push({sourceId, 0.0});

    int nodesProcessed = 0;
    while (!pq.empty()) {
        DijkstraNode current = pq.top();
        pq.pop();
        if (visited.find(current.id) != visited.end()) {
            continue;
        }
        visited.insert(current.id);
        nodesProcessed++;
        auto it = roadNetwork.getAdjList().find(current.id);
        if (it != roadNetwork.getAdjList().end()) {
            const auto& edges = it->second;

            for (const auto& edge : edges) {
                string neighborId = edge.getTo();
                double weight = edge.getLength();
                if (visited.find(neighborId) != visited.end()) {
                    continue;
                }
                double newDistance = result[current.id].first + weight;
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
    int reachableCount = 0;
    for (const auto& pair : result) {
        if (pair.second.first < std::numeric_limits<double>::infinity()) {
            reachableCount++;
        }
    }
    return result;
}

vector<string> GraphProcessor::reconstructPath(
    const map<string, pair<double, string>>& dijkstraResult,
    string sourceId,
    string targetId) const {

    vector<string> path;
    auto it = dijkstraResult.find(targetId);
    if (it == dijkstraResult.end() || it->second.first == numeric_limits<double>::infinity()) {
        return path;
    }
    string currentNodeId = targetId;
    string currentEdgeId;

    while (currentNodeId != sourceId) {
        currentEdgeId = dijkstraResult.at(currentNodeId).second;
        if (currentEdgeId.empty()) {
            break;
        }
        path.push_back(currentEdgeId);
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == currentEdgeId) {
                    currentNodeId = nodePair.first;
                    break;
                }
            }
        }
    }
    reverse(path.begin(), path.end());
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
    if (sourceEdgeId == targetEdgeId) {
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
                    return edge.getLength();
                }
            }
        }
        return 0.0;
    }
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
    if (sourceToNode == targetFromNode) {
        double totalLen = sourceLength + targetLength;
        return totalLen;
    }
    double shortestLength = std::numeric_limits<double>::max();
    std::vector<std::pair<std::string, std::string>> nodePairs = {
        {sourceToNode, targetFromNode},     // Source end to target start
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
    if (shortestLength < std::numeric_limits<double>::max()) {
        double totalLength = sourceLength + shortestLength + targetLength;
        return totalLength;
    }
    return -1.0;
}
vector<int> GraphProcessor::getOptimalVehicleAssignment(
    const vector<string>& sourceEdges,
    const vector<string>& destEdges) const {
    int numVehicles = sourceEdges.size();
    int numDestinations = destEdges.size();
    if (numVehicles == 0 || numDestinations == 0) return vector<int>();
    int n = max(numVehicles, numDestinations);
    vector<vector<double>> C(n, vector<double>(n, 9999999.0));
    const double NO_PATH_PENALTY = 9999999.0, SAME_EDGE_PENALTY = 5000.0;
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

    for (int i = 0; i < n; ++i) {
        double rowMin = *min_element(C[i].begin(), C[i].end());
        for (int j = 0; j < n; ++j) C[i][j] -= rowMin;
    }
    for (int j = 0; j < n; ++j) {
        double colMin = C[0][j];
        for (int i = 1; i < n; ++i) colMin = min(colMin, C[i][j]);
        for (int i = 0; i < n; ++i) C[i][j] -= colMin;
    }
    vector<int> rowAssign(n, -1), colAssign(n, -1);
    vector<vector<bool>> starred(n, vector<bool>(n, false));
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
            int starCol = -1;
            for (int j = 0; j < n; ++j) if (starred[zrow][j]) { starCol = j; break; }
            if (starCol == -1) {
                vector<pair<int, int>> path;
                path.emplace_back(zrow, zcol);
                while (true) {
                    int rowStar = -1;
                    for (int i = 0; i < n; ++i) if (starred[i][path.back().second]) { rowStar = i; break; }
                    if (rowStar == -1) break;
                    path.emplace_back(rowStar, path.back().second);
                    int colPrime = -1;
                    for (int j = 0; j < n; ++j) if (prime[path.back().first][j]) { colPrime = j; break; }
                    path.emplace_back(path.back().first, colPrime);
                }
                for (size_t k = 0; k < path.size(); ++k) {
                    int r = path[k].first, c = path[k].second;
                    starred[r][c] = !starred[r][c];
                }
                fill(rowCover.begin(), rowCover.end(), false);
                fill(colCover.begin(), colCover.end(), false);
                for (auto& v : prime) fill(v.begin(), v.end(), false);
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
        double delta = NO_PATH_PENALTY;
        for (int i = 0; i < n; ++i) if (!rowCover[i]) for (int j = 0; j < n; ++j)
            if (!colCover[j]) delta = min(delta, C[i][j]);
        for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) {
            if (rowCover[i]) C[i][j] += delta;
            if (!colCover[j]) C[i][j] -= delta;
        }
    }
    vector<int> result(numVehicles, -1);
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numDestinations; ++j)
            if (starred[i][j]) { result[i] = j; break; }
    }
    return result;
}

vector<int> GraphProcessor::getOptimalAssignmentWithMatrix(
    const vector<vector<double>>& costMatrix) const {
    
    if (costMatrix.empty() || costMatrix[0].empty()) {
        return vector<int>();
    }
    
    int numRows = costMatrix.size();
    int numCols = costMatrix[0].size();
    int n = max(numRows, numCols);
    
    // Create square matrix for Hungarian algorithm
    vector<vector<double>> C(n, vector<double>(n, 9999999.0));
    
    // Copy input matrix
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            C[i][j] = costMatrix[i][j];
        }
    }
    
    // Apply Hungarian algorithm steps
    // Step 1: Row reduction
    for (int i = 0; i < n; ++i) {
        double rowMin = *min_element(C[i].begin(), C[i].end());
        if (rowMin < 9999999.0 && rowMin > 0) {
            for (int j = 0; j < n; ++j) {
                if (C[i][j] < 9999999.0) {
                    C[i][j] -= rowMin;
                }
            }
        }
    }
    
    // Step 2: Column reduction
    for (int j = 0; j < n; ++j) {
        double colMin = C[0][j];
        for (int i = 1; i < n; ++i) {
            colMin = min(colMin, C[i][j]);
        }
        if (colMin < 9999999.0 && colMin > 0) {
            for (int i = 0; i < n; ++i) {
                if (C[i][j] < 9999999.0) {
                    C[i][j] -= colMin;
                }
            }
        }
    }
    
    // Initialize assignment arrays
    vector<int> rowAssign(n, -1), colAssign(n, -1);
    vector<vector<bool>> starred(n, vector<bool>(n, false));
    
    // Find initial starred zeros
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (C[i][j] == 0 && rowAssign[i] == -1 && colAssign[j] == -1) {
                starred[i][j] = true;
                rowAssign[i] = j;
                colAssign[j] = i;
            }
        }
    }
    
    // Main Hungarian loop
    while (true) {
        // Cover all columns with a starred zero
        vector<bool> rowCover(n, false), colCover(n, false);
        for (int j = 0; j < n; ++j) {
            if (colAssign[j] != -1) colCover[j] = true;
        }
        
        // If n columns are covered, optimal assignment found
        int count = 0;
        for (int j = 0; j < n; ++j) if (colCover[j]) ++count;
        if (count == n) break;
        
        // Find a non-covered zero and prime it
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
            
            if (!found) break; // Go to step 4: adjust matrix
            
            prime[zrow][zcol] = true;
            int starCol = -1;
            for (int j = 0; j < n; ++j) {
                if (starred[zrow][j]) { starCol = j; break; }
            }
            
            if (starCol == -1) {
                // Augment path starting from primed zero
                vector<pair<int, int>> path;
                path.emplace_back(zrow, zcol);
                
                while (true) {
                    int rowStar = -1;
                    for (int i = 0; i < n; ++i) {
                        if (starred[i][path.back().second]) { rowStar = i; break; }
                    }
                    if (rowStar == -1) break;
                    path.emplace_back(rowStar, path.back().second);
                    
                    int colPrime = -1;
                    for (int j = 0; j < n; ++j) {
                        if (prime[path.back().first][j]) { colPrime = j; break; }
                    }
                    path.emplace_back(path.back().first, colPrime);
                }
                
                // Update starred zeros along path
                for (size_t k = 0; k < path.size(); ++k) {
                    int r = path[k].first, c = path[k].second;
                    starred[r][c] = !starred[r][c];
                }
                
                // Reset covers and primes
                fill(rowCover.begin(), rowCover.end(), false);
                fill(colCover.begin(), colCover.end(), false);
                for (auto& v : prime) fill(v.begin(), v.end(), false);
                fill(rowAssign.begin(), rowAssign.end(), -1);
                fill(colAssign.begin(), colAssign.end(), -1);
                
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        if (starred[i][j]) { 
                            rowAssign[i] = j; 
                            colAssign[j] = i; 
                        }
                    }
                }
                break;
            } else {
                rowCover[zrow] = true;
                colCover[starCol] = false;
            }
        }
        
        // Step 4: Adjust matrix
        double delta = 9999999.0;
        for (int i = 0; i < n; ++i) {
            if (!rowCover[i]) {
                for (int j = 0; j < n; ++j) {
                    if (!colCover[j]) {
                        delta = min(delta, C[i][j]);
                    }
                }
            }
        }
        
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (rowCover[i]) C[i][j] += delta;
                if (!colCover[j]) C[i][j] -= delta;
            }
        }
    }
    
    // Extract final assignment
    vector<int> result(numRows, -1);
    for (int i = 0; i < numRows; ++i) {
        for (int j = 0; j < numCols; ++j) {
            if (starred[i][j]) { 
                result[i] = j; 
                break; 
            }
        }
    }
    
    return result;
}

// Network query methods (moved from NetworkSourceManager)
vector<string> GraphProcessor::getAllNodes() const {
    vector<string> result;
    const auto& nodes = roadNetwork.getNodes();
    for (const auto& pair : nodes) {
        result.push_back(pair.first);
    }
    return result;
}

vector<string> GraphProcessor::getAllEdges() const {
    vector<string> result;
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

vector<string> GraphProcessor::getEdgesFromNode(const string& nodeId) const {
    vector<string> result;
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

vector<string> GraphProcessor::getConnectedEdges(const string& edgeId) const {
    vector<string> result;
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

string GraphProcessor::getEdgeSource(const string& edgeId) const {
    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getFrom();
    }
    return "";
}

string GraphProcessor::getEdgeTarget(const string& edgeId) const {
    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getTo();
    }
    return "";
}

double GraphProcessor::getEdgeLength(const string& edgeId) const {
    const Edge* edge = findEdge(edgeId);
    if (edge) {
        return edge->getLength();
    }
    return 0.0;
}

const Edge* GraphProcessor::findEdge(const string& edgeId) const {
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
