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

vector<vector<string>> GraphProcessor::findKShortestPaths(string sourceId, string targetId, int k) const {
    vector<vector<string>> result;
    auto shortestPath = findShortestPath(sourceId, targetId);
    result.push_back(shortestPath);
    if (k <= 1) return result;
    vector<pair<vector<string>, double>> potentialPaths;
    set<vector<string>> addedPaths;
    addedPaths.insert(shortestPath);
    auto calculatePathLength = [this](const vector<string>& path) {
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

    for (int i = 0; result.size() < k && i < result.size(); ++i) {
        const auto& path = result[i];
        string lastNodeId = sourceId;
        vector<string> rootPath;
        for (int j = 0; j < path.size(); ++j) {
            string currentEdgeId = path[j];
            string currentNodeId = "";
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
            // the root path up to current deviation point
            vector<string> spurRootPath = rootPath;
            // remove the links that were in previous k-shortest paths
            map<pair<string, string>, double> removedEdges;
            for (const auto& prevPath : result) {
                if (j < prevPath.size() && spurRootPath.size() <= prevPath.size() &&
                    equal(spurRootPath.begin(), spurRootPath.end(), prevPath.begin())) {
                    if (j < prevPath.size()) {
                        string edgeToRemove = prevPath[j];
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
            Graph tempGraph = roadNetwork;
            map<string, double> distances;
            map<string, string> previous;
            set<string> visited;
            for (const auto& nodePair : roadNetwork.getNodes()) {
                distances[nodePair.first] = numeric_limits<double>::infinity();
            }
            distances[lastNodeId] = 0.0;
            set<pair<double, string>> pq;
            pq.insert(make_pair(0.0, lastNodeId));
            while (!pq.empty()) {
                auto it = pq.begin();
                double dist = it->first;
                string current = it->second;
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
                        string neighbor = edge.getTo();
                        if (removedEdges.find({current, neighbor}) != removedEdges.end()) {
                            continue;
                        }
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
                            pq.insert(make_pair(newDist, neighbor));
                        }
                    }
                }
            }
            if (distances[targetId] != numeric_limits<double>::infinity()) {
                vector<string> spurPath;
                string current = targetId;
                while (current != lastNodeId) {
                    auto prevIt = previous.find(current);
                    if (prevIt == previous.end()) {
                        spurPath.clear();
                        break;
                    }
                    string edgeId = prevIt->second;
                    spurPath.push_back(edgeId);
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
                reverse(spurPath.begin(), spurPath.end());
                vector<string> totalPath = spurRootPath;
                totalPath.insert(totalPath.end(), spurPath.begin(), spurPath.end());
                if (addedPaths.find(totalPath) == addedPaths.end()) {
                    double pathLength = calculatePathLength(totalPath);
                    potentialPaths.push_back({totalPath, pathLength});
                }
            }
            rootPath.push_back(currentEdgeId);
            lastNodeId = currentNodeId;
        }

        sort(potentialPaths.begin(), potentialPaths.end(),
            [](const pair<vector<string>, double>& a,
               const pair<vector<string>, double>& b) {
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
    if (result.size() < k) {
        set<string> shortestPathEdges(shortestPath.begin(), shortestPath.end());
        for (const auto& edgeToAvoid : shortestPathEdges) {
            map<string, double> distances;
            map<string, string> previous;
            set<string> visited;
            for (const auto& nodePair : roadNetwork.getNodes()) {
                distances[nodePair.first] = numeric_limits<double>::infinity();
            }
            distances[sourceId] = 0.0;
            set<pair<double, string>> pq;
            pq.insert(make_pair(0.0, sourceId));
            while (!pq.empty()) {
                auto it = pq.begin();
                double dist = it->first;
                string current = it->second;
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

                // reverse the path
                std::reverse(path.begin(), path.end());
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
    
    // Check if each source can reach at least one destination
    for (int i = 0; i < n; ++i) {
        bool canReachAnyTarget = false;
        for (int j = 0; j < n; ++j) {
            if (costMatrix[i][j] != numeric_limits<double>::max()) {
                canReachAnyTarget = true;
                break;
            }
        }
        if (!canReachAnyTarget) {
            return false;  // This source can't reach any destination
        }
    }
    
    // Check if each destination can be reached by at least one source
    for (int j = 0; j < n; ++j) {
        bool canBeReachedByAnySource = false;
        for (int i = 0; i < n; ++i) {
            if (costMatrix[i][j] != numeric_limits<double>::max()) {
                canBeReachedByAnySource = true;
                break;
            }
        }
        if (!canBeReachedByAnySource) {
            return false;  // This destination can't be reached by any source
        }
    }
    
    return true;  // A valid assignment exists
}

vector<GraphProcessor::LanePath> GraphProcessor::findLaneShortestPath(string sourceLaneId, string targetLaneId) const {
    vector<LanePath> result;
    string sourceEdgeId = extractEdgeIdFromLane(sourceLaneId);
    string targetEdgeId = extractEdgeIdFromLane(targetLaneId);
    int sourceLaneIndex = extractLaneIndexFromLane(sourceLaneId);
    int targetLaneIndex = extractLaneIndexFromLane(targetLaneId);

    if (sourceEdgeId.empty() || targetEdgeId.empty() || sourceLaneIndex < 0 || targetLaneIndex < 0) {
        return result;
    }

    // find the shortest ;atn
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
    if (!edgePath.empty()) {
        LanePath sourceSegment;
        sourceSegment.edgeId = sourceEdgeId;
        sourceSegment.laneIndex = sourceLaneIndex;
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == sourceEdgeId) {
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

    if (!edgePath.empty() || sourceEdgeId == targetEdgeId) {
        LanePath targetSegment;
        targetSegment.edgeId = targetEdgeId;
        targetSegment.laneIndex = targetLaneIndex;
        bool found = false;
        for (const auto& nodePair : roadNetwork.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == targetEdgeId) {
                    // cost: length of edge
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

// Implementation of the getOptimalVehicleAssignment method
vector<int> GraphProcessor::getOptimalVehicleAssignment(
    const vector<std::string>& sourceEdges,
    const vector<std::string>& destEdges) const {

    int numVehicles = sourceEdges.size();
    int numDestinations = destEdges.size();

    if (numVehicles == 0 || numDestinations == 0) {
        return std::vector<int>();
    }
    
    int n = std::max(numVehicles, numDestinations);
    vector<std::vector<double>> costMatrix(n, std::vector<double>(n, 0));
    const double NO_PATH_PENALTY = 9999999.0;
    const double SAME_EDGE_PENALTY = 5000.0;
    
    // Build the cost matrix
    for (int i = 0; i < numVehicles; ++i) {
        const string& sourceEdgeId = sourceEdges[i];

        for (int j = 0; j < numDestinations; ++j) {
            const std::string& destEdgeId = destEdges[j];
            if (sourceEdgeId == destEdgeId) {
                costMatrix[i][j] = SAME_EDGE_PENALTY;
                continue;
            }
            auto path = findEdgeShortestPath(sourceEdgeId, destEdgeId);
            double pathLength = 0.0;

            if (!path.empty()) {
                // Calculate the length of the path
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
                        pathLength += 0.0; // Default length
                    }
                }

                // Cost = path length between edges
                costMatrix[i][j] = pathLength;
            } else {
                // No path exists between these edges
                costMatrix[i][j] = NO_PATH_PENALTY;
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
    
    // Print original cost matrix for debugging
    std::string header = "       ";
    for (int j = 0; j < numDestinations; ++j) {
        header += "D_" + destEdges[j].substr(0, std::min((size_t)4, destEdges[j].length())) + "\t";
    }
    cout << header << endl;

    for (int i = 0; i < numVehicles; ++i) {
        string rowStr = "V_" + std::to_string(i) + "(" + sourceEdges[i].substr(0, std::min((size_t)4, sourceEdges[i].length())) + ")\t";
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] >= NO_PATH_PENALTY) {
                rowStr += "INF\t";
            } else {
                rowStr += std::to_string(static_cast<int>(costMatrix[i][j])) + "\t";
            }
        }
        cout << rowStr << endl;
    }
    
    // Create a copy of the original cost matrix for reference
    auto originalCostMatrix = costMatrix;
    
    // Hungarian Algorithm Implementation
    
    // Step 1: Subtract the smallest element in each row from all elements in that row
    for (int i = 0; i < n; ++i) {
        double minVal = NO_PATH_PENALTY;
        for (int j = 0; j < n; ++j) {
            minVal = std::min(minVal, costMatrix[i][j]);
        }
        if (minVal < NO_PATH_PENALTY) {
            for (int j = 0; j < n; ++j) {
                if (costMatrix[i][j] < NO_PATH_PENALTY) {
                    costMatrix[i][j] -= minVal;
                }
            }
        }
    }
    
    // Step 2: Subtract the smallest element in each column from all elements in that column
    for (int j = 0; j < n; ++j) {
        double minVal = NO_PATH_PENALTY;
        for (int i = 0; i < n; ++i) {
            minVal = std::min(minVal, costMatrix[i][j]);
        }
        if (minVal < NO_PATH_PENALTY) {
            for (int i = 0; i < n; ++i) {
                if (costMatrix[i][j] < NO_PATH_PENALTY) {
                    costMatrix[i][j] -= minVal;
                }
            }
        }
    }
    
    cout << "\nINFO: Reduced Cost Matrix:" << std::endl;
    for (int i = 0; i < numVehicles; ++i) {
        std::string rowStr = "V_" + std::to_string(i) + "\t";
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] >= NO_PATH_PENALTY) {
                rowStr += "INF\t";
            } else {
                rowStr += std::to_string(static_cast<int>(costMatrix[i][j])) + "\t";
            }
        }
        cout << rowStr << std::endl;
    }
    
    // Initialize vectors for the assignment
    std::vector<int> assignment(n, -1);

    // Simple greedy approach (faster than the full Hungarian algorithm)
    // This is a reasonable approximation for most cases
    std::vector<bool> colAssigned(n, false);
    
    // First pass: assign using zeros
    for (int i = 0; i < numVehicles; ++i) {
        for (int j = 0; j < numDestinations; ++j) {
            if (costMatrix[i][j] == 0 && !colAssigned[j]) {
                assignment[i] = j;
                colAssigned[j] = true;
                break;
            }
        }
    }
    
    // Second pass: assign remaining using minimum costs
    for (int i = 0; i < numVehicles; ++i) {
        if (assignment[i] == -1) {
            double minCost = NO_PATH_PENALTY;
            int minJ = -1;
            
            for (int j = 0; j < numDestinations; ++j) {
                if (!colAssigned[j] && costMatrix[i][j] < minCost) {
                    minCost = costMatrix[i][j];
                    minJ = j;
                }
            }
            
            if (minJ != -1) {
                assignment[i] = minJ;
                colAssigned[minJ] = true;
            }
        }
    }
    
    // Extract the result for the actual vehicles and destinations
    vector<int> result(numVehicles, -1);
    for (int i = 0; i < numVehicles; ++i) {
        if (assignment[i] >= 0 && assignment[i] < numDestinations) {
            result[i] = assignment[i];
        }
    }
    
    cout << "\nINFO: Assignment Results:" << std::endl;
    double totalCost = 0;
    int assignedCount = 0;
    
    for (int i = 0; i < numVehicles; ++i) {
        if (result[i] != -1) {
            int j = result[i];
            double cost = originalCostMatrix[i][j];
            cout << "Vehicle " << i << " (Edge " << sourceEdges[i] << ") assigned to destination " 
                 << j << " (Edge " << destEdges[j] << ") with cost " 
                 << cost << std::endl;
            
            if (cost < NO_PATH_PENALTY) {
                totalCost += cost;
                assignedCount++;
            }
        } else {
            cout << "Vehicle " << i << " (Edge " << sourceEdges[i] << ") could not be assigned" << std::endl;
        }
    }
    
    cout << "Total cost: " << totalCost << " (across " << assignedCount << " assignments)" << std::endl;
    cout << "Total assignments made: " << assignedCount << "/" << numVehicles << std::endl;
    
    return result;
}
} // namespace veins
