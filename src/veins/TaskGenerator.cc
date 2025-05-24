#include "TaskGenerator.h"
#include <algorithm>
#include <set>
#include <iostream>

namespace veins {

TaskGenerator::TaskGenerator(const GraphProcessor& processor): graphProcessor(processor) {
    rng.seed(random_device()());
}

vector<Destination> TaskGenerator::generateDestinations(int n, unsigned seedValue) {
    vector<Destination> destinations;
    if (seedValue > 0) {
        rng.seed(seedValue);
    }
    const auto& graph = graphProcessor.getGraph();
    const auto& nodes = graph.getNodes();
    vector<string> nodeIds;
    for (const auto& nodePair : nodes) {
        nodeIds.push_back(nodePair.first);
    }
    
    // Check if we have enough nodes
    if (nodeIds.size() < n) {
        return destinations;
    }
    uniform_int_distribution<size_t> nodeDistribution(0, nodeIds.size() - 1);
    
    // Uniform distributions for time windows
    std::uniform_real_distribution<double> earlinessDistribution(0.0, 100.0);
    std::uniform_real_distribution<double> durationDistribution(50.0, 200.0);
    
    // generate n random destinations
    for (int i = 0; i < n; ++i) {
        size_t nodeIndex = nodeDistribution(rng); // select random node
        string nodeId = nodeIds[nodeIndex];
        // generate random time window
        double earliness = earlinessDistribution(rng);
        double tardiness = earliness + durationDistribution(rng);
        destinations.emplace_back(nodeId, TimeWindow(earliness, tardiness));
        nodeIds.erase(nodeIds.begin() + nodeIndex);
    }
    
    return destinations;
}

vector<vector<string>> TaskGenerator::findKPaths(const string& sourceId, const string& destinationId, int k) {
    return graphProcessor.findKShortestPaths(sourceId, destinationId, k);
}

bool TaskGenerator::existsValidAssignment(const vector<string>& sources, const vector<string>& destinations) {
    return graphProcessor.existsValidAssignment(sources, destinations);
}

vector<string> TaskGenerator::getPotentialDestinationEdges(int n, const vector<string>& currentSourceEdges, unsigned seedValue) {
    vector<string> potentialDestEdges;
    const auto& graph = graphProcessor.getGraph(); // Get the graph from GraphProcessor

    if (seedValue > 0) {
        rng.seed(seedValue);
    }

    // Get all actual edges from the graph
    vector<string> allPossibleEdges;
    cout << "DEBUG: Getting all possible destination edges from graph" << endl;
    
    int edgeCounter = 0;
    for (const auto& pair : graph.getAdjList()) {
        for (const auto& edge : pair.second) {
            allPossibleEdges.push_back(edge.getId());
            if (++edgeCounter <= 10) {
                cout << "DEBUG: Found edge " << edge.getId() << " from "
                     << pair.first << " to " << edge.getTo() << endl;
            }
        }
    }

    if (allPossibleEdges.empty()) {
        cout << "ERROR: No edges found in the graph by TaskGenerator!" << endl;
        return potentialDestEdges; // Return empty if no edges
    }

    // Remove duplicates
    sort(allPossibleEdges.begin(), allPossibleEdges.end());
    allPossibleEdges.erase(unique(allPossibleEdges.begin(), allPossibleEdges.end()), allPossibleEdges.end());

    cout << "DEBUG: Found " << allPossibleEdges.size() << " unique edges in the graph" << endl;

    // Filter out source edges and keep only edges that are likely reachable
    vector<string> filteredEdges;
    set<string> sourceEdgeSet(currentSourceEdges.begin(), currentSourceEdges.end());
    
    // For each edge in the graph
    for (const auto& edgeId : allPossibleEdges) {
        // Skip if this is a source edge
        if (sourceEdgeSet.find(edgeId) != sourceEdgeSet.end()) {
            continue;
        }
        
        // Find edge information to check if it's likely reachable
        bool edgeFound = false;
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    edgeFound = true;

                    // Assume the edge is reachable if we found it
                    filteredEdges.push_back(edgeId);

                    // Try to verify if there might be a path from at least one source
                    for (const auto& srcEdge : currentSourceEdges) {
                        // Find source edge info
                        string srcEdgeToNode;
                        for (const auto& srcNodePair : graph.getAdjList()) {
                            for (const auto& srcEdgeObj : srcNodePair.second) {
                                if (srcEdgeObj.getId() == srcEdge) {
                                    srcEdgeToNode = srcEdgeObj.getTo();
                                    break;
                                }
                            }
                            if (!srcEdgeToNode.empty()) break;
                        }

                        // Find target edge info
                        string destEdgeFromNode = nodePair.first;

                        // Check if there might be a path (same node means direct connection)
                        if (srcEdgeToNode == destEdgeFromNode) {
                            cout << "DEBUG: Found potentially direct path: " << srcEdge
                                 << " -> " << edgeId << endl;
                            break;
                        }
                    }

                    break;
                }
            }
            if (edgeFound) break;
        }
    }
    cout << "DEBUG: Have " << filteredEdges.size() << " filtered edges after removing sources" << endl;
    shuffle(filteredEdges.begin(), filteredEdges.end(), rng);
    int selectedCount = 0;
    for (int i = 0; i < filteredEdges.size() && selectedCount < n; ++i) {
        potentialDestEdges.push_back(filteredEdges[i]);
        selectedCount++;
    }
    cout << "INFO: TaskGenerator selected " << potentialDestEdges.size() << " potential destination edges:" << endl;
    for (const auto& edge : potentialDestEdges) {
        cout << "  - " << edge << endl;
    }
    return potentialDestEdges;
}

} // namespace veins
