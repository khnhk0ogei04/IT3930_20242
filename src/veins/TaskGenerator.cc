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
    
    if (nodeIds.size() < n) {
        return destinations;
    }
    uniform_int_distribution<size_t> nodeDistribution(0, nodeIds.size() - 1);
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
    const auto& graph = graphProcessor.getGraph();
    if (seedValue > 0) {
        rng.seed(seedValue);
    }
    vector<string> allPossibleEdges;
    cout << "DEBUG: Getting all possible destination edges from graph" << endl;
    
    int edgeCounter = 0;
    for (const auto& pair : graph.getAdjList()) {
        for (const auto& edge : pair.second) {
            allPossibleEdges.push_back(edge.getId());
        }
    }
    sort(allPossibleEdges.begin(), allPossibleEdges.end());
    allPossibleEdges.erase(unique(allPossibleEdges.begin(), allPossibleEdges.end()), allPossibleEdges.end());
    vector<string> filteredEdges;
    set<string> sourceEdgeSet(currentSourceEdges.begin(), currentSourceEdges.end());
    for (const auto& edgeId : allPossibleEdges) {
        if (sourceEdgeSet.find(edgeId) != sourceEdgeSet.end()) {
            continue;
        }
        
        bool edgeFound = false;
        for (const auto& nodePair : graph.getAdjList()) {
            for (const auto& edge : nodePair.second) {
                if (edge.getId() == edgeId) {
                    edgeFound = true;
                    filteredEdges.push_back(edgeId);
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
                        string destEdgeFromNode = nodePair.first;
                        if (srcEdgeToNode == destEdgeFromNode) {
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
