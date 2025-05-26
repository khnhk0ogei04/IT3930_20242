#include "TaskGenerator.h"
#include <algorithm>    // For std::shuffle
#include <set>         // For std::set
#include <iostream>    // For std::cout

namespace veins {

TaskGenerator::TaskGenerator(const GraphProcessor& processor)
    : graphProcessor(processor) {
    // Initialize random number generator with a time-based seed
    rng.seed(std::random_device()());
}

std::vector<Destination> TaskGenerator::generateDestinations(int n, unsigned seedValue) {
    std::vector<Destination> destinations;

    // Seed the random number generator if a seed value is provided
    if (seedValue > 0) {
        rng.seed(seedValue);
    }

    // Get all nodes from the graph
    const auto& graph = graphProcessor.getGraph();
    const auto& nodes = graph.getNodes();

    // Convert nodes map to vector for random selection
    std::vector<std::string> nodeIds;
    for (const auto& nodePair : nodes) {
        nodeIds.push_back(nodePair.first);
    }
    
    // Check if we have enough nodes
    if (nodeIds.size() < n) {
        return destinations;  // Not enough nodes
    }

    // Uniform distribution for selecting nodes
    std::uniform_int_distribution<size_t> nodeDistribution(0, nodeIds.size() - 1);

    // Uniform distributions for time windows
    std::uniform_real_distribution<double> earlinessDistribution(0.0, 100.0);
    std::uniform_real_distribution<double> durationDistribution(50.0, 200.0);
    
    // Generate n random destinations
    for (int i = 0; i < n; ++i) {
        // Select a random node
        size_t nodeIndex = nodeDistribution(rng);
        std::string nodeId = nodeIds[nodeIndex];

        // Generate a random time window
        double earliness = earlinessDistribution(rng);
        double tardiness = earliness + durationDistribution(rng);

        // Create a destination with the selected node and time window
        destinations.emplace_back(nodeId, TimeWindow(earliness, tardiness));

        // Remove the selected node to avoid duplicates
        nodeIds.erase(nodeIds.begin() + nodeIndex);
    }
    
    return destinations;
}

std::vector<std::vector<std::string>> TaskGenerator::findKPaths(
    const std::string& sourceId,
    const std::string& destinationId,
    int k) {

    return graphProcessor.findKShortestPaths(sourceId, destinationId, k);
}

bool TaskGenerator::existsValidAssignment(
    const std::vector<std::string>& sources,
    const std::vector<std::string>& destinations) {

    return graphProcessor.existsValidAssignment(sources, destinations);
}

std::vector<std::string> TaskGenerator::getPotentialDestinationEdges(int n, const std::vector<std::string>& currentSourceEdges, unsigned seedValue) {
    std::vector<std::string> potentialDestEdges;
    const auto& graph = graphProcessor.getGraph(); // Get the graph from GraphProcessor

    if (seedValue > 0) {
        rng.seed(seedValue);
    }

    // Get all actual edges from the graph
    std::vector<std::string> allPossibleEdges;
    
    int edgeCounter = 0;
    for (const auto& pair : graph.getAdjList()) {
        for (const auto& edge : pair.second) {
            allPossibleEdges.push_back(edge.getId());
            if (++edgeCounter <= 10) {
                std::cout << "DEBUG: Found edge " << edge.getId() << " from "
                         << pair.first << " to " << edge.getTo() << std::endl;
            }
        }
    }

    if (allPossibleEdges.empty()) {
        std::cout << "ERROR: No edges found in the graph by TaskGenerator!" << std::endl;
        return potentialDestEdges; // Return empty if no edges
    }

    // Remove duplicates
    std::sort(allPossibleEdges.begin(), allPossibleEdges.end());
    allPossibleEdges.erase(std::unique(allPossibleEdges.begin(), allPossibleEdges.end()), allPossibleEdges.end());

    std::cout << "DEBUG: Found " << allPossibleEdges.size() << " unique edges in the graph" << std::endl;

    // Filter out source edges and keep only edges that are likely reachable
    std::vector<std::string> filteredEdges;
    std::set<std::string> sourceEdgeSet(currentSourceEdges.begin(), currentSourceEdges.end());

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
                        std::string srcEdgeToNode;
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
                        std::string destEdgeFromNode = nodePair.first;

                        // Check if there might be a path (same node means direct connection)
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

    if (filteredEdges.empty()) {
        for (const auto& edgeId : allPossibleEdges) {
            if (sourceEdgeSet.find(edgeId) == sourceEdgeSet.end()) {
                filteredEdges.push_back(edgeId);
            }
        }
    }

    // Shuffle the filtered edges
    std::shuffle(filteredEdges.begin(), filteredEdges.end(), rng);

    // Select n destination edges
    int selectedCount = 0;
    for (int i = 0; i < filteredEdges.size() && selectedCount < n; ++i) {
        potentialDestEdges.push_back(filteredEdges[i]);
        selectedCount++;
    }

    return potentialDestEdges;
}

} // namespace veins
