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

//std::vector<std::vector<std::string>> TaskGenerator::findKPaths(
//    const std::string& sourceId,
//    const std::string& destinationId,
//    int k) {
//
//    return graphProcessor.findKShortestPaths(sourceId, destinationId, k);
//}

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

    // Create a set of excluded edges (source edges and their opposite directions)
    std::set<std::string> excludedEdges;
    for (const auto& srcEdge : currentSourceEdges) {
        excludedEdges.insert(srcEdge);
        
        // Check if this is a directed edge (has a minus sign version)
        if (srcEdge[0] != '-') {
            // Add the opposite direction to excluded list
            excludedEdges.insert("-" + srcEdge);
        } else {
            // If it already has a minus sign, add the version without it
            excludedEdges.insert(srcEdge.substr(1));
        }
    }
    
    // Vector to store valid destination candidates
    std::vector<std::string> validDestinations;
    
    // Shuffle the possible edges to randomize selection order
    std::vector<std::string> shuffledEdges = allPossibleEdges;
    std::shuffle(shuffledEdges.begin(), shuffledEdges.end(), rng);
    
    // For each possible edge, check if it's a valid destination
    for (const auto& destEdge : shuffledEdges) {
        // Skip if this is a source edge or its opposite direction
        if (excludedEdges.find(destEdge) != excludedEdges.end()) {
            continue;
        }
        
        bool isValidForAll = true;
        bool isValidForAny = false;
        
        // Check path existence from each source edge
        for (const auto& srcEdge : currentSourceEdges) {
            // Use GraphProcessor to check if a path exists
            auto path = graphProcessor.findEdgeShortestPath(srcEdge, destEdge);
            if (!path.empty()) {
                isValidForAny = true;
                
                // Optional: If you need all sources to have a path to this destination
                // keep isValidForAll true, otherwise set to false
                // For now, we only require at least one source to have a valid path
            }
        }
        if (isValidForAny) {
            validDestinations.push_back(destEdge);
        }
        
    }
    std::shuffle(validDestinations.begin(), validDestinations.end(), rng);
    for (int i = 0; i < std::min(static_cast<size_t>(n), validDestinations.size()); ++i) {
        potentialDestEdges.push_back(validDestinations[i]);
    }
    std::cout << "INFO: Selected " << potentialDestEdges.size() << " valid destinations" << std::endl;
    return potentialDestEdges;
}

} // namespace veins
