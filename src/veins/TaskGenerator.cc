#include "TaskGenerator.h"
#include<bits/stdc++.h>

namespace veins {

TaskGenerator::TaskGenerator(const GraphProcessor& processor)
    : graphProcessor(processor) {
    // Initialize random number generator with a time-based seed
    rng.seed(std::random_device()());
}

bool TaskGenerator::existsValidAssignment(
    const vector<string>& sources,
    const vector<string>& destinations) {

    return graphProcessor.existsValidAssignment(sources, destinations);
}

std::vector<std::string> TaskGenerator::getPotentialDestinationEdges(int n, const std::vector<std::string>& currentSourceEdges, unsigned seedValue) {
    vector<string> potentialDestEdges;
    const auto& graph = graphProcessor.getGraph();
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
