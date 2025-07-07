#include "TaskGenerator.h"
#include <random>
#include <algorithm>
#include <set>
#include <limits>
#include <iostream>

using namespace std;

namespace veins {

TaskGenerator::TaskGenerator(const GraphProcessor& processor)
    : graphProcessor(processor) {
    rng.seed(random_device()());
}

bool TaskGenerator::existsValidAssignment(
    const vector<string>& sources,
    const vector<string>& destinations) {

    return graphProcessor.existsValidAssignment(sources, destinations);
}

vector<string> TaskGenerator::getPotentialDestinationEdges(int n, const vector<string>& currentSourceEdges, unsigned seedValue) {
    vector<string> potentialDestEdges;
    const auto& graph = graphProcessor.getGraph();
    if (seedValue > 0) {
        rng.seed(seedValue);
    }

    vector<string> allPossibleEdges;
    
    for (const auto& pair : graph.getAdjList()) {
        for (const auto& edge : pair.second) {
            allPossibleEdges.push_back(edge.getId());
        }
    }

    if (allPossibleEdges.empty()) {
        return potentialDestEdges;
    }

    sort(allPossibleEdges.begin(), allPossibleEdges.end());
    allPossibleEdges.erase(unique(allPossibleEdges.begin(), allPossibleEdges.end()), allPossibleEdges.end());

    set<string> excludedEdges;
    for (const auto& srcEdge : currentSourceEdges) {
        excludedEdges.insert(srcEdge);
        
        if (srcEdge[0] != '-') {
            excludedEdges.insert("-" + srcEdge);
        } else {
            excludedEdges.insert(srcEdge.substr(1));
        }
    }
    
    vector<string> validDestinations;
    
    vector<string> shuffledEdges = allPossibleEdges;
    shuffle(shuffledEdges.begin(), shuffledEdges.end(), rng);
    
    for (const auto& destEdge : shuffledEdges) {
        if (excludedEdges.find(destEdge) != excludedEdges.end()) {
            continue;
        }
        
        bool isValidForAny = false;
        
        for (const auto& srcEdge : currentSourceEdges) {
            auto path = graphProcessor.findEdgeShortestPath(srcEdge, destEdge);
            if (!path.empty()) {
                isValidForAny = true;
            }
        }
        
        if (isValidForAny) {
            validDestinations.push_back(destEdge);
        }
    }
    
    shuffle(validDestinations.begin(), validDestinations.end(), rng);
    for (int i = 0; i < min(static_cast<size_t>(n), validDestinations.size()); ++i) {
        potentialDestEdges.push_back(validDestinations[i]);
    }
    
    return potentialDestEdges;
}

vector<Destination> TaskGenerator::generateDestinationsWithTimeWindows(
    int n, 
    const vector<string>& currentSourceEdges, 
    const Graph& graph,
    unsigned seedValue) {
    
    vector<Destination> destinations;
    
    if (seedValue > 0) {
        rng.seed(seedValue);
    }
    
    auto destEdges = getPotentialDestinationEdges(n, currentSourceEdges, seedValue);
    
    uniform_real_distribution<double> earlinessDistribution(20.0, 150.0);
    
    for (const auto& edgeId : destEdges) {
        double earliness = earlinessDistribution(rng);
        double tardiness = 1.5 * earliness;
        
        destinations.emplace_back(edgeId, TimeWindow(earliness, tardiness));
    }
    
    return destinations;
}

} // namespace veins
