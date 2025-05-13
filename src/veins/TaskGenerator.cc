#include "TaskGenerator.h"

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

} // namespace veins 
