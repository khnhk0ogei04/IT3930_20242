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

std::vector<Destination> TaskGenerator::generateOptimalDestinations(
    const std::vector<std::string>& sourceNodes,
    int n, 
    unsigned seedValue) {
    
    std::vector<Destination> destinations;
    
    // Seed the random number generator if a seed value is provided
    if (seedValue > 0) {
        rng.seed(seedValue);
    }
    
    // Get all nodes from the graph
    const auto& graph = graphProcessor.getGraph();
    const auto& nodes = graph.getNodes();
    
    // Convert nodes map to vector for selection
    std::vector<std::string> nodeIds;
    for (const auto& nodePair : nodes) {
        // Skip nodes that are sources (vehicles shouldn't be assigned to their own location)
        bool isSource = false;
        for (const auto& source : sourceNodes) {
            if (nodePair.first == source) {
                isSource = true;
                break;
            }
        }
        if (!isSource) {
            nodeIds.push_back(nodePair.first);
        }
    }
    
    // Check if we have enough nodes
    if (nodeIds.size() < n) {
        return destinations;  // Not enough nodes
    }
    
    // Shuffle the node IDs for random selection
    if (seedValue > 0) {
        std::shuffle(nodeIds.begin(), nodeIds.end(), rng);
    } else {
        std::shuffle(nodeIds.begin(), nodeIds.end(), std::default_random_engine(std::random_device()()));
    }
    
    // Select n potential destinations
    std::vector<std::string> potentialDests;
    for (int i = 0; i < n && i < nodeIds.size(); ++i) {
        potentialDests.push_back(nodeIds[i]);
    }
    
    // Use the GraphProcessor's optimal assignment method
    std::vector<int> assignments = graphProcessor.getOptimalVehicleAssignment(sourceNodes, potentialDests);
    
    // Generate destinations based on the optimal assignment
    for (size_t i = 0; i < assignments.size() && i < sourceNodes.size(); ++i) {
        int destIndex = assignments[i];
        
        // Skip invalid assignments
        if (destIndex < 0 || destIndex >= potentialDests.size()) {
            continue;
        }
        
        std::string nodeId = potentialDests[destIndex];
        
        // Calculate estimated travel time for this assignment
        double distance = graphProcessor.getShortestPathLength(sourceNodes[i], nodeId);
        
        // Generate appropriate time window based on the distance
        TimeWindow timeWindow = generateTimeWindowForDistance(distance);
        
        // Create the destination
        destinations.emplace_back(nodeId, timeWindow);
    }
    
    return destinations;
}

TimeWindow TaskGenerator::generateTimeWindowForDistance(double distance) {
    // Default values if no path exists or distance is invalid
    if (distance <= 0) {
        // Uniform distributions for time windows when we don't have distance
        std::uniform_real_distribution<double> earlinessDistribution(0.0, 100.0);
        std::uniform_real_distribution<double> durationDistribution(50.0, 200.0);
        
        double earliness = earlinessDistribution(rng);
        double tardiness = earliness + durationDistribution(rng);
        
        return TimeWindow(earliness, tardiness);
    }
    
    // Calculate estimated travel time based on distance
    // Assuming average speed of 15 units per time unit
    double averageSpeed = 15.0;
    double estimatedTravelTime = distance / averageSpeed;
    
    // Add some randomness to the time window
    std::uniform_real_distribution<double> earlyBufferDist(0.0, estimatedTravelTime * 0.2); // 0-20% buffer
    std::uniform_real_distribution<double> lateBufferDist(0.1, 0.5); // 10-50% buffer for tardiness
    
    double earlyBuffer = earlyBufferDist(rng);
    double lateBuffer = estimatedTravelTime * lateBufferDist(rng);
    
    // Create time window
    double earliness = estimatedTravelTime - earlyBuffer;
    double tardiness = estimatedTravelTime + lateBuffer;
    
    // Ensure earliness is not negative
    earliness = std::max(0.0, earliness);
    
    return TimeWindow(earliness, tardiness);
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
