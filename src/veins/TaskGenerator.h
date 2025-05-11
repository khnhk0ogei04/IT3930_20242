#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include <string>
#include <vector>
#include <random>
#include "GraphProcessor.h"

namespace veins {

/**
 * Structure to represent a time window
 */
struct TimeWindow {
    double earliness;  // Earliest arrival time
    double tardiness;  // Latest arrival time
    
    TimeWindow(double early = 0.0, double late = 0.0)
        : earliness(early), tardiness(late) {}
};

/**
 * Structure to represent a destination with a time window
 */
struct Destination {
    std::string nodeId;  // Node ID
    TimeWindow timeWindow;  // Time window for arrival
    
    Destination(const std::string& id = "", const TimeWindow& tw = TimeWindow())
        : nodeId(id), timeWindow(tw) {}
};

/**
 * Class for generating random destinations with time windows.
 */
class TaskGenerator {
public:
    /**
     * Constructor
     */
    TaskGenerator(const GraphProcessor& processor);
    
    /**
     * Generate N random destinations with time windows
     * @param n Number of destinations to generate
     * @param seedValue Random seed (optional)
     * @return Vector of destinations with time windows
     */
    std::vector<Destination> generateDestinations(int n, unsigned seedValue = 0);
    
    /**
     * Find k shortest paths from source to destination
     * @param sourceId The ID of the source node
     * @param destinationId The ID of the destination node
     * @param k The number of paths to find
     * @return Vector of paths, each path is a vector of road IDs
     */
    std::vector<std::vector<std::string>> findKPaths(
        const std::string& sourceId, 
        const std::string& destinationId, 
        int k);
    
    /**
     * Check if there exists a valid assignment between sources and destinations
     * @param sources Vector of source node IDs
     * @param destinations Vector of destination node IDs
     * @return True if there exists a valid assignment
     */
    bool existsValidAssignment(
        const std::vector<std::string>& sources, 
        const std::vector<std::string>& destinations);
    
private:
    // Reference to the graph processor
    const GraphProcessor& graphProcessor;
    
    // Random number generator
    std::mt19937 rng;
};

} // namespace veins

#endif // TASK_GENERATOR_H 