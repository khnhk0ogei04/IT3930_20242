#ifndef GRAPH_PROCESSOR_H
#define GRAPH_PROCESSOR_H

#include <string>
#include <vector>
#include <map>
#include "Graph.h"

namespace veins {

/**
 * Class for processing road network graphs.
 * Provides methods to find shortest paths.
 */
class GraphProcessor {
public:
    /**
     * Constructor
     */
    GraphProcessor(const Graph& graph);
    
    /**
     * Find the shortest path between source and target nodes
     * @param sourceId The ID of the source node
     * @param targetId The ID of the target node
     * @return A vector of road IDs forming the shortest path, empty if no path exists
     */
    std::vector<std::string> findShortestPath(const std::string& sourceId, const std::string& targetId) const;
    
    /**
     * Calculate the length of the shortest path between source and target nodes
     * @param sourceId The ID of the source node
     * @param targetId The ID of the target node
     * @return The length of the shortest path, -1 if no path exists
     */
    double getShortestPathLength(const std::string& sourceId, const std::string& targetId) const;
    
    /**
     * Find k shortest paths between source and target nodes
     * @param sourceId The ID of the source node
     * @param targetId The ID of the target node
     * @param k The number of paths to find
     * @return A vector of paths, each path is a vector of road IDs
     */
    std::vector<std::vector<std::string>> findKShortestPaths(
        const std::string& sourceId, const std::string& targetId, int k) const;
    
    /**
     * Check if there exists a valid assignment between sources and targets
     * @param sources Vector of source node IDs
     * @param targets Vector of target node IDs
     * @return True if there exists a valid assignment
     */
    bool existsValidAssignment(
        const std::vector<std::string>& sources, 
        const std::vector<std::string>& targets) const;
    
    /**
     * Get the underlying graph
     * @return The road network graph
     */
    const Graph& getGraph() const { return roadNetwork; }
    
private:
    // Reference to the road network graph
    const Graph& roadNetwork;
    
    // Struct to represent a node in the priority queue for Dijkstra's algorithm
    struct DijkstraNode {
        std::string id;
        double distance;
        
        bool operator>(const DijkstraNode& other) const {
            return distance > other.distance;
        }
    };
    
    // Helper method to implement Dijkstra's algorithm
    std::map<std::string, std::pair<double, std::string>> dijkstra(const std::string& sourceId) const;
    
    // Helper method to reconstruct a path from Dijkstra's algorithm results
    std::vector<std::string> reconstructPath(
        const std::map<std::string, std::pair<double, std::string>>& dijkstraResult,
        const std::string& sourceId, 
        const std::string& targetId) const;
    
    // Helper method to implement Hungarian algorithm for assignment problem
    bool hungarianAlgorithm(const std::vector<std::vector<double>>& costMatrix) const;
};

} // namespace veins

#endif // GRAPH_PROCESSOR_H 