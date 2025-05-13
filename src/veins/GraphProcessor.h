#ifndef GRAPH_PROCESSOR_H
#define GRAPH_PROCESSOR_H

#include <string>
#include <vector>
#include <map>
#include "Graph.h"

using namespace std;

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
    vector<string> findShortestPath(string sourceId, string targetId) const;
    
    /**
     * Calculate the length of the shortest path between source and target nodes
     * @param sourceId The ID of the source node
     * @param targetId The ID of the target node
     * @return The length of the shortest path, -1 if no path exists
     */
    double getShortestPathLength(string sourceId, string targetId) const;
    
    /**
     * Find k shortest paths between source and target nodes
     * @param sourceId The ID of the source node
     * @param targetId The ID of the target node
     * @param k The number of paths to find
     * @return A vector of paths, each path is a vector of road IDs
     */
    vector<vector<string>> findKShortestPaths(
        string sourceId, string targetId, int k) const;
    
    /**
     * Check if there exists a valid assignment between sources and targets
     * @param sources Vector of source node IDs
     * @param targets Vector of target node IDs
     * @return True if there exists a valid assignment
     */
    bool existsValidAssignment(
        const vector<string>& sources,
        const vector<string>& targets) const;
    
    /**
     * Get the underlying graph
     * @return The road network graph
     */
    const Graph& getGraph() const { return roadNetwork; }
    
    /**
     * Find shortest path between two edges
     * @param sourceEdgeId ID of the source edge
     * @param targetEdgeId ID of the target edge
     * @return A vector of edge IDs forming the shortest path
     */
    vector<string> findEdgeShortestPath(string sourceEdgeId, string targetEdgeId) const;
    
    /**
     * Cấu trúc biểu diễn một đoạn đường đi qua làn đường cụ thể
     */
    struct LanePath {
        string edgeId;
        int laneIndex;
        double cost;
    };
    
    /**
     * Tìm đường đi ngắn nhất giữa hai làn đường
     * @param sourceLaneId ID của làn đường nguồn
     * @param targetLaneId ID của làn đường đích
     * @return Danh sách các đoạn đường đi qua các làn
     */
    vector<LanePath> findLaneShortestPath(string sourceLaneId, string targetLaneId) const;
    
private:
    // Reference to the road network graph
    const Graph& roadNetwork;
    
    // Struct to represent a node in the priority queue for Dijkstra's algorithm
    struct DijkstraNode {
        string id;
        double distance;
        
        bool operator>(const DijkstraNode& other) const {
            return distance > other.distance;
        }
    };
    
    // Helper method to implement Dijkstra's algorithm
    map<string, pair<double, string>> dijkstra(string sourceId) const;
    
    // Helper method to reconstruct a path from Dijkstra's algorithm results
    vector<string> reconstructPath(
        const map<string, pair<double, string>>& dijkstraResult,
        string sourceId,
        string targetId) const;
    
    // Helper method to implement Hungarian algorithm for assignment problem
    bool hungarianAlgorithm(const vector<vector<double>>& costMatrix) const;
    
    // Helper methods for lane path finding
    string extractEdgeIdFromLane(string laneId) const;
    int extractLaneIndexFromLane(string laneId) const;
    int findBestLaneForEdge(string edgeId) const;
};

} // namespace veins

#endif // GRAPH_PROCESSOR_H 
