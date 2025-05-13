#ifndef NETWORK_SOURCE_MANAGER_H
#define NETWORK_SOURCE_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include "Graph.h"
#include "Edge.h"
#include "Node.h"

namespace veins {

/**
 * Manages network source files and provides a simplified interface
 * for working with road network data.
 */
class NetworkSourceManager {
public:
    /**
     * Constructor
     */
    NetworkSourceManager();

    /**
     * Destructor
     */
    ~NetworkSourceManager() = default;

    /**
     * Load a network file
     * @param filename The path to the network file
     * @return True if loading was successful
     */
    bool loadNetwork(string filename);

    /**
     * Get all nodes in the network
     * @return A vector of node IDs
     */
    std::vector<std::string> getAllNodes() const;

    /**
     * Get all edges in the network
     * @return A vector of edge IDs
     */
    std::vector<std::string> getAllEdges() const;

    /**
     * Get all edges connected to a specific node
     * @param nodeId The ID of the node
     * @return A vector of edge IDs
     */
    std::vector<std::string> getEdgesFromNode(string nodeId) const;

    /**
     * Get all edges that can be accessed directly from a specific edge
     * @param edgeId The ID of the edge
     * @return A vector of edge IDs
     */
    std::vector<std::string> getConnectedEdges(string edgeId) const;

    /**
     * Get the source node of an edge
     * @param edgeId The ID of the edge
     * @return The ID of the source node
     */
    std::string getEdgeSource(string edgeId) const;

    /**
     * Get the target node of an edge
     * @param edgeId The ID of the edge
     * @return The ID of the target node
     */
    std::string getEdgeTarget(string edgeId) const;

    /**
     * Get the length of an edge
     * @param edgeId The ID of the edge
     * @return The length of the edge
     */
    double getEdgeLength(string edgeId) const;

    /**
     * Check if the network is loaded
     * @return True if the network is loaded
     */
    bool isNetworkLoaded() const { return networkLoaded; }
    
    /**
     * Get the underlying graph for advanced operations
     * @return A reference to the graph
     */
    const Graph& getGraph() const { return roadNetwork; }

private:
    // The road network graph
    Graph roadNetwork;

    // Flag indicating if the network is loaded
    bool networkLoaded;

    // Helper method to find an edge by ID
    const Edge* findEdge(string edgeId) const;
};

} // namespace veins

#endif // NETWORK_SOURCE_MANAGER_H 
