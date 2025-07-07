#ifndef GRAPH_PROCESSOR_H
#define GRAPH_PROCESSOR_H

#include <string>
#include <vector>
#include <map>
#include <queue>
#include <limits>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include "Graph.h"

using namespace std;

namespace veins {
class GraphProcessor {
public:
    GraphProcessor(const Graph& graph);
    virtual ~GraphProcessor();
    
    // Core functionality
    vector<string> findShortestPath(string sourceId, string targetId) const;
    double getShortestPathLength(string sourceId, string targetId) const;
    bool existsValidAssignment(
        const vector<string>& sources,
        const vector<string>& targets) const;
    const Graph& getGraph() const { return roadNetwork; }
    
    vector<string> findEdgeShortestPath(string sourceEdgeId, string targetEdgeId) const;
    double getEdgeShortestPathLength(string sourceEdgeId, string targetEdgeId) const;
    vector<int> getOptimalVehicleAssignment(
        const vector<string>& sourceEdges,
        const vector<string>& destEdges) const;
    
    // Hungarian algorithm with custom cost matrix
    vector<int> getOptimalAssignmentWithMatrix(
        const vector<vector<double>>& costMatrix) const;
    
    // Core algorithms - made public for VehicleControlApp access
    map<string, pair<double, string>> dijkstra(string sourceId) const;
    vector<string> reconstructPath(
        const map<string, pair<double, string>>& dijkstraResult,
        string sourceId,
        string targetId) const;

private:
    const Graph& roadNetwork;
    struct DijkstraNode {
        string id;
        double distance;
        bool operator>(const DijkstraNode& other) const {
            return distance > other.distance;
        }
    };
    
    bool hungarianAlgorithm(const vector<vector<double>>& costMatrix) const;
    
    // Essential helper methods
    string extractEdgeIdFromLane(string laneId) const;
    const Edge* findEdge(const string& edgeId) const;
};

} // namespace veins

#endif
