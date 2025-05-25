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
    vector<string> findShortestPath(string sourceId, string targetId) const;
    double getShortestPathLength(string sourceId, string targetId) const;

    vector<vector<string>> findKShortestPaths(
        string sourceId, string targetId, int k) const;
    bool existsValidAssignment(
        const vector<string>& sources,
        const vector<string>& targets) const;
    const Graph& getGraph() const { return roadNetwork; }
    vector<string> findEdgeShortestPath(string sourceEdgeId, string targetEdgeId) const;
    double getEdgeShortestPathLength(string sourceEdgeId, string targetEdgeId) const;
    struct LanePath {
        string edgeId;
        int laneIndex;
        double cost;
    };
    vector<LanePath> findLaneShortestPath(string sourceLaneId, string targetLaneId) const;

    vector<int> getOptimalVehicleAssignment(
        const vector<string>& sourceNodes,
        const vector<string>& destNodes) const;

private:
    const Graph& roadNetwork;
    struct DijkstraNode {
        string id;
        double distance;
        bool operator>(const DijkstraNode& other) const {
            return distance > other.distance;
        }
    };
    map<string, pair<double, string>> dijkstra(string sourceId) const;
    vector<string> reconstructPath(
        const map<string, pair<double, string>>& dijkstraResult,
        string sourceId,
        string targetId) const;
    bool hungarianAlgorithm(const vector<vector<double>>& costMatrix) const;
    string extractEdgeIdFromLane(string laneId) const;
    int extractLaneIndexFromLane(string laneId) const;
    int findBestLaneForEdge(string edgeId) const;
};

} // namespace veins

#endif
