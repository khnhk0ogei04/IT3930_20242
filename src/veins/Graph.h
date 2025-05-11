#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <string>
#include "Node.h"
#include "Edge.h"

using namespace std;

class Graph {
public:
    void addNode(const string& id, double x = 0.0, double y = 0.0);
    void addEdge(const string& from, const string& to, double length, const string& id = "");
    void addEdge(const Edge& edge);
    size_t getNodeCount() const;
    size_t getEdgeCount() const;
    const unordered_map<string, Node>& getNodes() const { return nodes; }
    const unordered_map<string, vector<Edge>>& getAdjList() const {
        return adjList;
    }
    Edge getEdge(const string& edgeId) const;
    static vector<Edge> getRoadsFromXml(const string& filePath);
    static unordered_map<string, Node> getJunctionsFromXml(const string& filePath);
    void printNeighbors() const;

private:
    unordered_map<string, Node> nodes;
    unordered_map<string, vector<Edge>> adjList;
    unordered_map<string, Edge> edges;
    size_t edgeCount = 0;
};

bool parseNetXml(const string& filePath, Graph& graph);

#endif // GRAPH_H
