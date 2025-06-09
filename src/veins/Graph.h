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
    void addNode(string id, double x = 0.0, double y = 0.0);
    void addEdge(string from, string to, double length, string id = "");
    void addEdge(const Edge& edge);
    size_t getNodeCount() const;
    size_t getEdgeCount() const;
    const unordered_map<string, Node>& getNodes() const {
        return nodes;
    }
    const unordered_map<string, vector<Edge>>& getAdjList() const {
        return adjList;
    }
    Edge getEdge(string edgeId) const;
    static vector<Edge> getRoadsFromXml(string filePath);
    static unordered_map<string, Node> getJunctionsFromXml(string filePath);

private:
    unordered_map<string, Node> nodes;
    unordered_map<string, vector<Edge>> adjList;
    unordered_map<string, Edge> edges;
    size_t edgeCount = 0;
};

bool parseNetXml(string filePath, Graph& graph);

#endif // GRAPH_H
