#ifndef NETWORK_SOURCE_MANAGER_H
#define NETWORK_SOURCE_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include "Graph.h"
#include "Edge.h"
#include "Node.h"

using namespace std;
namespace veins {

class NetworkSourceManager {
public:
    NetworkSourceManager();
    ~NetworkSourceManager() = default;
    bool loadNetwork(string filename);
    vector<string> getAllNodes() const;
    vector<string> getAllEdges() const;
    vector<string> getEdgesFromNode(string nodeId) const;
    vector<string> getConnectedEdges(string edgeId) const;
    string getEdgeSource(string edgeId) const;
    string getEdgeTarget(string edgeId) const;
    double getEdgeLength(string edgeId) const;
    bool isNetworkLoaded() const { return networkLoaded; }
    const Graph& getGraph() const { return roadNetwork; }

private:
    Graph roadNetwork;
    bool networkLoaded;
    const Edge* findEdge(string edgeId) const;
};

} // namespace veins

#endif // NETWORK_SOURCE_MANAGER_H
