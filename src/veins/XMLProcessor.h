#ifndef XMLPROCESSOR_H
#define XMLPROCESSOR_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include "Graph.h"
#include "tinyxml2.h"
#include "Vehicle.h"

using namespace std;
namespace veins {



class XMLProcessor {
public:
    XMLProcessor();
    ~XMLProcessor() = default;
    bool loadNetworkFile(const string& filePath);
    vector<string> getAllRoads() const;
    vector<string> getAccessibleRoads(const string& roadId) const;
    vector<string> getIncomingRoads(const string& roadId) const;
    map<string, string> getRoadAttributes(const string& roadId) const;
    bool loadRouteFile(const string& filePath);
    vector<Vehicle> getVehicles() const;
    bool isNetworkLoaded() const { return networkLoaded; }
    const Graph& getGraph() const { return roadNetwork; }
    static vector<Edge> getRoadsFromXml(const string& filePath);
    static unordered_map<string, Node> getJunctionsFromXml(const string& filePath);
private:
    Graph roadNetwork;
    bool networkLoaded;
    bool parseNetXml(const std::string& filePath, Graph& graph);
    map<string, map<string, string>> roadAttributes;
    map<string, vector<string>> incomingRoadsMap;
    map<string, string> extractAttributes(const tinyxml2::XMLElement* element) const;
    void buildIncomingRoadsMap();
    vector<Vehicle> vehicleData;
};

} // namespace veins

#endif // XMLPROCESSOR_H
