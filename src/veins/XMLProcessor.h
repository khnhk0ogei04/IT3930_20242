#ifndef XMLPROCESSOR_H
#define XMLPROCESSOR_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include "Graph.h"
#include "tinyxml2.h"

using namespace std;
namespace veins {

struct VehicleInfo {
    std::string id;
    double depart;
    std::string from;
    std::string to;
    std::string via; // Can be empty
};

class XMLProcessor {
public:
    XMLProcessor();
    ~XMLProcessor() = default;
    
    bool loadNetworkFile(const std::string& filePath);
    std::vector<std::string> getAllRoads() const;
    std::vector<std::string> getAccessibleRoads(const std::string& roadId) const;
    std::vector<std::string> getIncomingRoads(const std::string& roadId) const;
    std::map<std::string, std::string> getRoadAttributes(const std::string& roadId) const;

    bool loadRouteFile(const std::string& filePath);
    std::vector<VehicleInfo> getVehicles() const;

    bool isNetworkLoaded() const { return networkLoaded; }
    const Graph& getGraph() const { return roadNetwork; }
    
private:
    Graph roadNetwork;
    bool networkLoaded;
    bool parseNetXml(const std::string& filePath, Graph& graph);
    
    std::map<std::string, std::map<std::string, std::string>> roadAttributes;
    std::map<std::string, std::vector<std::string>> incomingRoadsMap;
    
    std::map<std::string, std::string> extractAttributes(const tinyxml2::XMLElement* element) const;
    void buildIncomingRoadsMap();
    vector<VehicleInfo> vehicleData;
};

} // namespace veins

#endif // XMLPROCESSOR_H
