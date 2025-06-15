#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "GraphProcessor.h"
#include "TaskGenerator.h"
#include "SimulationLogger.h"
#include <string>
#include <vector>
#include <map>
#include <chrono>

using namespace omnetpp;
using namespace std;

namespace veins {
class DemoSafetyMessage;
class DemoServiceAdvertisment;

class VehicleControlApp : public TraCIDemo11p {
public:
    void initialize(int stage) override;
    void finish() override;

protected:
    virtual void onWSM(BaseFrame1609_4* wsm) override;
    virtual void onBSM(DemoSafetyMessage* bsm) override;
    virtual void onWSA(DemoServiceAdvertisment* wsa) override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void handlePositionUpdate(cObject* obj) override;
    virtual void handleLowerMsg(cMessage* msg) override;
    virtual void handleEndOfVehicleLifecycle();
    
private:
    cMessage* statusUpdateMsg;
    cMessage* requestRoadInfoMsg;
    cMessage* cleanupTimer;

    int myInternalId;       // the internal ID used by Veins/OMNeT++
    int mySimulationId;     // the simulation ID visible in the TreeView
    bool hasLoggedDeparture = false;
    double startTime;
    double endTime;
    double earliestArrival;
    double latestArrival;
    string startingRoad;
    string targetRoad;
    double pathLength;
    
    string currentRoadId;
    vector<string> allRoads;
    vector<string> accessibleRoads;
    vector<string> incomingRoads;
    map<string, string> currentRoadAttributes;
    vector<string> currentPath;
    vector<Destination> destinations;

    unique_ptr<GraphProcessor> graphProcessor;
    Graph roadNetwork;

    void sendStatusUpdate();
    void requestAllRoads();
    void requestAccessibleRoads(const string& roadId);
    void requestIncomingRoads(const string& roadId);
    void requestRoadAttributes(const string& roadId);
    void requestShortestPath(const string& sourceId, const string& targetId);
    void requestDestinations(int count);
    void requestValidAssignment(const vector<string>& sources, const vector<string>& destinations);

    void processAllRoadsResponse(const string& data);
    void processAccessibleRoadsResponse(const string& data);
    void processIncomingRoadsResponse(const string& data);
    void processRoadAttributesResponse(const string& data);
    void processShortestPathResponse(const string& data);
    void processDestinationsResponse(const string& data);
    void processValidAssignmentResponse(const string& data);

    // Helper methods
    vector<string> parseRoadList(const string& data, char delimiter = ',');
    map<string, string> parseAttributes(const string& data);
    void printRoadInfo();
    void buildLocalRoadNetwork();
    
    vector<string> findShortestPath(const string& sourceId, const string& targetId);
    double getShortestPathLength(const string& sourceId, const string& targetId);

    void cleanupMessages();
    void logDepartureIfNeeded();
    void checkVehicleStatus();
    void updateTimingFile(double endTime);
    // Đã loại bỏ phương thức measureRoutingAlgorithmTime không còn sử dụng
};

} // namespace veins
