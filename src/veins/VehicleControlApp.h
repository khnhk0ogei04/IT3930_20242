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
    
private:
    cMessage* statusUpdateMsg;
    cMessage* cleanupTimer;

    int myInternalId;
    int mySimulationId;
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
    vector<string> currentPath;
    vector<Destination> destinations;

    unique_ptr<GraphProcessor> graphProcessor;
    Graph roadNetwork;

    void sendStatusUpdate();
    void requestAllRoads();
    void requestDestinations(int count);

    void processAllRoadsResponse(const string& data);
    void processDestinationsResponse(const string& data);

    // Helper methods
    vector<string> parseRoadList(const string& data, char delimiter = ',');
    void cleanupMessages();
    void logDepartureIfNeeded();
    void checkVehicleStatus();
};

} // namespace veins
