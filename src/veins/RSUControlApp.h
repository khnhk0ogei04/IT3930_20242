#ifndef VEINS_INET_RSUCONTROLAPP_H_
#define VEINS_INET_RSUCONTROLAPP_H_

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "XMLProcessor.h"
#include "GraphProcessor.h"
#include "TaskGenerator.h"

using namespace std;

namespace veins {
class RSUControlApp : public TraCIDemoRSU11p {
public:
    void initialize(int stage) override;
    void finish() override;
    virtual void onWSA(DemoServiceAdvertisment* wsa) override;
    virtual void onWSM(BaseFrame1609_4* wsm) override;

protected:
    void handleSelfMsg(cMessage* msg) override;
    
private:
    struct VehicleData {
       simtime_t lastMessageTime = 0;
       Destination assignedDestination; // assigned destination
       vector<string> assignedPath; // assigned path
       vector<string> lastSentPath; // last path sent to the vehicle
       int simulationId = -1; // actual simulation ID of the vehicle
    };

    cMessage* statusCheckMsg;
    cMessage* rerouteMsg;
//    cMessage* periodicRerouteMsg;
    unique_ptr<XMLProcessor> xmlProcessor;      // for processing road network XML
    unique_ptr<GraphProcessor> graphProcessor;  // for path finding
    unique_ptr<TaskGenerator> taskGenerator;    // for generating tasks

    string networkFilePath;
    void handleVehicleMessage(const string& message, LAddress::L2Type vehicleId);
    void sendRoadListMessage(LAddress::L2Type vehicleId, const vector<string>& roadList);
    void sendRerouteMessage(LAddress::L2Type vehicleId, const vector<string>& edgePath);
    map<LAddress::L2Type, VehicleData> vehicleDataMap;
    
    // map between RSU internal IDs and simulation IDs
    map<int, LAddress::L2Type> simulationIdToAddressMap;
    void updateVehicleIdMapping(LAddress::L2Type vehicleAddress, int simulationId);
    void cleanupVehicleData();
    void sendRerouteToAllVehicles();
    vector<string> getAllRoads() const;
    vector<string> getAllNodes() const;

    // path finding
    vector<string> findShortestPath(const string& sourceId, const string& targetId) const;
    double getShortestPathLength(const string& sourceId, const string& targetId) const;

    // information display methods
    void printRoadNetworkInfo() const;
    void printNodeInfo() const;
    void printVehicleRouteInfo(const vector<VehicleInfo>& vehicles);
    void generateAndAssignDestinations(const vector<VehicleInfo>& vehicles);

    // void findLanePathAndPrint(std::string sourceLaneId, std::string targetLaneId) const;

    // Edge path finding method
    void findEdgePathAndPrint(string sourceEdgeId, string targetEdgeId) const;
    void testTaskGenerator();
    double getEdgeLength(const string& edgeId) const; // get edge length
};

} // namespace veins

#endif /* VEINS_INET_RSUCONTROLAPP_H_ */
