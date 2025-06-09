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
#include "SimulationLogger.h"
#include "Vehicle.h"

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
    cMessage* statusCheckMsg;
    cMessage* rerouteMsg;

    unique_ptr<XMLProcessor> xmlProcessor;      // For processing road network XML
    unique_ptr<GraphProcessor> graphProcessor;  // For path finding
    unique_ptr<TaskGenerator> taskGenerator;    // For generating tasks

    string networkFilePath;
    int numVehicles = 10;   // Default number of vehicles to pre-map

    void handleVehicleMessage(const string& message, LAddress::L2Type vehicleId);
    void sendRoadListMessage(LAddress::L2Type vehicleId, const vector<string>& roadList);
    void sendRerouteMessage(LAddress::L2Type vehicleId, const vector<string>& edgePath);
    map<LAddress::L2Type, Vehicle> vehicleDataMap;

    // Map between RSU internal IDs and simulation IDs
    map<int, LAddress::L2Type> simulationIdToAddressMap;
    void updateVehicleIdMapping(LAddress::L2Type vehicleAddress, int simulationId);

    void cleanupVehicleData();
    void sendRerouteToAllVehicles();
    
    // Road network data access methods
    vector<string> getAllRoads() const;
    vector<string> getAllNodes() const;

    vector<string> findShortestPath(const string& sourceId, const string& targetId) const;
    double getShortestPathLength(const std::string& sourceId, const std::string& targetId) const;

    void printNodeInfo() const;
    void printVehicleRouteInfo(const std::vector<Vehicle>& vehicles);
    void generateAndAssignDestinations(const std::vector<Vehicle>& vehicles);
    void findEdgePathAndPrint(std::string sourceEdgeId, std::string targetEdgeId) const;
    void testTaskGenerator();
    double getEdgeLength(const std::string& edgeId) const;
    void recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime);
    void recordVehicleDestination(int vehicleId, const std::string& targetRoad, 
                                  double earliestArrival, double latestArrival,
                                  const std::vector<std::string>& path, double pathLength);
    void recordAlgorithmTime(int vehicleId, double algorithmTime);
    
    // Simulation info
    std::string mapName;
    std::string routingAlgorithm;
    std::string implementationVersion;
    
    // Track vehicle departures and arrivals for statistics
    void processVehicleDepartureNotification(const std::string& data);
    void processVehicleArrivalNotification(const std::string& data);
};

} // namespace veins

#endif /* VEINS_INET_RSUCONTROLAPP_H_ */
