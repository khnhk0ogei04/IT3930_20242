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
        Destination assignedDestination; // Store the assigned destination
        vector<string> assignedPath; // Store the assigned path
        vector<string> lastSentPath; // Store the last path sent to the vehicle
        int simulationId = -1; // Store the actual simulation ID of the vehicle
        double startTime = 0.0; // Thời gian xe bắt đầu
        double estimatedTravelTime = 0.0; // Thời gian di chuyển ước tính
        double earliestArrival = 0.0; // Thời gian đến sớm nhất (time window)
        double latestArrival = 0.0; // Thời gian đến trễ nhất (time window)
        double pathLength = 0.0; // Độ dài đường đi
        double algorithmTime = 0.0; // Thời gian để tìm đường
    };
    cMessage* statusCheckMsg;
    cMessage* rerouteMsg;

    unique_ptr<XMLProcessor> xmlProcessor;      // For processing road network XML
    unique_ptr<GraphProcessor> graphProcessor;  // For path finding
    unique_ptr<TaskGenerator> taskGenerator;    // For generating tasks

    string networkFilePath;

    void handleVehicleMessage(const string& message, LAddress::L2Type vehicleId);
    void sendRoadListMessage(LAddress::L2Type vehicleId, const vector<string>& roadList);
    void sendRerouteMessage(LAddress::L2Type vehicleId, const vector<string>& edgePath);
    map<LAddress::L2Type, VehicleData> vehicleDataMap;

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
    void printVehicleRouteInfo(const std::vector<VehicleInfo>& vehicles);
    void generateAndAssignDestinations(const std::vector<VehicleInfo>& vehicles);
    // Edge path finding method
    void findEdgePathAndPrint(std::string sourceEdgeId, std::string targetEdgeId) const;

    // Test methods
    void testTaskGenerator();

    // Helper method to get edge length
    double getEdgeLength(const std::string& edgeId) const;
    
    // Vehicle tracking methods
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
