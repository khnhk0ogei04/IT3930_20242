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

namespace veins {

/**
 * RSU Control Application that manages the road network and provides
 * routing capabilities through composition with specialized components.
 */
class RSUControlApp : public TraCIDemoRSU11p {
public:
    void initialize(int stage) override;
    void finish() override;
    virtual void onWSA(DemoServiceAdvertisment* wsa) override;
    virtual void onWSM(BaseFrame1609_4* wsm) override;

protected:
    void handleSelfMsg(cMessage* msg) override;

private:
    // Message for periodic status checks
    cMessage* statusCheckMsg;
    // Message for triggering rerouting at t=2s
    cMessage* rerouteMsg;

    // Specialized components (composition)
    std::unique_ptr<XMLProcessor> xmlProcessor;      // For processing road network XML
    std::unique_ptr<GraphProcessor> graphProcessor;  // For path finding
    std::unique_ptr<TaskGenerator> taskGenerator;    // For generating tasks

    // Network file path
    std::string networkFilePath;

    // Process vehicle messages and send responses
    void handleVehicleMessage(const std::string& message, LAddress::L2Type vehicleId);
    void sendRoadListMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& roadList);
    void sendRerouteMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& edgePath);

    // Vehicle tracking data
    struct VehicleData {
        simtime_t lastMessageTime = 0;
        Destination assignedDestination; // Store the assigned destination
        std::vector<std::string> assignedPath; // Store the assigned path
        std::vector<std::string> lastSentPath; // Store the last path sent to the vehicle
        int simulationId = -1; // Store the actual simulation ID of the vehicle
        double startTime = 0.0; // Thời gian xe bắt đầu
        double estimatedTravelTime = 0.0; // Thời gian di chuyển ước tính
        double earliestArrival = 0.0; // Thời gian đến sớm nhất (time window)
        double latestArrival = 0.0; // Thời gian đến trễ nhất (time window)
        double pathLength = 0.0; // Độ dài đường đi
        double algorithmTime = 0.0; // Thời gian để tìm đường
    };
    std::map<LAddress::L2Type, VehicleData> vehicleDataMap;

    // Map between RSU internal IDs and simulation IDs
    std::map<int, LAddress::L2Type> simulationIdToAddressMap;
    void updateVehicleIdMapping(LAddress::L2Type vehicleAddress, int simulationId);

    void cleanupVehicleData();
    void sendRerouteToAllVehicles();
    
    // Road network data access methods
    std::vector<std::string> getAllRoads() const;
    std::vector<std::string> getAllNodes() const;

    // Path finding methods
    std::vector<std::string> findShortestPath(const std::string& sourceId, const std::string& targetId) const;
    double getShortestPathLength(const std::string& sourceId, const std::string& targetId) const;

    // Information display methods
    void printRoadNetworkInfo() const;
    void printNodeInfo() const;
    void printVehicleRouteInfo(const std::vector<VehicleInfo>& vehicles);
    void generateAndAssignDestinations(const std::vector<VehicleInfo>& vehicles);

    // Lane path finding method
    void findLanePathAndPrint(std::string sourceLaneId, std::string targetLaneId) const;

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
