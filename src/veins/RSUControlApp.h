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
    
    // Specialized components (composition)
    std::unique_ptr<XMLProcessor> xmlProcessor;      // For processing road network XML
    std::unique_ptr<GraphProcessor> graphProcessor;  // For path finding
    std::unique_ptr<TaskGenerator> taskGenerator;    // For generating tasks
    
    // Network file path
    std::string networkFilePath;
    
    // Process vehicle messages and send responses
    void handleVehicleMessage(const std::string& message, LAddress::L2Type vehicleId);
    void sendRoadListMessage(LAddress::L2Type vehicleId, const std::vector<std::string>& roadList);
    
    // Vehicle tracking data
    struct VehicleData {
        simtime_t lastMessageTime = 0;
    };
    std::map<LAddress::L2Type, VehicleData> vehicleDataMap;
    void cleanupVehicleData();
    
    // Road network data access methods
    std::vector<std::string> getAllRoads() const;
    std::vector<std::string> getAllNodes() const;
    
    // Path finding methods
    std::vector<std::string> findShortestPath(const std::string& sourceId, const std::string& targetId) const;
    double getShortestPathLength(const std::string& sourceId, const std::string& targetId) const;
    
    // Information display methods
    void printRoadNetworkInfo() const;
    void printNodeInfo() const;
    
    // Lane path finding method
    void findLanePathAndPrint(std::string sourceLaneId, std::string targetLaneId) const;
    
    // Edge path finding method
    void findEdgePathAndPrint(std::string sourceEdgeId, std::string targetEdgeId) const;
    
    // Test methods
    void testTaskGenerator();
};

} // namespace veins

#endif /* VEINS_INET_RSUCONTROLAPP_H_ */
