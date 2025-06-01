#ifndef VEINS_SIMULATIONLOGGER_H_
#define VEINS_SIMULATIONLOGGER_H_

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <mutex>

namespace veins {

class SimulationLogger {
public:
    // Singleton pattern
    static SimulationLogger& getInstance() {
        static SimulationLogger instance;
        return instance;
    }
    struct VehicleStats {
        int vehicleId;                  // ID of vehicle
        std::string startingRoad;       // edgeId start
        std::string targetRoad;         // edgeId target
        double startTime;
        double endTime;
        double travelTime;
        double earliestArrival;
        double latestArrival;
        double timeWindowDeviation;
        bool arrivedOnTime;
        std::vector<std::string> path;
        double pathLength;
        double algorithmTime;           // time for algorithm
        double estimatedTravelTime;

        VehicleStats() : 
            vehicleId(-1), startTime(0), endTime(0), travelTime(0),
            earliestArrival(0), latestArrival(0), timeWindowDeviation(0),
            arrivedOnTime(true), pathLength(0), algorithmTime(0), estimatedTravelTime(0) {}
    };

    struct SimulationSummary {
        std::string mapName;
        std::string routingAlgorithm;
        std::string implementationVersion;  // PhiÃªn báº£n cÃ i Ä‘áº·t
        int totalVehicles;                  // Tá»•ng sá»‘ xe
        int lateVehicles;                   // Sá»‘ xe Ä‘áº¿n khÃ´ng ká»‹p
        double totalTravelTime;             // Tá»•ng thá»�i gian cÃ¡c xe cháº¡y
        double totalTimeWindowDeviation;    // Tá»•ng Ä‘á»™ lá»‡ch khung thá»�i gian
        double totalAlgorithmTime;          // Tá»•ng thá»�i gian giáº£i thuáº­t tÃ¬m Ä‘Æ°á»�ng
        double objectiveFunctionValue;      // HÃ m má»¥c tiÃªu = totalTravelTime + totalTimeWindowDeviation
        std::string simulationTimestamp;

        SimulationSummary() : 
            totalVehicles(0), lateVehicles(0), totalTravelTime(0), 
            totalTimeWindowDeviation(0), totalAlgorithmTime(0), objectiveFunctionValue(0) {}
    };
    void recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime);
    void updateVehicleDestination(int vehicleId, const std::string& destination, 
                                 double earliestArrival, double latestArrival,
                                 const std::vector<std::string>& path,
                                 double pathLength,
                                 double estimatedTravelTime = 0.0);
    void recordAlgorithmTime(int vehicleId, double algorithmTime);
    void recordVehicleEnd(int vehicleId, double endTime);
    void setSimulationInfo(const std::string& mapName, const std::string& algorithm, 
                          const std::string& version);
    void saveToCSV(const std::string& filename = "simulation_results_1.csv");
    bool allVehiclesFinished() const;
    void printSummary() const;

private:
    SimulationLogger();
    ~SimulationLogger();
    SimulationLogger(const SimulationLogger&) = delete;
    SimulationLogger& operator=(const SimulationLogger&) = delete;
    void calculateSummaryStats();
    std::string getCurrentTimeStamp() const;
    double calculateTimeWindowDeviation(double endTime, double earliestArrival, double latestArrival) const;
    
    std::map<int, VehicleStats> vehicleStats;  // ThÃ´ng tin vá»� má»—i xe
    SimulationSummary summary;                 // ThÃ´ng tin tá»•ng há»£p
    std::mutex mutex;                          // Mutex Ä‘á»ƒ báº£o vá»‡ truy cáº­p Ä‘á»“ng thá»�i
    bool summaryCalculated;                    // Ä�Ã£ tÃ­nh tá»•ng há»£p chÆ°a
};

} // namespace veins

#endif /* VEINS_SIMULATIONLOGGER_H_ */ 
