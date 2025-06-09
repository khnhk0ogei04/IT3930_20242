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
#include "Vehicle.h"

namespace veins {

class SimulationLogger {
public:
    // Singleton pattern
    static SimulationLogger& getInstance() {
        static SimulationLogger instance;
        return instance;
    }

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
    
    std::map<int, Vehicle> vehicleStats;       // Thông tin về mỗi xe
    SimulationSummary summary;                 // ThÃ´ng tin tá»•ng há»£p
    std::mutex mutex;                          // Mutex Ä‘á»ƒ báº£o vá»‡ truy cáº­p Ä‘á»“ng thá»�i
    bool summaryCalculated;                    // Ä�Ã£ tÃ­nh tá»•ng há»£p chÆ°a
};

} // namespace veins

#endif /* VEINS_SIMULATIONLOGGER_H_ */ 
