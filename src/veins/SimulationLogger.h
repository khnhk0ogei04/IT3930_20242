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
        std::string implementationVersion;
        int totalVehicles;
        int lateVehicles;
        double totalTravelTime;
        double totalTimeWindowDeviation;
        double totalAlgorithmTime;
        double objectiveFunctionValue;
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
    // Chỉ giữ lại phương thức ghi nhận tổng thời gian thuật toán
    void recordTotalAlgorithmTime(double totalAlgorithmTime);
    void recordVehicleEnd(int vehicleId, double endTime);
    void setSimulationInfo(const std::string& mapName, const std::string& algorithm, 
                          const std::string& version);
    void saveToCSV(const std::string& filename = "simulation_results_2.csv");
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
    std::map<int, Vehicle> vehicleStats;
    SimulationSummary summary;
    std::mutex mutex;
    bool summaryCalculated;
};

} // namespace veins

#endif /* VEINS_SIMULATIONLOGGER_H_ */ 
