#include "SimulationLogger.h"
#include <algorithm>
#include <numeric>
#include <ctime>
#include <sstream>
#include <iomanip>

using namespace veins;

SimulationLogger::SimulationLogger() : summaryCalculated(false) {
    summary.mapName = "Unknown";
    summary.routingAlgorithm = "Unknown";
    summary.implementationVersion = "1.0";
    summary.simulationTimestamp = getCurrentTimeStamp();
}

SimulationLogger::~SimulationLogger() {
    if (!vehicleStats.empty() && !summaryCalculated) {
        calculateSummaryStats();
    }
}

void SimulationLogger::recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime) {
    std::lock_guard<std::mutex> lock(mutex);
    Vehicle& vehicle = vehicleStats[vehicleId];
    vehicle.vehicleId = vehicleId;
    vehicle.startingRoad = startRoad;
    vehicle.start(startTime);
}

void SimulationLogger::updateVehicleDestination(int vehicleId, const std::string& destination, 
                                              double earliestArrival, double latestArrival,
                                              const std::vector<std::string>& path,
                                              double pathLength,
                                              double estimatedTravelTime) {
    std::lock_guard<std::mutex> lock(mutex);
    auto it = vehicleStats.find(vehicleId);
    if (it != vehicleStats.end()) {
        it->second.targetRoad = destination;
        it->second.earliestArrival = earliestArrival;
        it->second.latestArrival = latestArrival;
        it->second.plannedPath = path;
        it->second.pathLength = pathLength;
        it->second.estimatedTravelTime = estimatedTravelTime;
    } else {
        Vehicle vehicle;
        vehicle.vehicleId = vehicleId;
        vehicle.targetRoad = destination;
        vehicle.earliestArrival = earliestArrival;
        vehicle.latestArrival = latestArrival;
        vehicle.plannedPath = path;
        vehicle.pathLength = pathLength;
        vehicle.estimatedTravelTime = estimatedTravelTime;
        
        vehicleStats[vehicleId] = vehicle;
    }
}

// Đã loại bỏ phương thức recordAlgorithmTime không còn sử dụng

void SimulationLogger::recordTotalAlgorithmTime(double totalAlgorithmTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    // Lưu thời gian thuật toán tổng vào summary
    summary.totalAlgorithmTime = totalAlgorithmTime;
    
    // Log với 6 chữ số thập phân
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "TOTAL routing algorithm time: " << totalAlgorithmTime << " seconds" << std::endl;
    std::cout.unsetf(std::ios_base::fixed); // Reset formatting to default
    
    // Không cần phân chia thời gian thuật toán cho từng xe nữa
}

void SimulationLogger::recordVehicleEnd(int vehicleId, double endTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    auto it = vehicleStats.find(vehicleId);
    if (it == vehicleStats.end()) {
        std::cout << "SimulationLogger: WARNING - Recording end for unknown vehicle " 
                  << vehicleId << " at time " << endTime << std::endl;
        // create new record for this vehicle if it doesn't exist
        vehicleStats[vehicleId] = Vehicle();
        vehicleStats[vehicleId].vehicleId = vehicleId;
        vehicleStats[vehicleId].finish(endTime);
    } else {
        it->second.endTime = endTime;
        
        // check if valid start time, calculate travel time for vehicle.
        if (it->second.startTime > 0) {
            it->second.travelTime = endTime - it->second.startTime;
        }

        if (it->second.earliestArrival <= 0 && it->second.latestArrival <= 0) {
            std::cout << "SimulationLogger: WARNING - Vehicle " << vehicleId 
                      << " has invalid time window [0,0], this will cause incorrect statistics" << std::endl;
        }
        if (it->second.earliestArrival > 0 || it->second.latestArrival > 0) {
            double travelTime = it->second.travelTime;
            
            // calculate time window deviation
            it->second.timeWindowDeviation = calculateTimeWindowDeviation(
                endTime, it->second.earliestArrival, it->second.latestArrival);
            // evaluate if we arrive on time.
            if (travelTime < it->second.earliestArrival) {
                // early arrival
                it->second.arrivedOnTime = false; 
            } else if (travelTime > it->second.latestArrival) {
                // late arrival
                it->second.arrivedOnTime = false;
            } else {
                // On time
                it->second.arrivedOnTime = true;
            }
        }
    }
    
    // Log arrival event with fixed precision for time values
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "VEHICLE_FINISH: ID=" << vehicleId 
              << ", START=" << vehicleStats[vehicleId].startTime 
              << ", END=" << endTime 
              << ", DURATION=" << vehicleStats[vehicleId].travelTime
              << ", TIME_WINDOW=[" << vehicleStats[vehicleId].earliestArrival 
              << ", " << vehicleStats[vehicleId].latestArrival << "]" << std::endl;
    std::cout.unsetf(std::ios_base::fixed); // Reset formatting to default
    if (allVehiclesFinished()) {
        calculateSummaryStats();
    }
}

void SimulationLogger::setSimulationInfo(const std::string& mapName, const std::string& algorithm,
                                       const std::string& version) {
    std::lock_guard<std::mutex> lock(mutex);
    
    summary.mapName = mapName;
    summary.routingAlgorithm = algorithm;
    summary.implementationVersion = version;
    std::cout << "Simulation info set: Map=" << mapName 
              << ", Algorithm=" << algorithm
              << ", Version=" << version << std::endl;
}

bool SimulationLogger::allVehiclesFinished() const {
    for (const auto& pair : vehicleStats) {
        if (pair.second.endTime == 0) {
            return false;
        }
    }
    return !vehicleStats.empty();
}

void SimulationLogger::calculateSummaryStats() {
    if (summaryCalculated) {
        return;
    }
    
    summary.totalVehicles = vehicleStats.size();
    summary.lateVehicles = 0;
    summary.totalTravelTime = 0;
    summary.totalTimeWindowDeviation = 0;
    // totalAlgorithmTime is now set directly by recordTotalAlgorithmTime
    
    for (const auto& pair : vehicleStats) {
        const Vehicle& vehicle = pair.second;
        
        if (!vehicle.arrivedOnTime) {
            summary.lateVehicles++;
        }
        
        summary.totalTravelTime += vehicle.travelTime;
        summary.totalTimeWindowDeviation += vehicle.timeWindowDeviation;
        // No longer accumulating algorithm time from individual vehicles
    }
    summary.objectiveFunctionValue = summary.totalTravelTime + summary.totalTimeWindowDeviation;
    summaryCalculated = true;
}

void SimulationLogger::printSummary() const {
    std::cout << "\n========== SIMULATION SUMMARY ==========\n";
    std::cout << "Map: " << summary.mapName << std::endl;
    std::cout << "Routing Algorithm: " << summary.routingAlgorithm << std::endl;
    std::cout << "Implementation Version: " << summary.implementationVersion << std::endl;
    std::cout << "Timestamp: " << summary.simulationTimestamp << std::endl;
    std::cout << "Total Vehicles: " << summary.totalVehicles << std::endl;
    std::cout << "Late Vehicles: " << summary.lateVehicles << " (" 
              << (summary.totalVehicles > 0 ? (100.0 * summary.lateVehicles / summary.totalVehicles) : 0)
              << "%)" << std::endl;
    std::cout << "Total Travel Time: " << summary.totalTravelTime << " seconds" << std::endl;
    std::cout << "Average Travel Time: " 
              << (summary.totalVehicles > 0 ? (summary.totalTravelTime / summary.totalVehicles) : 0)
              << " seconds/vehicle" << std::endl;
    std::cout << "Total Time Window Deviation: " << summary.totalTimeWindowDeviation << std::endl;
    std::cout << "Average Time Window Deviation: "
              << (summary.totalVehicles > 0 ? (summary.totalTimeWindowDeviation / summary.totalVehicles) : 0)
              << " per vehicle" << std::endl;
    std::cout << "Total Algorithm Time: " << summary.totalAlgorithmTime << " seconds" << std::endl;
    std::cout << "Average Algorithm Time: "
              << (summary.totalVehicles > 0 ? (summary.totalAlgorithmTime / summary.totalVehicles) : 0)
              << " seconds/vehicle" << std::endl;
    std::cout << "Objective Function Value: " << summary.objectiveFunctionValue << std::endl;
    std::cout << "=========================================\n";
}

void SimulationLogger::saveToCSV(const std::string& filename) {
    if (!summaryCalculated) {
        calculateSummaryStats();
    }
    
    // Save all information to a single file
    std::ofstream file(filename);
    if (file.is_open()) {
        // Write the summary information first
        file << "SIMULATION SUMMARY" << std::endl;
        file << "MapName,RoutingAlgorithm,Version,TotalVehicles,LateVehicles,"
             << "TotalTravelTime,AvgTravelTime,TotalTimeWindowDeviation,"
             << "AvgTimeWindowDeviation,TotalAlgorithmTime,AvgAlgorithmTime,"
             << "ObjectiveFunctionValue,Timestamp" << std::endl;
             
        file << summary.mapName << ","
             << summary.routingAlgorithm << ","
             << summary.implementationVersion << ","
             << summary.totalVehicles << ","
             << summary.lateVehicles << ","
             << summary.totalTravelTime << ","
             << (summary.totalVehicles > 0 ? (summary.totalTravelTime / summary.totalVehicles) : 0) << ","
             << summary.totalTimeWindowDeviation << ","
             << (summary.totalVehicles > 0 ? (summary.totalTimeWindowDeviation / summary.totalVehicles) : 0) << ","
             << summary.totalAlgorithmTime << ","
             << (summary.totalVehicles > 0 ? (summary.totalAlgorithmTime / summary.totalVehicles) : 0) << ","
             << summary.objectiveFunctionValue << ","
             << summary.simulationTimestamp << std::endl;
        
        // Add a blank line to separate summary from vehicle data
        file << std::endl;
        
        // Write the vehicle statistics
        file << "VEHICLE STATISTICS" << std::endl;
        file << "VehicleID,StartRoad,TargetRoad,StartTime,EndTime,TravelTime,"
             << "EarliestArrival,LatestArrival,TimeWindowDeviation,ArrivedOnTime,"
             << "PathLength" << std::endl;  // Đã loại bỏ AlgorithmTime
             
        // Vehicle data
        for (const auto& pair : vehicleStats) {
            const Vehicle& vehicle = pair.second;
            file << vehicle.vehicleId << ","
                 << vehicle.startingRoad << ","
                 << vehicle.targetRoad << ","
                 << vehicle.startTime << ","
                 << vehicle.endTime << ","
                 << vehicle.travelTime << ","
                 << vehicle.earliestArrival << ","
                 << vehicle.latestArrival << ","
                 << vehicle.timeWindowDeviation << ","
                 << (vehicle.arrivedOnTime ? "Yes" : "No") << ","
                 << vehicle.pathLength << std::endl;  // Đã loại bỏ algorithmTime
        }
        
        file.close();
        std::cout << "Saved simulation results to " << filename << std::endl;
    } else {
        std::cerr << "Error: Unable to open file " << filename << " for writing" << std::endl;
    }
}

std::string SimulationLogger::getCurrentTimeStamp() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

double SimulationLogger::calculateTimeWindowDeviation(double endTime, double earliestArrival, double latestArrival) const {
    if (endTime < earliestArrival) {
        return earliestArrival - endTime;
    }
    else if (endTime > latestArrival) {
        return endTime - latestArrival;
    }
    else {
        return 0.0;
    }
} 
