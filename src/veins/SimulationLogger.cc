#include "SimulationLogger.h"
#include <algorithm>
#include <numeric>
#include <ctime>
#include <sstream>
#include <iomanip>

using namespace veins;

SimulationLogger::SimulationLogger() : summaryCalculated(false) {
    // Khởi tạo các thông tin mặc định
    summary.mapName = "Unknown";
    summary.routingAlgorithm = "Unknown";
    summary.implementationVersion = "1.0";
    summary.simulationTimestamp = getCurrentTimeStamp();
}

SimulationLogger::~SimulationLogger() {
    // Tự động lưu dữ liệu khi hủy logger
    if (!vehicleStats.empty() && !summaryCalculated) {
        calculateSummaryStats();
    }
}

void SimulationLogger::recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    // Tạo hoặc cập nhật thông tin xe
    VehicleStats& stats = vehicleStats[vehicleId];
    stats.vehicleId = vehicleId;
    stats.startingRoad = startRoad;
    stats.startTime = startTime;
    
    // Log
    std::cout << "Vehicle " << vehicleId << " started at time " << startTime 
              << " from road " << startRoad << std::endl;
}

void SimulationLogger::updateVehicleDestination(int vehicleId, const std::string& destination, 
                                              double earliestArrival, double latestArrival,
                                              const std::vector<std::string>& path,
                                              double pathLength,
                                              double estimatedTravelTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    // Tìm xe trong danh sách theo ID
    auto it = vehicleStats.find(vehicleId);
    if (it != vehicleStats.end()) {
        it->second.targetRoad = destination;
        it->second.earliestArrival = earliestArrival;
        it->second.latestArrival = latestArrival;
        it->second.path = path;
        it->second.pathLength = pathLength;
        it->second.estimatedTravelTime = estimatedTravelTime;
        
        // In thông tin đích và khung thời gian
        std::cout << "DESTINATION_ASSIGNED: vehicleId=" << vehicleId
                  << ", destination=" << destination
                  << ", timeWindow=[" << earliestArrival << ", " << latestArrival << "]"
                  << ", pathLength=" << pathLength
                  << ", estimatedTravelTime=" << estimatedTravelTime << std::endl;
    } else {
        // Nếu xe chưa tồn tại, tạo bản ghi mới
        VehicleStats stats;
        stats.vehicleId = vehicleId;
        stats.targetRoad = destination;
        stats.earliestArrival = earliestArrival;
        stats.latestArrival = latestArrival;
        stats.path = path;
        stats.pathLength = pathLength;
        stats.estimatedTravelTime = estimatedTravelTime;
        
        vehicleStats[vehicleId] = stats;
        
        // In thông tin đích và khung thời gian
        std::cout << "DESTINATION_CREATED: vehicleId=" << vehicleId
                  << ", destination=" << destination
                  << ", timeWindow=[" << earliestArrival << ", " << latestArrival << "]"
                  << ", pathLength=" << pathLength
                  << ", estimatedTravelTime=" << estimatedTravelTime << std::endl;
    }
}

void SimulationLogger::recordAlgorithmTime(int vehicleId, double algorithmTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    // Cập nhật thời gian tìm đường
    VehicleStats& stats = vehicleStats[vehicleId];
    stats.algorithmTime = algorithmTime;
    
    // Log
    std::cout << "Vehicle " << vehicleId << " routing algorithm time: " 
              << algorithmTime << " seconds" << std::endl;
}

void SimulationLogger::recordVehicleEnd(int vehicleId, double endTime) {
    std::lock_guard<std::mutex> lock(mutex);
    
    auto it = vehicleStats.find(vehicleId);
    if (it == vehicleStats.end()) {
        std::cout << "SimulationLogger: WARNING - Recording end for unknown vehicle " 
                  << vehicleId << " at time " << endTime << std::endl;
        // Create new record for this vehicle if it doesn't exist
        vehicleStats[vehicleId] = VehicleStats();
        vehicleStats[vehicleId].vehicleId = vehicleId;
        vehicleStats[vehicleId].endTime = endTime;
    } else {
        it->second.endTime = endTime;
        
        // If we have a valid start time, calculate the duration
        if (it->second.startTime > 0) {
            it->second.travelTime = endTime - it->second.startTime;
        }
        
        // Validate time windows (ensure we don't have [0,0])
        if (it->second.earliestArrival <= 0 && it->second.latestArrival <= 0) {
            // If time windows were not set, fetch them from RSU data if possible
            // For now, just log a warning
            std::cout << "SimulationLogger: WARNING - Vehicle " << vehicleId 
                      << " has invalid time window [0,0], this will cause incorrect statistics" << std::endl;
        }
        
        // Calculate if arrival was within time window
        if (it->second.earliestArrival > 0 || it->second.latestArrival > 0) {
            double travelTime = it->second.travelTime;
            
            // Calculate time window deviation
            it->second.timeWindowDeviation = calculateTimeWindowDeviation(
                endTime, it->second.earliestArrival, it->second.latestArrival);
                
            if (travelTime < it->second.earliestArrival) {
                // Early arrival
                it->second.arrivedOnTime = false; 
            } else if (travelTime > it->second.latestArrival) {
                // Late arrival
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
              
    // Check if all vehicles have finished, calculate summary
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
    
    // Log
    std::cout << "Simulation info set: Map=" << mapName 
              << ", Algorithm=" << algorithm
              << ", Version=" << version << std::endl;
}

bool SimulationLogger::allVehiclesFinished() const {
    // Kiểm tra xem tất cả xe đã kết thúc chưa
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
    summary.totalAlgorithmTime = 0;
    
    for (const auto& pair : vehicleStats) {
        const VehicleStats& stats = pair.second;
        
        if (!stats.arrivedOnTime) {
            summary.lateVehicles++;
        }
        
        summary.totalTravelTime += stats.travelTime;
        summary.totalTimeWindowDeviation += stats.timeWindowDeviation;
        summary.totalAlgorithmTime += stats.algorithmTime;
    }
    
    summaryCalculated = true;
}

void SimulationLogger::printSummary() const {
    if (!summaryCalculated) {
        std::cout << "Summary not yet calculated!" << std::endl;
        return;
    }
    
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
    std::cout << "=========================================\n";
}

void SimulationLogger::saveToCSV(const std::string& vehicleStatsFilename, 
                               const std::string& summaryFilename) {
    if (!summaryCalculated) {
        calculateSummaryStats();
    }
    
    // Lưu thông tin từng xe
    std::ofstream vehicleFile(vehicleStatsFilename);
    if (vehicleFile.is_open()) {
        // Header
        vehicleFile << "VehicleID,StartRoad,TargetRoad,StartTime,EndTime,TravelTime,"
                   << "EarliestArrival,LatestArrival,TimeWindowDeviation,ArrivedOnTime,"
                   << "PathLength,AlgorithmTime" << std::endl;
                   
        // Dữ liệu xe
        for (const auto& pair : vehicleStats) {
            const VehicleStats& stats = pair.second;
            vehicleFile << stats.vehicleId << ","
                       << stats.startingRoad << ","
                       << stats.targetRoad << ","
                       << stats.startTime << ","
                       << stats.endTime << ","
                       << stats.travelTime << ","
                       << stats.earliestArrival << ","
                       << stats.latestArrival << ","
                       << stats.timeWindowDeviation << ","
                       << (stats.arrivedOnTime ? "Yes" : "No") << ","
                       << stats.pathLength << ","
                       << stats.algorithmTime << std::endl;
        }
        vehicleFile.close();
        std::cout << "Saved vehicle statistics to " << vehicleStatsFilename << std::endl;
    } else {
        std::cerr << "Error: Unable to open file " << vehicleStatsFilename << " for writing" << std::endl;
    }
    
    // Lưu thông tin tổng hợp
    std::ofstream summaryFile(summaryFilename);
    if (summaryFile.is_open()) {
        // Header
        summaryFile << "MapName,RoutingAlgorithm,Version,TotalVehicles,LateVehicles,"
                   << "TotalTravelTime,AvgTravelTime,TotalTimeWindowDeviation,"
                   << "AvgTimeWindowDeviation,TotalAlgorithmTime,AvgAlgorithmTime,Timestamp" << std::endl;
                   
        // Dữ liệu tổng hợp
        summaryFile << summary.mapName << ","
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
                   << summary.simulationTimestamp << std::endl;
                   
        summaryFile.close();
        std::cout << "Saved simulation summary to " << summaryFilename << std::endl;
    } else {
        std::cerr << "Error: Unable to open file " << summaryFilename << " for writing" << std::endl;
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
    // Nếu xe đến sớm hơn thời gian sớm nhất
    if (endTime < earliestArrival) {
        return earliestArrival - endTime;
    }
    // Nếu xe đến trễ hơn thời gian trễ nhất
    else if (endTime > latestArrival) {
        return endTime - latestArrival;
    }
    // Xe đến trong khung thời gian
    else {
        return 0.0;
    }
} 