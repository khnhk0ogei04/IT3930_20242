#include "SimulationLogger.h"
#include <algorithm>
#include <numeric>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <mutex>

using namespace veins;
using namespace std;

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

void SimulationLogger::recordVehicleStart(int vehicleId, const string& startRoad, double startTime) {
    lock_guard<std::mutex> lock(mutex);
    Vehicle& vehicle = vehicleStats[vehicleId];
    vehicle.vehicleId = vehicleId;
    vehicle.startingRoad = startRoad;
    vehicle.start(startTime);
}

void SimulationLogger::updateVehicleDestination(int vehicleId, const string& destination, 
                                              double earliestArrival, double latestArrival,
                                              const vector<string>& path,
                                              double pathLength,
                                              double estimatedTravelTime) {
    lock_guard<std::mutex> lock(mutex);
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

void SimulationLogger::recordTotalAlgorithmTime(double totalAlgorithmTime) {
    lock_guard<std::mutex> lock(mutex);
    summary.totalAlgorithmTime = totalAlgorithmTime;
    
    cout << fixed << setprecision(6);
    cout << "Total algorithm time: " << totalAlgorithmTime << " seconds" << endl;
    cout.unsetf(ios_base::fixed);
}

void SimulationLogger::recordVehicleEnd(int vehicleId, double endTime) {
    lock_guard<std::mutex> lock(mutex);
    
    auto it = vehicleStats.find(vehicleId);
    if (it == vehicleStats.end()) {
        vehicleStats[vehicleId] = Vehicle();
        vehicleStats[vehicleId].vehicleId = vehicleId;
        vehicleStats[vehicleId].finish(endTime);
    } else {
        it->second.endTime = endTime;
        
        if (it->second.startTime > 0) {
            it->second.travelTime = endTime - it->second.startTime;
        }

        if (it->second.earliestArrival <= 0 && it->second.latestArrival <= 0) {
            // Invalid time window
        }
        if (it->second.earliestArrival > 0 || it->second.latestArrival > 0) {
            double travelTime = it->second.travelTime;
            
            it->second.timeWindowDeviation = calculateTimeWindowDeviation(
                endTime, it->second.earliestArrival, it->second.latestArrival);
                
            if (travelTime < it->second.earliestArrival) {
                it->second.arrivedOnTime = false; 
            } else if (travelTime > it->second.latestArrival) {
                it->second.arrivedOnTime = false;
            } else {
                it->second.arrivedOnTime = true;
            }
        }
    }
    
    cout << fixed << setprecision(1);
    cout << "Vehicle " << vehicleId 
         << " finished at time " << endTime 
         << ", duration " << vehicleStats[vehicleId].travelTime << endl;
    cout.unsetf(ios_base::fixed);
    
    if (allVehiclesFinished()) {
        calculateSummaryStats();
    }
}

void SimulationLogger::setSimulationInfo(const string& mapName, const string& algorithm,
                                       const string& version) {
    lock_guard<std::mutex> lock(mutex);
    
    summary.mapName = mapName;
    summary.routingAlgorithm = algorithm;
    summary.implementationVersion = version;
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
    
    for (const auto& pair : vehicleStats) {
        const Vehicle& vehicle = pair.second;
        
        if (!vehicle.arrivedOnTime) {
            summary.lateVehicles++;
        }
        
        summary.totalTravelTime += vehicle.travelTime;
        summary.totalTimeWindowDeviation += vehicle.timeWindowDeviation;
    }
    summary.objectiveFunctionValue = summary.totalTravelTime + summary.totalTimeWindowDeviation;
    summaryCalculated = true;
}

void SimulationLogger::printSummary() const {
    cout << "\n=== Simulation Summary ===" << endl;
    cout << "Map: " << summary.mapName << endl;
    cout << "Algorithm: " << summary.routingAlgorithm << endl;
    cout << "Version: " << summary.implementationVersion << endl;
    cout << "Total Vehicles: " << summary.totalVehicles << endl;
    cout << "Late Vehicles: " << summary.lateVehicles << " (" 
         << (summary.totalVehicles > 0 ? (100.0 * summary.lateVehicles / summary.totalVehicles) : 0)
         << "%)" << endl;
    cout << "Total Travel Time: " << summary.totalTravelTime << " seconds" << endl;
    cout << "Average Travel Time: " 
         << (summary.totalVehicles > 0 ? (summary.totalTravelTime / summary.totalVehicles) : 0)
         << " seconds/vehicle" << endl;
    cout << "Total Time Window Deviation: " << summary.totalTimeWindowDeviation << endl;
    cout << "Average Time Window Deviation: "
         << (summary.totalVehicles > 0 ? (summary.totalTimeWindowDeviation / summary.totalVehicles) : 0)
         << " per vehicle" << endl;
    cout << "Total Algorithm Time: " << summary.totalAlgorithmTime << " seconds" << endl;
    cout << "Average Algorithm Time: "
         << (summary.totalVehicles > 0 ? (summary.totalAlgorithmTime / summary.totalVehicles) : 0)
         << " seconds/vehicle" << endl;
    cout << "Objective Function Value: " << summary.objectiveFunctionValue << endl;
}

void SimulationLogger::saveToCSV(const string& filename) {
    if (!summaryCalculated) {
        calculateSummaryStats();
    }
    
    ofstream file(filename);
    if (file.is_open()) {
        file << "SIMULATION SUMMARY" << endl;
        file << "MapName,RoutingAlgorithm,Version,TotalVehicles,LateVehicles,"
             << "TotalTravelTime,AvgTravelTime,TotalTimeWindowDeviation,"
             << "AvgTimeWindowDeviation,TotalAlgorithmTime,AvgAlgorithmTime,"
             << "ObjectiveFunctionValue,Timestamp" << endl;
             
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
             << summary.simulationTimestamp << endl;
        
        file << endl;
        
        file << "VEHICLE STATISTICS" << endl;
        file << "VehicleID,StartRoad,TargetRoad,StartTime,EndTime,TravelTime,"
             << "EarliestArrival,LatestArrival,TimeWindowDeviation,ArrivedOnTime,"
             << "PathLength" << endl;
             
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
                 << vehicle.pathLength << endl;
        }
        
        file.close();
        cout << "Saved results to " << filename << endl;
    } else {
        cerr << "Error: Unable to open file " << filename << endl;
    }
}

string SimulationLogger::getCurrentTimeStamp() const {
    auto now = chrono::system_clock::now();
    auto time = chrono::system_clock::to_time_t(now);
    
    stringstream ss;
    ss << put_time(localtime(&time), "%Y-%m-%d %H:%M:%S");
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
