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

/**
 * Lớp ghi log mô phỏng - ghi nhận các thông tin và thống kê về các xe
 */
class SimulationLogger {
public:
    // Singleton pattern
    static SimulationLogger& getInstance() {
        static SimulationLogger instance;
        return instance;
    }

    // Cấu trúc lưu trữ thông tin về một xe
    struct VehicleStats {
        int vehicleId;                  // ID của xe
        std::string startingRoad;       // Đoạn đường bắt đầu
        std::string targetRoad;         // Đoạn đường đích
        double startTime;               // Thời điểm xe bắt đầu
        double endTime;                 // Thời điểm xe kết thúc
        double travelTime;              // Thời gian đi (endTime - startTime)
        double earliestArrival;         // Thời gian đến sớm nhất (time window)
        double latestArrival;           // Thời gian đến trễ nhất (time window)
        double timeWindowDeviation;     // Độ lệch so với khung thời gian
        bool arrivedOnTime;             // Xe có đến kịp không
        std::vector<std::string> path;  // Đường đi của xe
        double pathLength;              // Độ dài đường đi
        double algorithmTime;           // Thời gian để giải thuật tìm ra đường đi

        VehicleStats() : 
            vehicleId(-1), startTime(0), endTime(0), travelTime(0),
            earliestArrival(0), latestArrival(0), timeWindowDeviation(0),
            arrivedOnTime(true), pathLength(0), algorithmTime(0) {}
    };

    // Cấu trúc lưu trữ thông tin tổng hợp về mô phỏng
    struct SimulationSummary {
        std::string mapName;                // Tên bản đồ
        std::string routingAlgorithm;       // Tên giải thuật tìm đường
        std::string implementationVersion;  // Phiên bản cài đặt
        int totalVehicles;                  // Tổng số xe
        int lateVehicles;                   // Số xe đến không kịp
        double totalTravelTime;             // Tổng thời gian các xe chạy
        double totalTimeWindowDeviation;    // Tổng độ lệch khung thời gian
        double totalAlgorithmTime;          // Tổng thời gian giải thuật tìm đường
        std::string simulationTimestamp;    // Thời điểm mô phỏng

        SimulationSummary() : 
            totalVehicles(0), lateVehicles(0), totalTravelTime(0), 
            totalTimeWindowDeviation(0), totalAlgorithmTime(0) {}
    };

    // Ghi nhận thông tin về một xe
    void recordVehicleStart(int vehicleId, const std::string& startRoad, double startTime);
    
    // Cập nhật thông tin đích và khung thời gian cho xe
    void updateVehicleDestination(int vehicleId, const std::string& targetRoad, 
                                double earliestArrival, double latestArrival,
                                const std::vector<std::string>& path, double pathLength);
    
    // Ghi nhận thời gian giải thuật tìm đường cho xe
    void recordAlgorithmTime(int vehicleId, double algorithmTime);
    
    // Ghi nhận thời điểm xe đến đích
    void recordVehicleEnd(int vehicleId, double endTime);
    
    // Ghi nhận các thông tin về mô phỏng
    void setSimulationInfo(const std::string& mapName, const std::string& algorithm, 
                          const std::string& version);
    
    // Lưu toàn bộ dữ liệu xuống file CSV
    void saveToCSV(const std::string& vehicleStatsFilename = "vehicle_stats.csv", 
                  const std::string& summaryFilename = "simulation_summary.csv");
    
    // Kiểm tra xem đã ghi nhận tất cả xe kết thúc chưa
    bool allVehiclesFinished() const;
    
    // In thông tin tóm tắt
    void printSummary() const;

private:
    SimulationLogger(); // Private constructor for singleton
    ~SimulationLogger();
    
    // Không cho phép sao chép hoặc gán
    SimulationLogger(const SimulationLogger&) = delete;
    SimulationLogger& operator=(const SimulationLogger&) = delete;
    
    // Tính toán các thống kê tổng hợp
    void calculateSummaryStats();
    
    // Lấy thời gian hiện tại định dạng string
    std::string getCurrentTimeStamp() const;
    
    // Tính độ lệch so với khung thời gian
    double calculateTimeWindowDeviation(double endTime, double earliestArrival, double latestArrival) const;
    
    std::map<int, VehicleStats> vehicleStats;  // Thông tin về mỗi xe
    SimulationSummary summary;                 // Thông tin tổng hợp
    std::mutex mutex;                          // Mutex để bảo vệ truy cập đồng thời
    bool summaryCalculated;                    // Đã tính tổng hợp chưa
};

} // namespace veins

#endif /* VEINS_SIMULATIONLOGGER_H_ */ 