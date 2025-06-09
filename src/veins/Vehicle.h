#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>
#include <chrono>

using namespace std;

namespace veins {

struct TimeWindow {
    double earliness;
    double tardiness;
    TimeWindow(double early = 0.0, double late = 0.0)
        : earliness(early), tardiness(late) {}
};

struct Destination {
    string nodeId; 
    TimeWindow timeWindow;  
    Destination(const string& id = "", const TimeWindow& tw = TimeWindow())
        : nodeId(id), timeWindow(tw) {}

    bool operator<(const Destination& other) const {
        if (nodeId != other.nodeId)
            return nodeId < other.nodeId;
        if (timeWindow.earliness != other.timeWindow.earliness)
            return timeWindow.earliness < other.timeWindow.earliness;
        return timeWindow.tardiness < other.timeWindow.tardiness;
    }
};

struct Vehicle {
    int vehicleId;
    int simulationId;
    string sumoId;
    int index;

    string startingRoad;
    string targetRoad;
    string viaRoad;

    double departTime;
    double startTime;
    double endTime;
    double travelTime;
    double estimatedTravelTime;

    double earliestArrival;
    double latestArrival;
    double timeWindowDeviation;
    bool arrivedOnTime;

    vector<string> plannedPath;
    vector<string> actualPath;
    vector<string> lastSentPath;
    double pathLength;
    Destination assignedDestination;
    double algorithmTime;
    double lastMessageTime;

    bool hasLoggedDeparture;
    bool isActive;
    Vehicle() : 
        vehicleId(-1), simulationId(-1), sumoId(""), index(-1),
        startingRoad(""), targetRoad(""), viaRoad(""),
        departTime(0.0), startTime(0.0), endTime(0.0), travelTime(0.0), estimatedTravelTime(0.0),
        earliestArrival(0.0), latestArrival(0.0), timeWindowDeviation(0.0), arrivedOnTime(true),
        pathLength(0.0), algorithmTime(0.0), lastMessageTime(0.0),
        hasLoggedDeparture(false), isActive(false) {}
    
    Vehicle(int id, const string& sumo_id, int idx) :
        vehicleId(id), simulationId(16 + 6 * id), sumoId(sumo_id), index(idx),
        startingRoad(""), targetRoad(""), viaRoad(""),
        departTime(0.0), startTime(0.0), endTime(0.0), travelTime(0.0), estimatedTravelTime(0.0),
        earliestArrival(0.0), latestArrival(0.0), timeWindowDeviation(0.0), arrivedOnTime(true),
        pathLength(0.0), algorithmTime(0.0), lastMessageTime(0.0),
        hasLoggedDeparture(false), isActive(true) {}
    
    void setRouteInfo(const string& from, const string& to, double depart, const string& via = "") {
        startingRoad = from;
        targetRoad = to;
        departTime = depart;
        viaRoad = via;
    }
    
    void setDestination(const Destination& dest) {
        assignedDestination = dest;
        targetRoad = dest.nodeId;
        earliestArrival = dest.timeWindow.earliness;
        latestArrival = dest.timeWindow.tardiness;
    }

    void setPath(const vector<string>& path, double length) {
        plannedPath = path;
        pathLength = length;
    }
    
    void start(double time) {
        startTime = time;
        isActive = true;
    }

    void finish(double time) {
        endTime = time;
        travelTime = endTime - startTime;
        isActive = false;
        hasLoggedDeparture = true;

        if (endTime < earliestArrival) {
            timeWindowDeviation = earliestArrival - endTime;
            arrivedOnTime = false;
        } else if (endTime > latestArrival) {
            timeWindowDeviation = endTime - latestArrival;
            arrivedOnTime = false;
        } else {
            timeWindowDeviation = 0.0;
            arrivedOnTime = true;
        }
    }

    void updateMessageTime(double time) {
        lastMessageTime = time;
    }

    bool hasTimedOut(double currentTime, double timeout = 10.0) const {
        return (currentTime - lastMessageTime) > timeout;
    }
};

} // namespace veins

#endif // VEHICLE_H 
