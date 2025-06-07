#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include <string>
#include <vector>
#include <random>
#include "GraphProcessor.h"

using namespace std;

namespace veins {

struct TimeWindow {
    double earliness;  // Earliest arrival time
    double tardiness;  // Latest arrival time
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

class TaskGenerator {
public:
    TaskGenerator(const GraphProcessor& processor);
    vector<Destination> generateDestinations(int n, unsigned seedValue = 0);
    vector<string> getPotentialDestinationEdges(int n, const vector<string>& huidigeSourceEdges, unsigned seedValue = 0);
    bool existsValidAssignment(const vector<string>& sources, const vector<string>& destinations);
    mt19937& getRNG() { return rng; }
private:
    const GraphProcessor& graphProcessor;
    mt19937 rng;
};

} // namespace veins

#endif // TASK_GENERATOR_H
