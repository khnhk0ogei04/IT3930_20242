#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include <string>
#include <vector>
#include <random>
#include "GraphProcessor.h"
#include "Vehicle.h"

using namespace std;

namespace veins {

class TaskGenerator {
public:
    TaskGenerator(const GraphProcessor& processor);
    vector<string> getPotentialDestinationEdges(int n, const vector<string>& currentSourceEdges, unsigned seedValue = 0);
    bool existsValidAssignment(const vector<string>& sources, const vector<string>& destinations);
    mt19937& getRNG() { return rng; }
    vector<Destination> generateDestinationsWithTimeWindows(
        int n, 
        const vector<string>& currentSourceEdges, 
        const Graph& graph,
        unsigned seedValue = 0);
private:
    const GraphProcessor& graphProcessor;
    mt19937 rng;
};

} // namespace veins

#endif // TASK_GENERATOR_H
