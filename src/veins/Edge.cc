#include "Edge.h"
using namespace std;

Edge::Edge() : id(""), from(""), to(""), length(0.0) {
}

Edge::Edge(const string& id, const string& from, const string& to, double length, const string& function)
    : id(id), from(from), to(to), length(length), function(function) {
}

// New constructor that accepts lanes
Edge::Edge(const string& id, const string& from, const string& to, const vector<Lane>& lanes, double length, const string& function)
    : id(id), from(from), to(to), length(length), function(function), lanes(lanes) {
    
    // If length is 0, calculate max length from lanes
    if (length == 0.0 && !lanes.empty()) {
        double maxLength = 0.0;
        for (const auto& lane : lanes) {
            if (lane.length > maxLength) {
                maxLength = lane.length;
            }
        }
        this->length = maxLength;
    }
}
