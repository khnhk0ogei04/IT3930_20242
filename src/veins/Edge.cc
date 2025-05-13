#include "Edge.h"
using namespace std;

Edge::Edge(){
    this->id = "";
    this->from = "";
    this->to = "";
    this->length = 0.0;
    this->function = "";
}

Edge::Edge(string id, string from, string to, double length, string function){
    this->id = id;
    this->from = from;
    this->to = to;
    this->length = length;
    this->function = function;
}

Edge::Edge(string id, string from, string to, vector<Lane> lanes, double length, string function)
    : id(id), from(from), to(to), length(length), function(function), lanes(lanes) {
    if (length == 0.0 && !lanes.empty()) {
        double maxLength = 0.0;
        for (Lane lane : lanes) {
            if (lane.length > maxLength) {
                maxLength = lane.length;
            }
        }
        this->length = maxLength;
    }
}
