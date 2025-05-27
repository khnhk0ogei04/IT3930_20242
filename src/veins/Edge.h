#ifndef EDGE_H
#define EDGE_H

#include<bits/stdc++.h>
using namespace std;

struct Lane{
    string id;
    int index;
    double speed;
    double length;
    string shape;
    Lane(string id="", int index=0, double speed=0.0, double length=0.0, string shape="")
        : id(id), index(index), speed(speed), length(length), shape(shape) {}
};

class Edge {
private:
    string id;
    string from;
    string to;
    double length;
    string function;
    vector<Lane> lanes;
public:
    Edge();
    Edge(string id, string from, string to, double length, string function);
    Edge(string id, string from, string to, vector<Lane> lanes, double length = 0.0, string function = "");

    // Getter:
    string getId() const {
        return id;
    }
    string getFrom() const{
        return from;
    }
    string getTo() const{
        return to;
    }
    double getLength() const{
        return length;
    }
    string getFunction() const {
        return function;
    }
    vector<Lane> getLanes() const {
        return lanes;
    }
    double getMaxSpeed() const;

    // Setter:
    void setId(string newId){
        id = newId;
    }
    void setFrom(string newFrom){
        from = newFrom;
    }
    void setTo(string newTo){
        to =  newTo;
    }
    void setLength(double newLength){
        length = newLength;
    }
    void setFunction(string newFunction) {
        function = newFunction;
    }
    void addLane(Lane lane) {
        lanes.push_back(lane);
    }
};

#endif // EDGE_H
