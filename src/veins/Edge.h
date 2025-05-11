#ifndef EDGE_H
#define EDGE_H

#include<string>
#include<vector>
using namespace std;

struct Lane{
    string id;
    int index;
    double speed;
    double length;
    string shape;
    Lane(const string& id="", int index=0, double speed=0.0, double length=0.0, const string& shape="")
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
    Edge(const string& id, const string& from, const string& to, double length, const string& function);
    
    // Add constructor that accepts a vector of lanes
    Edge(const string& id, const string& from, const string& to, const vector<Lane>& lanes, double length = 0.0, const string& function = "");

    // Getter:
    const string& getId() const {return id;}
    const string& getFrom() const{
        return from;
    }
    const string& getTo() const{
        return to;
    }
    double getLength() const{
        return length;
    }
    const string& getFunction() const {return function;}
    const vector<Lane>& getLanes() const {return lanes;}

    // Setter:
    void setId(const string &newId){ id = newId; }
    void setFrom(const string &newFrom){
        from = newFrom;
    }
    void setTo(const string &newTo){
        to =  newTo;
    }
    void setLength(double newLength){
        length = newLength;
    }
    void setFunction(const string &newFunction) {function = newFunction;}
    void addLane(const Lane& lane) {lanes.push_back(lane);}
};

#endif // EDGE_H
