#ifndef NODE_H
#define NODE_H

#include <string>

using namespace std;
class Node {
private:
    string id;
    double x;
    double y;

public:
    Node();
    // ~Node();
    Node(string id, double x = 0.0, double y = 0.0);
    string getId() const {return id;}
    double getX() const {return x;}
    double getY() const {return y;}

    void setId(string newId) {id = newId;}
    void setX(double newX){
        x = newX;
    }
    void setY(double newY){
        y = newY;
    }
};

#endif // NODE_H
