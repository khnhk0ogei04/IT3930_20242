#include "Node.h"

using namespace std;

Node::Node(){
    this->id = "";
    this->x = 0.0;
    this->y = 0.0;
}

Node::Node(string id, double x, double y){
    this->id = id;
    this->x = x;
    this->y = y;
}
