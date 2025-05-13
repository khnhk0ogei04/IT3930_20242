#include "Node.h"

using namespace std;

Node::Node() : id(""), x(0.0), y(0.0){
}

Node::Node(string id, double x, double y)
    : id(id), x(x), y(y) {
} 