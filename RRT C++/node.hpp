#ifndef NODE_H_
#define NODE_H_
#include <vector>

// Node class to represent a node in the graph
class Node{
public:
    float x;  // x coordinate of the node
    float y;  // y coordinate of the node
    std::vector<float> path_x;  // x coordinates of the path towards new node
    std::vector<float> path_y;  // y coordinates of the path towards new node

    Node();
    Node(float x_, float y_);
};

#endif // NODE_H_