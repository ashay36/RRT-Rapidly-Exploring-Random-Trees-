#include <math.h>
#include <cmath>
#include <random>
#include "rrt.hpp"

using namespace std;

RRT::RRT(const float start_[],
         const float goal_[], 
         const float min_max_points_[], 
         const float max_len_, 
         const int goal_sample_rate_, 
         const int max_iter_){
    
    start.x = start_[0];
    start.y = start_[1];
    start.path_x = {start_[0]};
    start.path_y = {start_[1]};

    goal.x = goal_[0];
    goal.y = goal_[1];

    min_point = min_max_points_[0];
    max_point = min_max_points_[1];

    max_len = max_len_;
    goal_sample_rate = goal_sample_rate_;
    max_iter = max_iter_;    
}

bool RRT::findPath(vector< vector<float> > &path_, vector<Node> &nodes_, MatrixXd obstacle_list){
    node_list.push_back(start);

    for(int i = 0; i < max_iter; i++){
        // generating a random node
        Node random_node = getRandomNode();
        // getting index of the nearest node
        int nearest_node_index = getNearestNodeIndex(random_node);
        // getting the node at the corresponding index
        Node nearest_node = node_list[nearest_node_index];
        // moving in the direction of the random node
        Node new_node = move(nearest_node, random_node);
        float dist = calcDist(random_node, nearest_node);

        if (dist > max_len){
            continue;
        }

        // check if collision occurs while moving
        if (checkCollision(new_node, obstacle_list)){
            // if not, add the node to the node list
            node_list.push_back(new_node);
            // check if the node is near the goal node
            if (calcDistToGoal(node_list[node_list.size() - 1].x, node_list[node_list.size() - 1].y) <= max_len){
                // if yes move towards the goal node
                Node final_node = move(node_list[node_list.size() - 1], goal);
                // check if collision occurs while moving
                if (checkCollision(final_node, obstacle_list)){
                    node_list.push_back(final_node);
                    // if not, get the final path and the nodes
                    path_ = generateFinalCourse(final_node);
                    nodes_ = node_list;

                    return true;
                }
            } 
        }
    }
    return false;
}

Node RRT::move(Node from_node, Node to_node){
    Node new_node(to_node.x, to_node.y);

    new_node.path_x = from_node.path_x;
    new_node.path_y = from_node.path_y;

    new_node.path_x.push_back(new_node.x);
    new_node.path_y.push_back(new_node.y);

    return new_node;
}

vector< vector<float> > RRT::generateFinalCourse(Node final_node){    
    vector< vector<float> > path;

    path.push_back(final_node.path_x);
    path.push_back(final_node.path_y);

    return path;
}

Node RRT::getRandomNode(){
    float a = min_point + (static_cast <float>(rand()) / (static_cast <float>(RAND_MAX) / (max_point - min_point)));
    float b = min_point + (static_cast <float>(rand()) / (static_cast <float>(RAND_MAX) / (max_point - min_point)));
    Node node(a, b);
    return node;
}

int RRT::getNearestNodeIndex(Node rnd){
    float val;
    float min_val = 200000.0;
    int min_ind;
    for (int i = 0; i < node_list.size(); i++){
        val = pow(node_list[i].x - rnd.x, 2) + pow(node_list[i].y - rnd.y, 2);
        if (min_val > val){
            min_val = val;
            min_ind = i; 
        }
    }
    return min_ind;
}

bool RRT::checkCollision(Node node, MatrixXd obstacle_list){
    float dx, dy, val, min_val = 2000000.0;

    // find the minimum squared distance of each to the obstacle center
    for (int i = 0; i < obstacle_list.col(0).size(); i++){
        for (int j = 0; j < node.path_x.size(); j++){
            dx = obstacle_list(i, 0) - node.path_x[j];
            dy = obstacle_list(i, 1) - node.path_y[j];
            val = pow(dx , 2) + pow(dy, 2);
            if (min_val > val){
                min_val = val;
            }
        }
    }

    // if the min distance sqaured is less than the sqaured of the radius of any
    // obstacle, then there is collision
    for (int i = 0; i < obstacle_list.col(0).size(); i++){
        if (min_val <= pow(obstacle_list(i, 2), 2)){
            return false;  // collision
        }
    }
    return true;  // no collision
}

float RRT::calcDistToGoal(float x, float y){
    float dx = x - goal.x;
    float dy = y - goal.y;

    return hypot(dx, dy);
}

float RRT::calcDist(Node from_node, Node to_node){
    float dx = to_node.x - from_node.x;
    float dy = to_node.y - from_node.y;

    return hypot(dx, dy);
}

float RRT::calcAngle(Node from_node, Node to_node){
    float dx = to_node.x - from_node.x;
    float dy = to_node.y - from_node.y;

    return atan2(dx, dy);
}