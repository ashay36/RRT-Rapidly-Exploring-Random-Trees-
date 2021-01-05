#ifndef RRT_H_
#define RRT_H_
#include <vector>
#include "Eigen/Dense"
#include "node.cpp"

using Eigen::MatrixXd;
using std::vector;


// RRT class to perform the path finding algorithm
class RRT{
public:
    Node start;  // starting node
    Node goal;  // goal node
    vector<Node> node_list;  // list of all the valid nodes generated
    float min_point, max_point, max_len;
    int goal_sample_rate, max_iter;

    // constructor
    RRT(const float start_[],
        const float goal_[],
        const float min_max_points_[],
        const float max_len_,
        const int goal_sample_rate_,
        const int max_iter_);

    /*function to find the path between start and goal nodes given a list of obstacles and path and node vectors to write required values 
    returns true if path is found, false otherwise */
    bool findPath(vector< vector<float> > &path_, vector<Node> &nodes_, MatrixXd obstacle_list);
    
    /* function to move from one node to the other */
    Node move(Node from_node, Node to_node);
    
    /* function to generate the final path if generated, between start and goal nodes */
    vector< vector<float> > generateFinalCourse(Node final_node);

    /* function to generate a random node */
    Node getRandomNode();
            
    /* function to find the index of the node nearest to the given node */
    int getNearestNodeIndex(Node rnd);
    
    /* function to check if the given node collides with any of the obstacles */
    bool checkCollision(Node node, MatrixXd obstacle_list);

    /* function to calculate the distance from given x and y coordinates to the goal node */
    float calcDistToGoal(float x, float y);

    /* function to calculate distance between two given nodes */
    float calcDist(Node from_node, Node to_node);

    /* function to calculate angle between two given nodes */
    float calcAngle(Node from_node, Node to_node);
};

#endif // RRT_H_
