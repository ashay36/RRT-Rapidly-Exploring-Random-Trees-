#include <iostream>
#include "matplotlib-cpp/matplotlibcpp.h"
#include "rrt.cpp"

namespace plt = matplotlibcpp;

// main function
int main(){
    const float start_[2] = {4.0, 6.0};
    const float goal_[2] = {10.0, 8.0};
    const float min_max_points_[2] = {-2.0, 15.0};
    MatrixXd obstacle_list = MatrixXd(7, 3); 
    obstacle_list << 5.0, 5.0, 1.0,  // (center x, center y, radius)
                     3.0, 6.0, 2.0,
                     3.0, 8.0, 2.0,
                     3.0, 10.0, 2.0,
                     7.0, 5.0, 2.0,
                     9.0, 5.0, 2.0,
                     8.0, 10.0, 1.0;
    const float max_len_ = 3.0;
    const float goal_sample_rate_ = 5;
    const float max_iter_ = 500;

    // creating an instance of RRT class
    RRT rrt(start_, goal_, min_max_points_, max_len_, goal_sample_rate_, max_iter_);

    vector< vector<float> > path; // stores the final path
    vector<Node> nodes;  // stores all the nodes
    bool found;

    // finding the path
    found = rrt.findPath(path, nodes, obstacle_list);

    plt::title("RRT Path Planning");
    plt::grid(true);
    plt::axis("equal");

    // plotting obstacles
    for (unsigned int i = 0; i < obstacle_list.col(0).size(); i++){
        vector<float> cx, cy;
        for (int j = -1; j < 360; j += 5){
            float angle = j * M_PI / 180.0;
            float x = obstacle_list(i, 0);
            float y = obstacle_list(i, 1);
            float r = obstacle_list(i, 2);
            cx.push_back(x + r*cos(angle));
            cy.push_back(y + r*sin(angle));
        }
        plt::plot(cx, cy);
    }

    // plotting nodes and their paths
    for (unsigned int i = 1; i < nodes.size(); i++){
        vector<float> node_x, node_y;
        node_x = {nodes[i].x, nodes[i].path_x[nodes[i].path_x.size()-2]};
        node_y = {nodes[i].y, nodes[i].path_y[nodes[i].path_y.size()-2]};

        plt::plot(node_x, node_y, "g");
        plt::pause(0.01);
    } 

    if (found){
        cout << "Path Found!" << endl;
        plt::plot(path[0], path[1], "r");  // plotting the final path
    } else {
        cout << "Path Not Found!" << endl;
    }
    
    plt::show();

    return 0;
}