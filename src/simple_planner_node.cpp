/**
 * @file    elevation_map_node.cpp
 * @brief   Node that requests an elevation map from covins
 *          and reconstructs a grid map from the points cloud
 * @author  David Morilla Cabello, V4RL
 * @date    30.07.2021
 */

#include <ros/ros.h>
#include "mav_simple_planner/simple_planner.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mav_simple_planner");
    ros::NodeHandle nodeHandle("~");
    ros::NodeHandle nodeHandlePublic;
    mav_simple_planner::Planner mavPlanner(nodeHandlePublic, nodeHandle);

    // Spin
    ros::spin();    
    return 0;
}
