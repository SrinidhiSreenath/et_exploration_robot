// ROS Headers
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// Class header files
#include "et_exploration_robot/explorer.hpp"
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

Explorer::Explorer(ros::NodeHandle nh) {}

Explorer::~Explorer() {}

void Explorer::revolve() {}

void Explorer::processMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {}

void Explorer::determineFrontiers() {}

void Explorer::explore() {}
