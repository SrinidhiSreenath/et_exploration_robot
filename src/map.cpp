// ROS Headers
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

Map::Map() {}
Map::~Map() {}

bool Map::isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState) {}

void Map::determineFrontierGrids() {}

std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
Map::clusterFrontierGrids() {}

void Map::initialize(const nav_msgs::OccupancyGrid::ConstPtr &msg) {}

void Map::updateOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &msg) {}

std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
Map::getFrontierClusters() {}

std::vector<std::pair<float, float>> Map::gridToCartesian(
    const std::vector<std::pair<uint32_t, uint32_t>> &frontiers) {}
