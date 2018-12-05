#ifndef INCLUDE_ET_EXPLORATION_ROBOT_MAP_HPP_
#define INCLUDE_ET_EXPLORATION_ROBOT_MAP_HPP_

// ROS Headers
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Grid class header file
#include "et_exploration_robot/grid.hpp"

class Map {
 private:
  uint32_t mapWidth;
  uint32_t mapHeight;
  float mapResolution;

  geometry_msgs::Pose mapOrigin;
  geometry_msgs::Pose gridOriginCellPose;

  std::vector<std::vector<Grid>> occupancyGrid;

  bool isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState);

  void determineFrontierGrids();

  std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
  clusterFrontierGrids();

 public:
  Map();
  ~Map();

  void initialize(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  void updateOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> getFrontierClusters();

  std::vector<std::pair<float, float>> gridToCartesian(
      const std::vector<std::pair<uint32_t, uint32_t>> &frontiers);
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_MAP_HPP_
