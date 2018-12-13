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
  uint32_t mapWidth_;
  uint32_t mapHeight_;
  float mapResolution_;

  geometry_msgs::Pose mapOrigin_;

  std::vector<std::vector<Grid>> occupancyGrid_;

  bool isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState);

  void determineFrontierGrids();

  std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
  clusterFrontierGrids();

 public:
  Map();
  ~Map();

  std::vector<uint32_t> getMapDimensions();

  std::pair<float, geometry_msgs::Pose> getMapParameters();

  std::vector<std::vector<Grid>> getOccupancyGrid();

  void initialize(const nav_msgs::OccupancyGrid::ConstPtr &map);

  void updateOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> getFrontierClusters();

  std::vector<std::pair<float, float>> gridToCartesian(
      const std::vector<std::pair<uint32_t, uint32_t>> &frontiers);
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_MAP_HPP_
