/*******************************************************************************
 * BSD 3-Clause License
 * Copyright (c) 2018, Srinidhi Sreenath
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

/**
 *  @file    map.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief Map class definition
 *
 *  @section DESCRIPTION
 *
 *  Source file for class Map which stores the occupancy grid map information.
 *  The class also implements methods to determine the frontier grid cells and
 *  cluster them. It also has helper function to convert from occupancy grid
 *  coordinates to cartesian coordinates.
 *
 */
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

Map::Map() {
  mapWidth_ = 0;
  mapHeight_ = 0;
  mapResolution_ = 0.05;
}
Map::~Map() {}

std::vector<uint32_t> Map::getMapDimensions() {
  std::vector<uint32_t> mapDimensions{mapHeight_, mapWidth_};
  return mapDimensions;
}

std::pair<float, geometry_msgs::Pose> Map::getMapParameters() {
  auto mapParams = std::make_pair(mapResolution_, mapOrigin_);
  return mapParams;
}

std::vector<std::vector<Grid>> Map::getOccupancyGrid() {
  return occupancyGrid_;
}

bool Map::isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState) {
  int first = gridState.first;
  int second = gridState.second;
  for (int x = first - 1; x <= first + 1; ++x) {
    for (int y = second - 1; y <= second + 1; ++y) {
      if (x >= 0 && x < mapHeight_ && y >= 0 && y < mapWidth_ &&
          occupancyGrid_[x][y].getProbability() == -1) {
        return true;
      }
    }
  }
  return false;
}

void Map::determineFrontierGrids() {
  for (size_t i = 0; i < occupancyGrid_.size(); i++) {
    for (size_t j = 0; j < occupancyGrid_[0].size(); j++) {
      // If grid is free
      if (occupancyGrid_[i][j].getProbability() == 0) {
        auto state = occupancyGrid_[i][j].getGridState();
        auto status = isFrontierGrid(state);
        occupancyGrid_[i][j].setFrontierStatus(status);
      } else {
        occupancyGrid_[i][j].setFrontierStatus(false);
      }
      // Initialize the cluster number to -1 for all grids
      occupancyGrid_[i][j].setClusterNumber(-1);
    }
  }
}

std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
Map::clusterFrontierGrids() {
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> frontierGridSet;

  for (int i = 0; i < occupancyGrid_.size(); i++) {
    for (int j = 0; j < occupancyGrid_[0].size(); j++) {
      if (occupancyGrid_[i][j].getFrontierStatus()) {
        // 1. Top left grid
        if (i - 1 >= 0 && j - 1 >= 0 &&
            occupancyGrid_[i - 1][j - 1].getClusterNumber() != -1) {
          auto clusterNum = occupancyGrid_[i - 1][j - 1].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        } else if (i - 1 >= 0 &&  // 2. Top grid
                   occupancyGrid_[i - 1][j].getClusterNumber() != -1) {
          auto clusterNum = occupancyGrid_[i - 1][j].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        } else if (j - 1 >= 0 &&  // 3. Left grid
                   occupancyGrid_[i][j - 1].getClusterNumber() != -1) {
          auto clusterNum = occupancyGrid_[i][j - 1].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        } else if (i - 1 >= 0 && j >= 0 &&  // 4. top right grid
                   occupancyGrid_[i - 1][j + 1].getClusterNumber() != -1 &&
                   occupancyGrid_[i][j].getClusterNumber() == -1) {
          auto clusterNum = occupancyGrid_[i - 1][j + 1].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        } else if ((i - 1 >= 0 && j - 1 >= 0 &&  // 5. Both top and left
                    occupancyGrid_[i - 1][j].getClusterNumber() == -1 &&
                    occupancyGrid_[i][j - 1].getClusterNumber() == -1) ||
                   (i - 1 < 0 && j - 1 >= 0 &&
                    occupancyGrid_[i][j - 1].getClusterNumber() == -1) ||
                   (i - 1 >= 0 && j - 1 < 0 &&
                    occupancyGrid_[i - 1][j].getClusterNumber() == -1)) {
          // Define a new cluster
          size_t clusterNum = frontierGridSet.size();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);

          std::vector<std::pair<uint32_t, uint32_t>> cluster;
          auto state = occupancyGrid_[i][j].getGridState();
          cluster.push_back(state);
          frontierGridSet.push_back(cluster);
        }
      }
    }
  }
  return frontierGridSet;
}

void Map::initialize(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  mapOrigin_ = map->info.origin;
  mapHeight_ = map->info.height;
  mapWidth_ = map->info.width;
  mapResolution_ = map->info.resolution;

  occupancyGrid_.clear();

  size_t iter = 0;

  for (size_t i = 0; i < mapHeight_; i++) {
    std::vector<Grid> row;
    for (size_t j = 0; j < mapWidth_; j++) {
      Grid cell;
      cell.setGridState(i, j);
      cell.updateProbability(map->data[iter]);
      row.push_back(cell);
      iter++;
    }
    occupancyGrid_.push_back(row);
  }
}

void Map::updateOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  if (mapWidth_ != map->info.width || mapHeight_ != map->info.height ||
      mapOrigin_.position.x != map->info.origin.position.x ||
      mapOrigin_.position.y != map->info.origin.position.y ||
      mapResolution_ != map->info.resolution) {
    initialize(map);
  } else {
    size_t iter = 0;
    for (size_t i = 0; i < mapHeight_; i++) {
      for (size_t j = 0; j < mapWidth_; j++) {
        occupancyGrid_[i][j].updateProbability(map->data[iter]);
        iter++;
      }
    }
  }
}

std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
Map::getFrontierClusters() {
  determineFrontierGrids();
  auto frontierClusters = clusterFrontierGrids();
  return frontierClusters;
}

std::vector<std::pair<float, float>> Map::gridToCartesian(
    const std::vector<std::pair<uint32_t, uint32_t>> &frontiers) {
  std::vector<std::pair<float, float>> frontiersXY;
  for (auto frontier : frontiers) {
    float x, y;

    x = frontier.second * mapResolution_ + mapOrigin_.position.x;
    y = frontier.first * mapResolution_ + mapOrigin_.position.y;

    auto frontierXY = std::make_pair(x, y);
    frontiersXY.push_back(frontierXY);
  }
  return frontiersXY;
}
