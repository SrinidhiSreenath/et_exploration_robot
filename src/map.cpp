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

bool Map::isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState) {
  for (int x = gridState.first - 1; x <= gridState.first + 1; ++x) {
    for (int y = gridState.second - 1; y <= gridState.second + 1; ++y) {
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
      }
      // Initialize the cluster number to -1 for all grids
      occupancyGrid_[i][j].setClusterNumber(-1);
    }
  }
}

std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
Map::clusterFrontierGrids() {
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> frontierGridSet;
  // Left cluster
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
        } else if ((i - 1 >= 0 && j - 1 >= 0 &&  // 2. Both top and left
                    occupancyGrid_[i - 1][j].getClusterNumber() == -1 &&
                    occupancyGrid_[i][j - 1].getClusterNumber() == -1) ||
                   (i - 1 < 0 && j - 1 >= 0 &&
                    occupancyGrid_[i][j - 1].getClusterNumber() == -1) ||
                   (i - 1 >= 0 && j - 1 < 0 &&
                    occupancyGrid_[i - 1][j].getClusterNumber() == -1)) {
          // New Cluster
          size_t clusterNum = frontierGridSet.size();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);

          std::vector<std::pair<uint32_t, uint32_t>> cluster;
          auto state = occupancyGrid_[i][j].getGridState();
          cluster.push_back(state);
          frontierGridSet.push_back(cluster);
        } else if (i - 1 >= 0 &&  // 3. Top grid
                   occupancyGrid_[i - 1][j].getClusterNumber() != -1) {
          auto clusterNum = occupancyGrid_[i - 1][j].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        } else if (j - 1 >= 0 &&  // 4. Left grid
                   occupancyGrid_[i][j - 1].getClusterNumber() != -1) {
          auto clusterNum = occupancyGrid_[i][j - 1].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
        }
      }
    }
  }
  // Right cluster i.e top right grid
  for (int i = 0; i < occupancyGrid_.size(); i++) {
    for (int j = 0; j < occupancyGrid_[0].size(); j++) {
      if (occupancyGrid_[i][j].getFrontierStatus()) {
        if (i - 1 >= 0 && j >= 0 &&
            occupancyGrid_[i - 1][j + 1].getClusterNumber() != -1 &&
            occupancyGrid_[i][j].getClusterNumber() == -1) {
          auto clusterNum = occupancyGrid_[i - 1][j + 1].getClusterNumber();
          occupancyGrid_[i][j].setClusterNumber(clusterNum);
          auto state = occupancyGrid_[i][j].getGridState();
          frontierGridSet[clusterNum].push_back(state);
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
