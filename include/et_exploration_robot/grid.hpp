#ifndef INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_
#define INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_

// CPP Headers
#include <cstdint>
#include <utility>

class Grid {
 private:
  uint32_t gridHeight_;
  uint32_t gridWidth_;
  int8_t OccupancyProbability_;
  bool isFrontierCell_;
  int clusterNumber_;

 public:
  Grid();
  ~Grid();

  void setGridState(const uint32_t &height, const uint32_t &width);

  std::pair<uint32_t, uint32_t> getGridState();

  void updateProbability(const int8_t &updatedProbability);

  int8_t getProbability();

  void setFrontierStatus(const bool &status);

  bool getFrontierStatus();

  void setClusterNumber(const int &num);

  int getClusterNumber();
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_
