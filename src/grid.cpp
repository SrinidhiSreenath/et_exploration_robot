// CPP Headers
#include <cstdint>
#include <utility>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

Grid::Grid() {}
Grid::~Grid() {}

void Grid::setGridState(const uint32_t &height, const uint32_t &width) {
  gridHeight_ = height;
  gridWidth_ = width;
}

std::pair<uint32_t, uint32_t> Grid::getGridState() {
  auto state = std::make_pair(gridHeight_, gridWidth_);
  return state;
}

void Grid::updateProbability(const int8_t &updatedProbability) {
  occupancyProbability_ = updatedProbability;
}

int8_t Grid::getProbability() { return occupancyProbability_; }

void Grid::setFrontierStatus(const bool &status) { isFrontierCell_ = status; }

bool Grid::getFrontierStatus() { return isFrontierCell_; }

void Grid::setClusterNumber(const int &num) { clusterNumber_ = num; }

int Grid::getClusterNumber() { return clusterNumber_; }
