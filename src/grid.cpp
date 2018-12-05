// CPP Headers
#include <cstdint>
#include <utility>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

Grid::Grid() {}
Grid::~Grid() {}

void Grid::setGridState(const uint32_t &height, const uint32_t &width) {}

std::pair<uint32_t, uint32_t> Grid::getGridState() {}

void Grid::updateProbability(const int8_t &updatedProbability) {}

int8_t Grid::getProbability() {}

void Grid::setFrontierStatus(const bool &status) {}

bool Grid::getFrontierStatus() {}

void Grid::setClusterNumber(const int &num) {}

int Grid::getClusterNumber() {}
