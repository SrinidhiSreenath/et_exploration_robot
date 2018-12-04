// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

// CPP Headers
#include <cstdint>
#include <utility>

Grid::Grid() {}
Grid::~Grid() {}

void Grid::setGridState(uint32_t height, uint32_t width) {}

std::pair<uint32_t, uint32_t> Grid::getGridState() {}

void Grid::updateProbability(int8_t updatedProbability) {}

int8_t Grid::getProbability() {}

void Grid::setFrontierStatus(bool status) {}

bool Grid::getFrontierStatus() {}

void Grid::setClusterNumber(int num) {}

int Grid::getClusterNumber(){}

;