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
 *  @file    grid.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief Grid class definitions
 *
 *  @section DESCRIPTION
 *
 *  Source file for class Grid which stores information of a single grid cell in
 *  the occupancy grid map. The class implements setter and getter function for
 *  the grid cell height, width coordinate. occupancy probability, frontier
 *  status and cluster number.
 *
 */
// CPP Headers
#include <cstdint>
#include <utility>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

Grid::Grid() {
  gridHeight_ = 0;
  gridWidth_ = 0;
  occupancyProbability_ = -1;
  isFrontierCell_ = false;
  clusterNumber_ = -1;
}
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
