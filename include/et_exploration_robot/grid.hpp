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
 *  @file    grid.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief Grid class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Grid which stores information of a single grid cell in
 *  the occupancy grid map.
 *
 */
#ifndef INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_
#define INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_

// CPP Headers
#include <cstdint>
#include <utility>

/**
 *  @brief Class Grid
 *
 *  The following class Grid stores the information of a single grid cell in the
 *  occupancy grid map. Information of the grid cell include grid height, width
 *  coordinates, occupancy probability, frontier cell status and frontier
 *  cluster number.
 */
class Grid {
 private:
  uint32_t
      gridHeight_;  ///< height coordinate of the grid cell in the occupancy map
  uint32_t
      gridWidth_;  ///< width coordinate of the grid cell in the occupancy map
  int8_t occupancyProbability_;  ///< probability of the grid cell occupancy
                                 ///< status. Its value is -1 if unexplored, 0
                                 ///< if free and 100 if occupied
  bool isFrontierCell_;  ///< bool value to hold the frontier status of the grid
                         ///< cell. True if the grid cell is a frontier, false
                         ///< otherwise
  int clusterNumber_;  ///< The cluster number that the grid cell belongs to. If
                       ///< the cell isn't a frontier cell and hence it belongs
                       ///< to no cluster and the value is -1

 public:
  /**
   *   @brief  Default constructor for Grid.
   *
   *   @param  none
   *
   *   @return void
   */
  Grid();

  /**
   *   @brief  Destructor for Grid
   *
   *   @param  none
   *
   *   @return void
   */
  ~Grid();

  /**
   *   @brief  setter function to set the state of the grid cell
   *
   *   @param  height is the unsigned integer denoting the height coordinate of
   *           the grid cell in the occupancy map
   *   @param  width is the unsigned integer denoting the width coordinate of
   *           the grid cell in the occupancy map
   *
   *   @return void
   */
  void setGridState(const uint32_t &height, const uint32_t &width);

  /**
   *   @brief  getter function to obtain the state of the grid cell
   *
   *   @param  none
   *
   *   @return pair of unsigned integer containing the height and width
   *           coordinate of the grid cell
   */
  std::pair<uint32_t, uint32_t> getGridState();

  /**
   *   @brief  setter function to set the probablibility of occupancy of the
   *           grid cell
   *
   *   @param  updatedProbability is the integer denoting the occupancy
   *           probablibility of the grid cell. -1 means unexplored, 0 means
   *           free and 100 means occupied.
   *
   *   @return void
   */
  void updateProbability(const int8_t &updatedProbability);

  /**
   *   @brief  getter function to obtain the  occupancy
   *           probablibility of the grid cell
   *
   *   @param  none
   *
   *   @return integer denoting the occupancy probablibility
   */
  int8_t getProbability();

  /**
   *   @brief  setter function to set the frontier status of the
   *           grid cell
   *
   *   @param  status is the boolean value denoting whether the grid cell is a
   *           frontier cell or not
   *
   *   @return void
   */
  void setFrontierStatus(const bool &status);

  /**
   *   @brief  getter function to obtain the frontier status of the
   *           grid cell
   *
   *   @param  none
   *
   *   @return boolean value. True if the grid cell is a frontier cell, false
   *           otherwise
   */
  bool getFrontierStatus();

  /**
   *   @brief  setter function to set the cluster number of the
   *           grid cell
   *
   *   @param  num is the integer denoting the cluster number
   *
   *   @return void
   */
  void setClusterNumber(const int &num);

  /**
   *   @brief  getter function to obtain the the cluster number of the
   *           grid cell
   *
   *   @param  none
   *
   *   @return integer value. If the cell is not a frontier cell, then returns
   *           -1
   */
  int getClusterNumber();
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_GRID_HPP_
