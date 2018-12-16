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
 *  @file    map.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief Map class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Map which stores the occupancy grid map information
 *  and is used to determine the frontiers in the occupancy grid map.
 *
 */
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

/**
 *  @brief Class Map
 *
 *  The following class Map stores the occupancy grid map information and
 *  determines the frontiers in the map and also performs clustering of the
 *  frontiers into different clusters.
 */
class Map {
 private:
  uint32_t mapWidth_;   ///< unsigned integer to denote the number of grid cells
                        ///< in the width of the occupancy grid map
  uint32_t mapHeight_;  ///< unsigned integer to denote the number of grid cells
                        ///< in the height of the occupancy grid map
  float mapResolution_;  ///< float variable to hold the resolution of the
                         ///< occupancy grid i.e the length of each grid cell in
                         ///< meters

  geometry_msgs::Pose
      mapOrigin_;  ///< ROS standard message to hold the origin of the occupancy
                   ///< grid map wrt the world frame

  std::vector<std::vector<Grid>>
      occupancyGrid_;  ///< A matrix to denote the occupancy grid.

  /**
   *   @brief  funtion to determine whether the given grid cell is a frontier
   *           cell i.e has atleast one unexplored neighbouring grid cell.
   *
   *   @param  gridState is a pair of unsigned integer points denoting the
   *           height and width coordinate of the grid cell in the occupancy map
   *
   *   @return bool type. True if the grid cell is a frontier cell and false
   *           otherwise.
   */
  bool isFrontierGrid(const std::pair<uint32_t, uint32_t> &gridState);

  /**
   *   @brief  function to determine which grid cells are frontier cells and
   *           update the state of all cells in the map.
   *
   *   @param  none
   *
   *   @return void
   */
  void determineFrontierGrids();

  /**
   *   @brief  function that clusters the grid cells into different clusters
   *           based on the state of neighbouring frontier cells
   *
   *   @param  none
   *
   *   @return container of vectors of frontier grid cells as paired unsigned
   *           integers. Each vector denotes a cluster.
   */
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
  clusterFrontierGrids();

 public:
  /**
   *   @brief  Default constructor for Map.
   *
   *   @param  none
   *
   *   @return void
   */
  Map();

  /**
   *   @brief  Destructor for Map
   *
   *   @param  none
   *
   *   @return void
   */
  ~Map();

  /**
   *   @brief  getter function to obtain the dimesions of the occupancy map i.e
   *           number of cells in height and width of the map
   *
   *   @param  none
   *
   *   @return vector of unsigned integer containing the dimensions of the map
   */
  std::vector<uint32_t> getMapDimensions();

  /**
   *   @brief  getter function to obtain the parameters of the occupancy map i.e
   *           resolution and origin of the map
   *
   *   @param  none
   *
   *   @return pair of float and ROS message type pose denoting resoltion and
   *           origin of map respectively
   */
  std::pair<float, geometry_msgs::Pose> getMapParameters();

  /**
   *   @brief  getter function to obtain the occupancy grid map
   *
   *   @param  none
   *
   *   @return matrix i.e vector of vector of grid cells.
   */
  std::vector<std::vector<Grid>> getOccupancyGrid();

  /**
   *   @brief  function to process occupancy grid map message
   *           published and initialize the map.
   *
   *   @param  map is a boost shared pointer to a message of type
   *           nav_msgs::OccupancyGrid
   *
   *   @return void
   */
  void initialize(const nav_msgs::OccupancyGrid::ConstPtr &map);

  /**
   *   @brief  function to update the occupancy grid map message
   *           published in case dimensions or map parameters change
   *
   *   @param  map is a boost shared pointer to a message of type
   *           nav_msgs::OccupancyGrid
   *
   *   @return void
   */
  void updateOccupancyMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

  /**
   *   @brief  function that obtain the list of clusters of frontier grid cells.
   *
   *   @param  none
   *
   *   @return container of vectors of frontier grid cells as paired unsigned
   *           integers. Each vector denotes a cluster.
   */
  std::vector<std::vector<std::pair<uint32_t, uint32_t>>> getFrontierClusters();

  /**
   *   @brief  function to convert from grid cell coordinates i.e height, width
   *           to cartesian coordinates i.e x,y in the map
   *
   *   @param  vector of paired unsigned integer points where each point denotes
   *           a frontier in grid cell coordinates
   *
   *   @return vector of paired float points where each point denotes a frontier
   *           in cartesian coordinates
   */
  std::vector<std::pair<float, float>> gridToCartesian(
      const std::vector<std::pair<uint32_t, uint32_t>> &frontiers);
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_MAP_HPP_
