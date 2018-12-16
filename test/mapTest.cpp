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
 *  @file    mapTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief   Unit tests for class Map
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test member functions of class Map.
 *
 */
// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

/**
 *  @brief Class MapTest
 *
 *  A class to setup and teardown map class configurations for test fixtures.
 */
class MapTest : public ::testing::Test {
 protected:
  nav_msgs::MapMetaData
      metaData;  ///< ROS message of type nav_msgs::MapMetaData to define a
                 ///< custom map
  Map myMap;     ///< Object of class map

 public:
  /**
   *   @brief  Setup function to prepare for each test fixture. Sets a custom
   *           map and initializes the map
   *
   *   @param  none
   *
   *   @return void
   */
  void SetUp() {
    nav_msgs::OccupancyGridPtr myCustomMap(new nav_msgs::OccupancyGrid);

    metaData.origin.position.x = -4.2;
    metaData.origin.position.y = -6.3;
    metaData.origin.orientation.w = 1;

    metaData.resolution = 2.5;
    metaData.width = 5;
    metaData.height = 5;

    myCustomMap->info = metaData;

    myCustomMap->data = {0, 0,  0, 0, 0,   0, 0, 100, 0, 0, -1, 0, 100,
                         0, -1, 0, 0, 100, 0, 0, 0,   0, 0, 0,  0};

    myMap.initialize(myCustomMap);
  }

  /**
   *   @brief  Tesrdown function to release any resources allocated in SetUp
   *
   *   @param  none
   *
   *   @return void
   */
  void TearDown() {}
};

/**
 *   @brief  Test case to check initialization and updation of occupancy map
 *           dimensions and parameters
 *
 *   @param  MapTest - gtest framwork for test fixtures
 *   @param  testInitializationAndUpdationOfMap - test name
 *
 *   @return void
 */
TEST_F(MapTest, testInitializationAndUpdationOfMap) {
  auto dimensions = myMap.getMapDimensions();
  std::vector<uint32_t> dim = {5, 5};

  ASSERT_EQ(dimensions, dim);

  auto param = myMap.getMapParameters();

  ASSERT_EQ(param.first, 2.5);
  ASSERT_EQ(param.second.position.x, -4.2);
  ASSERT_EQ(param.second.position.y, -6.3);

  auto occpancyGrid = myMap.getOccupancyGrid();
  auto prob = occpancyGrid[2][2].getProbability();
  auto prob1 = occpancyGrid[2][0].getProbability();
  auto prob2 = occpancyGrid[2][4].getProbability();

  ASSERT_EQ(prob, 100);
  ASSERT_EQ(prob1, prob2);

  nav_msgs::OccupancyGridPtr myUpdatedMap(new nav_msgs::OccupancyGrid);

  metaData.origin.position.x = 5.6;
  metaData.origin.position.y = 8.9;
  metaData.origin.orientation.w = 1;

  metaData.resolution = 7.5;
  metaData.width = 4;
  metaData.height = 3;

  myUpdatedMap->info = metaData;

  myUpdatedMap->data = {0, 0, 0, 0, 0, 0, 100, 0, -1, 0, 100, 0};

  myMap.updateOccupancyMap(myUpdatedMap);

  auto updatedDimensions = myMap.getMapDimensions();
  std::vector<uint32_t> updatedDim = {3, 4};

  ASSERT_EQ(updatedDimensions, updatedDim);

  auto updatedParam = myMap.getMapParameters();

  ASSERT_EQ(updatedParam.first, 7.5);
  ASSERT_EQ(updatedParam.second.position.x, 5.6);
  ASSERT_EQ(updatedParam.second.position.y, 8.9);
}

/**
 *   @brief  Test case to check clustering of frontier cells
 *
 *   @param  MapTest - gtest framwork for test fixtures
 *   @param  testFrontierClusters - test name
 *
 *   @return void
 */
TEST_F(MapTest, testFrontierClusters) {
  // get frontier clusters
  auto clusters = myMap.getFrontierClusters();

  ASSERT_EQ(clusters.size(), 2);
  ASSERT_EQ(clusters[0].size(), 5);
  ASSERT_EQ(clusters[1].size(), 5);
}

/**
 *   @brief  Test case to validate centroid of each frontier cluster
 *
 *   @param  MapTest - gtest framwork for test fixtures
 *   @param  testFrontierClustersCentroid - test name
 *
 *   @return void
 */
TEST_F(MapTest, testFrontierClustersCentroid) {
  auto clusters = myMap.getFrontierClusters();

  uint32_t frontierHeight1, frontierWidth1, frontierHeight2, frontierWidth2;

  size_t i = 0;

  // Calculate centroid for each cluster
  for (auto cluster : clusters) {
    uint32_t h = 0;
    uint32_t w = 0;
    for (auto clusterPoint : cluster) {
      h += clusterPoint.first;
      w += clusterPoint.second;
    }
    i++;
    if (i == 1) {
      frontierHeight1 = h;
      frontierWidth1 = w;
    } else {
      frontierHeight2 = h;
      frontierWidth2 = w;
    }
  }

  frontierHeight1 /= clusters[0].size();
  frontierWidth1 /= clusters[0].size();

  frontierHeight2 /= clusters[1].size();
  frontierWidth2 /= clusters[1].size();

  ASSERT_EQ(frontierHeight1, 2);
  ASSERT_EQ(frontierWidth1, 0);

  ASSERT_EQ(frontierHeight2, 2);
  ASSERT_EQ(frontierWidth2, 3);
}

/**
 *   @brief  Test case to check conversion from grid coordinates to cartesian
 *           coordinates
 *
 *   @param  MapTest - gtest framwork for test fixtures
 *   @param  testGridToCartesianConversion - test name
 *
 *   @return void
 */
TEST_F(MapTest, testGridToCartesianConversion) {
  std::pair<uint32_t, uint32_t> first = std::make_pair(2, 0);
  std::pair<uint32_t, uint32_t> second = std::make_pair(2, 3);

  auto frontiers = {first, second};

  auto frontiersxy = myMap.gridToCartesian(frontiers);

  ASSERT_EQ(frontiersxy.size(), 2);

  ASSERT_NEAR(frontiersxy[0].first, -4.2, 0.1);
  ASSERT_NEAR(frontiersxy[0].second, -1.3, 0.1);

  ASSERT_NEAR(frontiersxy[1].first, 3.3, 0.1);
  ASSERT_NEAR(frontiersxy[1].second, -1.3, 0.1);
}
