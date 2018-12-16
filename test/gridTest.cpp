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
 *  @file    gridTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief   Unit tests for class Grid
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test member functions of class Grid.
 *
 */
// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

/**
 *  @brief Class GridTest
 *
 *  A class to setup and teardown grid class configurations for test fixtures.
 */
class GridTest : public ::testing::Test {
 protected:
  Grid cell;              ///< Object of class grid
  uint32_t height = 256;  ///< height of some desired value
  uint32_t width = 273;   ///< width of some desired value
  int8_t prob = 7;        ///< probability of some desired value
  int num = 91939;        ///< cluster number of some desired value

 public:
  /**
   *   @brief  Setup function to prepare for each test fixture
   *
   *   @param  none
   *
   *   @return void
   */
  void SetUp() {}

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
 *   @brief  Test case to check setting and getting the grid cell state
 *
 *   @param  GridTest - gtest framwork for test fixtures
 *   @param  testSettingGridState - test name
 *
 *   @return void
 */
TEST_F(GridTest, testSettingGridState) {
  cell.setGridState(height, width);
  auto state = cell.getGridState();

  ASSERT_EQ(state.first, height);
  ASSERT_EQ(state.second, width);

  cell.setGridState(111, 222);
  auto updatedState = cell.getGridState();

  ASSERT_EQ(updatedState.first, 111);
  ASSERT_EQ(updatedState.second, 222);
}

/**
 *   @brief  Test case to check updating and obtaining the probability of the
 *           grid cell
 *
 *   @param  GridTest - gtest framwork for test fixtures
 *   @param  testUpdatingGridProbability - test name
 *
 *   @return void
 */
TEST_F(GridTest, testUpdatingGridProbability) {
  cell.updateProbability(prob);
  auto probability = cell.getProbability();

  ASSERT_EQ(probability, prob);

  cell.updateProbability(18);
  auto updatedProbability = cell.getProbability();

  ASSERT_EQ(updatedProbability, 18);
}

/**
 *   @brief  Test case to check setting and obtaining the frontier status of the
 *           grid cell
 *
 *   @param  GridTest - gtest framwork for test fixtures
 *   @param  testSettingGridFrontierStatus - test name
 *
 *   @return void
 */
TEST_F(GridTest, testSettingGridFrontierStatus) {
  cell.setFrontierStatus(true);

  ASSERT_TRUE(cell.getFrontierStatus());

  cell.setFrontierStatus(false);

  ASSERT_FALSE(cell.getFrontierStatus());
}

/**
 *   @brief  Test case to check setting and obtaining the cluster number of the
 *           grid cell
 *
 *   @param  GridTest - gtest framwork for test fixtures
 *   @param  testSettingGridCluster - test name
 *
 *   @return void
 */
TEST_F(GridTest, testSettingGridCluster) {
  cell.setClusterNumber(num);
  auto cluster = cell.getClusterNumber();

  ASSERT_EQ(cluster, num);

  cell.setClusterNumber(7424);
  auto updatedCluster = cell.getClusterNumber();

  ASSERT_EQ(updatedCluster, 7424);
}
