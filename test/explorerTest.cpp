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
 *  @file    explorerTest.cpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief   Rostests for class Explorer
 *
 *  @section DESCRIPTION
 *
 *  Test cases to test subscribers and publishers in class Explorer
 *
 */
// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/explorer.hpp"
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

/**
 *  @brief Class TestPub
 *
 *  A class to hold dummy callback functions for subscriber callback
 */
class TestPub {
 private:
  geometry_msgs::Twist twist;               ///< ROS message to hold velocities
  visualization_msgs::MarkerArray markers;  ///< ROS message to hold markerarray

 public:
  /**
   *   @brief  Default constructor for TestPub.
   *
   *   @param  none
   *
   *   @return void
   */
  TestPub() {}

  /**
   *   @brief  Default destructor for TestPub.
   *
   *   @param  none
   *
   *   @return void
   */
  ~TestPub() {}

  /**
   *   @brief  dummy callback function for subscriber to velocity topic
   *
   *   @param  msg is a boost shared pointer to a message of type
   *           geometry_msgs::Twist
   *
   *   @return void
   */
  void velCallback(const geometry_msgs::Twist::ConstPtr &msg) { twist = *msg; }

  /**
   *   @brief  dummy callback function for subscriber to markerarray topic
   *
   *   @param  msg is a boost shared pointer to a message of type
   *           visualization_msgs::MarkerArray
   *
   *   @return void
   */
  void markerCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    markers = *msg;
  }
};

/**
 *  @brief Class ExplorerTest
 *
 *  A class to setup and teardown explorer class configurations for test
 * fixtures.
 */
class ExplorerTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh;  ///< ROS NodeHandle object
  TestPub test;        ///< TestPub object for dummy callbacks

  ros::Rate loop_rate = ros::Rate(10);  ///< ROS loop rate

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
 *   @brief  Test case to check if the velocity publisher is set up with ROS
 *           Master when the explorer class is initialized.
 *
 *   @param  ExplorerTest - gtest framwork for test fixtures
 *   @param  testVelocityPublisher - test name
 *
 *   @return void
 */
TEST_F(ExplorerTest, testVelocityPublisher) {
  Explorer curiosity(nh);

  // subscribe to topic where velocity is published to
  ros::Subscriber sub = nh.subscribe("/mobile_base/commands/velocity", 50,
                                     &TestPub::velCallback, &test);
  loop_rate.sleep();

  // Number of publishers to this should be 1
  EXPECT_EQ(1, sub.getNumPublishers());
}

/**
 *   @brief  Test case to check if the markerarray publisher is set up with ROS
 *           Master when the explorer class is initialized.
 *
 *   @param  ExplorerTest - gtest framwork for test fixtures
 *   @param  testMarkerPublisher - test name
 *
 *   @return void
 */
TEST_F(ExplorerTest, testMarkerPublisher) {
  Explorer curiosity(nh);

  // subscribe to topic where markerarray is published to
  ros::Subscriber sub = nh.subscribe("/visualization_marker_array", 1,
                                     &TestPub::markerCallback, &test);
  loop_rate.sleep();

  // Number of publishers to this should be 1
  EXPECT_EQ(1, sub.getNumPublishers());
}

/**
 *   @brief  Test case to check if the map subcriber is set up when the
 *           explorer class is initialized.
 *
 *   @param  ExplorerTest - gtest framwork for test fixtures
 *   @param  testMapSubscriber - test name
 *
 *   @return void
 */
TEST_F(ExplorerTest, testMapSubscriber) {
  Explorer curiosity(nh);

  // publish to topic map a dummy occupancygrid map message
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 20);

  loop_rate.sleep();

  // Number of subscribers to this topic should be 1 which is setup when
  // explorer is initialized
  EXPECT_EQ(1, pub.getNumSubscribers());
}
