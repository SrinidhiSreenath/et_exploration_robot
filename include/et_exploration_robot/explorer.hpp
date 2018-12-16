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
 *  @file    explorer.hpp
 *  @author  Srinidhi Sreenath (SrinidhiSreenath)
 *  @date    12/15/2018
 *  @version 1.0
 *
 *  @brief Explorer class declaration
 *
 *  @section DESCRIPTION
 *
 *  Header file for class Explorer which implements the exploration behavior for
 *  a turtlebot in an unknown environment.
 *
 */

#ifndef INCLUDE_ET_EXPLORATION_ROBOT_EXPLORER_HPP_
#define INCLUDE_ET_EXPLORATION_ROBOT_EXPLORER_HPP_

// ROS Headers
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

// Move base action client for navigation
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

/**
 *  @brief Class Explorer
 *
 *  The following class Explorer implements the exploration behavior by
 *  determining the frontiers in an unexplored environment and sending
 *  navigation goals for a turtlebot. Class Map is a part of this class.
 */
class Explorer {
 private:
  ros::NodeHandle n_;  ///< ROS node handle object
  ros::Subscriber
      mapSub_;  ///< ROS subscriber for subscribing to occupancy grid message
  ros::Publisher velocityPub_;  ///< ROS publisher for publishing velocity for
                                ///< turtlebot movement
  ros::Publisher
      markerPub_;  ///< ROS publisher for publishing an array of markers in Rviz
                   ///< for visualising the frontier points
  geometry_msgs::Twist
      vel_;  ///< ROS standard message type used for publshing velocities
  tf::TransformListener
      poseListener_;  ///< ROS tf transform listener for obtaining the current
                      ///< pose of the turtlebot

  Map myMap;              ///< Map class object to hold environment information
  bool mapInit_ = false;  ///< flag to check if environment has been initialized

  std::vector<std::pair<uint32_t, uint32_t>>
      frontiers_;  ///< container to hold the set of frontiers obtained from the
                   ///< map

  std::vector<std::pair<float, float>>
      notReachablefrontiers_;  ///< Container to hold the list of frontiers that
                               ///< are not succesfully reachable by the
                               ///< turtlebot

  uint32_t shape =
      visualization_msgs::Marker::CUBE;  ///< shape of the visualization marker
                                         ///< denoting the frontier in rviz
                                         ///< simulation

  /**
   *   @brief  callback function to process occupancy grid map message
   *           published. The map is initialised the first time and then
   *           updated subsequently.
   *
   *   @param  map is a boost shared pointer to a message of type
   *           nav_msgs::OccupancyGrid
   *
   *   @return void
   */
  void processMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

  /**
   *   @brief  function to publish velocity commands to a turtlebot to make it
   *           rotate 360 degrees about its position.
   *
   *   @param  none
   *
   *   @return void
   */
  void revolve();

  /**
   *   @brief  function to update the frontiers i.e centroids of given set of
   *           frontier clusters.
   *
   *   @param  frontierClusters is a container of vectors of paired unsigned
   *           integer points where each point is a frontier point and the
   *           vector of points denote a cluster. The contianer hold a list of
   *           clusters.
   *
   *   @return void
   */
  void determineFrontiers(
      const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
          &frontierClusters);

  /**
   *   @brief  funtion to publish markers to visualize frontiers in rviz
   *
   *   @param  frontiersXY is a vector of paired float points denoting the x and
   *           y coordinate of each frontier in the map.
   *
   *   @return void
   */
  void visualizeFrontiers(
      const std::vector<std::pair<float, float>> &frontiersXY);

  /**
   *   @brief  funtion to determine whether frontier is discarded i.e a frontier
   *           which the robot previously failed to navigate to.
   *
   *   @param  frontier is a pair of float points denoting x and y coordinate of
   *           the frontier.
   *
   *   @return bool type. True if the frontier is discarded and false otherwise.
   */
  bool isDiscardedFrontier(const std::pair<float, float> &frontier);

  /**
   *   @brief  funtion to determine the closest frontier to the turtlebot's
   *           current pose among the available frontiers
   *
   *   @param  frontiersXY is a vector of paired float points denoting the x and
   *           y coordinate of each frontier in the map.
   *
   *   @return a pair of float points denoting x and y coordinates of the
   *           closest frontier to the turtlebot
   */
  std::pair<float, float> closestFrontier(
      const std::vector<std::pair<float, float>> &frontiersXY);

 public:
  /**
   *   @brief  Default constructor for Explorer. Sets up the velocity and marker
   *           array publisher topic with ROS master and subscribes to occupancy
   *           grid map data.
   *
   *   @param  nh as ROS nodehandle object
   *
   *   @return void
   */
  explicit Explorer(const ros::NodeHandle &nh);

  /**
   *   @brief  Destructor for Explorer
   *
   *   @param  none
   *
   *   @return void
   */
  ~Explorer();

  /**
   *   @brief  wrapper funtion to intiate and perform all the necessary actions
   *           needed for the exploration of the unknown environment by
   *           turtlebot
   *
   *   @param  none
   *
   *   @return void
   */
  void explore();
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_EXPLORER_HPP_
