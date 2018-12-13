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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class Explorer {
 private:
  ros::NodeHandle n_;
  ros::Subscriber mapSub_;
  ros::Publisher velocityPub_;
  ros::Publisher markerPub_;
  geometry_msgs::Twist vel_;
  tf::TransformListener poseListener_;

  Map myMap;
  bool mapInit_ = false;
  std::vector<std::pair<uint32_t, uint32_t>> frontiers_;

  std::vector<std::pair<float, float>> notReachablefrontiers_;

  uint32_t shape = visualization_msgs::Marker::CUBE;

  void processMap(const nav_msgs::OccupancyGrid::ConstPtr &map);

  void revolve();

  void determineFrontiers(
      const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
          &frontierClusters);

  void visualizeFrontiers(
      const std::vector<std::pair<float, float>> &frontiersXY);

  bool isDiscardedFrontier(const std::pair<float, float> &frontier);

  std::pair<float, float> closestFrontier(
      const std::vector<std::pair<float, float>> &frontiersXY);

 public:
  explicit Explorer(ros::NodeHandle nh);

  ~Explorer();

  void explore();
};

#endif  //  INCLUDE_ET_EXPLORATION_ROBOT_EXPLORER_HPP_
