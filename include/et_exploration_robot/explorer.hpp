// ROS Headers
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

class Explorer {
 private:
  ros::NodeHandle n_;
  ros::Subscriber mapSub_;
  ros::Publisher velocityPub_;
  ros::Publisher markerPub_;
  geometry_msgs::Twist msg;

  Map myMap;
  bool mapInit_ = false;
  std::vector<std::pair<uint32_t, uint32_t>> frontiers;

  uint32_t shape = visualization_msgs::Marker::CUBE;

  void revolve();

  void determineFrontiers();

 public:
  explicit Explorer(ros::NodeHandle nh);

  ~Explorer();

  void processMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);

  void explore();
};
