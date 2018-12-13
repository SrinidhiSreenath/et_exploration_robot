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

class TestPub {
 private:
  geometry_msgs::Twist twist;
  visualization_msgs::MarkerArray markers;

 public:
  TestPub() {}
  ~TestPub() {}

  void velCallback(const geometry_msgs::Twist::ConstPtr &msg) { twist = *msg; }

  void markerCallback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    markers = *msg;
  }
};

class ExplorerTest : public ::testing::Test {
 public:
  ros::NodeHandle nh;
  TestPub test;

  ros::Rate loop_rate = ros::Rate(10);

  void SetUp() {}

  void TearDown() {}
};

TEST_F(ExplorerTest, testVelocityPublisher) {
  Explorer curiosity(nh);

  ros::Subscriber sub = nh.subscribe("/mobile_base/commands/velocity", 50,
                                     &TestPub::velCallback, &test);
  loop_rate.sleep();

  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(ExplorerTest, testMarkerPublisher) {
  Explorer curiosity(nh);

  ros::Subscriber sub = nh.subscribe("/visualization_marker_array", 1,
                                     &TestPub::markerCallback, &test);
  loop_rate.sleep();

  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(ExplorerTest, testMapSubscriber) {
  Explorer curiosity(nh);

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 20);

  loop_rate.sleep();

  EXPECT_EQ(1, pub.getNumSubscribers());
}
