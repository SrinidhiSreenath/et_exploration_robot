// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

class ExplorerTest : public ::testing::Test {
 public:
  nav_msgs::OccupancyGridPtr = myCustomMap(new nav_msgs::OccupancyGrid);
  geometry_msgs::Pose mapOrigin;
  Explorer curiosity;

  void setup() {
    myCustomMap->info.resolution = 2.5;
    myCustomMap->info.width = 5;
    myCustomMap->info.height = 5;

    mapOrigin.position.x = -4.2;
    mapOrigin.position.y = -6.3;

    myCustomMap->info.origin = mapOrigin;

    myCustomMap->data = {0, 0,  0, 0, 0,   0, 0, 100, 0, 0, -1, 0, 100,
                         0, -1, 0, 0, 100, 0, 0, 0,   0, 0, 0,  0};

    curiosity.processMap(myCustomMap);
    curiosity.determineFrontiers();
  }

  void teardown() {}
};

TEST_F(MapTest, testMap) {
  // get frontier clusters
  auto frontiers = curiosity.getFrontiers();

  auto frontiersxy = myMap.gridToCartesian(frontiers);

  ASSERT_EQ(frontierxy.size(), 2);

  ASSERT_EQ(frontierxy[0].first, -4.2);
  ASSERT_EQ(frontierxy[0].second, -1.3);

  ASSERT_EQ(frontierxy[1].first, 3.3);
  ASSERT_EQ(frontierxy[1].second, -1.3);
}