// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// Class header files
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

// CPP Headers
#include <cstdint>
#include <utility>

class MapTest : public ::testing::Test {
 public:
  nav_msgs::OccupancyGridPtr = myCustomMap(new nav_msgs::OccupancyGrid);
  geometry_msgs::Pose mapOrigin;
  Map myMap;

  void setup() {
    myCustomMap->info.resolution = 2.5;
    myCustomMap->info.width = 5;
    myCustomMap->info.height = 5;

    mapOrigin.position.x = -4.2;
    mapOrigin.position.y = -6.3;

    myCustomMap->info.origin = mapOrigin;

    myCustomMap->data = {0, 0,  0, 0, 0,   0, 0, 100, 0, 0, -1, 0, 100,
                         0, -1, 0, 0, 100, 0, 0, 0,   0, 0, 0,  0};

    myMap.initialize(myCustomMap);
  }

  void teardown() {}
};

TEST_F(MapTest, testMap) {
  // get frontier clusters
  auto clusters = myMap.getFrontierClusters();

  ASSERT_EQ(clusters.size(), 2);
  ASSERT_EQ(clusters[0].size(), 5);  // check
  ASSERT_EQ(clusters[1].size(), 5);

  uint32_t frontierHeight1, frontierWidth1, frontierHeight2, frontierWidth2;

  size_t i = 0;

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

  auto first = std::make_pair(frontierHeight1, frontierWidth1);
  auto second = std::make_pair(frontierHeight2, frontierWidth2);

  auto frontiers = {first, second};

  auto frontiersxy = myMap.gridToCartesian(frontiers);

  ASSERT_EQ(frontierxy.size(), 2);

  ASSERT_EQ(frontierxy[0].first, -4.2);
  ASSERT_EQ(frontierxy[0].second, -1.3);

  ASSERT_EQ(frontierxy[1].first, 3.3);
  ASSERT_EQ(frontierxy[1].second, -1.3);
}
