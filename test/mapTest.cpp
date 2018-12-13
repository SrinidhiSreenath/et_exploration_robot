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

class MapTest : public ::testing::Test {
 public:
  nav_msgs::MapMetaData metaData;
  Map myMap;

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

  void TearDown() {}
};

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

TEST_F(MapTest, testFrontierClusters) {
  // get frontier clusters
  auto clusters = myMap.getFrontierClusters();

  ASSERT_EQ(clusters.size(), 2);
  ASSERT_EQ(clusters[0].size(), 5);
  ASSERT_EQ(clusters[1].size(), 5);
}

TEST_F(MapTest, testFrontierClustersCentroid) {
  auto clusters = myMap.getFrontierClusters();

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
}

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
