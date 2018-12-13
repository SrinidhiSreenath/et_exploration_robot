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

  nav_msgs::OccupancyGridPtr myUpdatedMap(new nav_msgs::OccupancyGrid);

  metaData.origin.position.x = 5.6;
  metaData.origin.position.y = 8.9;
  metaData.origin.orientation.w = 1;

  metaData.resolution = 7.5;
  metaData.width = 7;
  metaData.height = 8;

  myUpdatedMap->info = metaData;

  myUpdatedMap->data = {0, 0,  0, 0, 0,   0, 0, 100, 0, 0, -1, 0, 100,
                        0, -1, 0, 0, 100, 0, 0, 0,   0, 0, 0,  0};

  myMap.updateOccupancyMap(myUpdatedMap);

  auto updatedDimensions = myMap.getMapDimensions();
  std::vector<uint32_t> updatedDim = {8, 7};

  ASSERT_EQ(updatedDimensions, updatedDim);

  auto updatedParam = myMap.getMapParameters();

  ASSERT_EQ(updatedParam.first, 7.5);
  ASSERT_EQ(updatedParam.second.position.x, 5.6);
  ASSERT_EQ(updatedParam.second.position.y, 8.9);
}
