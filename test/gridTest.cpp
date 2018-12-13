// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// CPP Headers
#include <cstdint>
#include <utility>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

class GridTest : public ::testing::Test {
 public:
  Grid cell;
  uint32_t height = 256;
  uint32_t width = 273;
  int8_t prob = 7;
  int num = 91939;

  void SetUp() {}

  void TearDown() {}
};

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

TEST_F(GridTest, testUpdatingGridProbability) {
  cell.updateProbability(prob);
  auto probability = cell.getProbability();

  ASSERT_EQ(probability, prob);

  cell.updateProbability(18);
  auto updatedProbability = cell.getProbability();

  ASSERT_EQ(updatedProbability, 18);
}

TEST_F(GridTest, testSettingGridFrontierStatus) {
  cell.setFrontierStatus(true);

  ASSERT_TRUE(cell.getFrontierStatus());

  cell.setFrontierStatus(false);

  ASSERT_FALSE(cell.getFrontierStatus());
}

TEST_F(GridTest, testSettingGridCluster) {
  cell.setClusterNumber(num);
  auto cluster = cell.getClusterNumber();

  ASSERT_EQ(cluster, num);

  cell.setClusterNumber(7424);
  auto updatedCluster = cell.getClusterNumber();

  ASSERT_EQ(updatedCluster, 7424);
}
