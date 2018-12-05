// GTest Headers
#include <gtest/gtest.h>

// ROS Headers
#include <ros/ros.h>

// Grid Class Header file
#include "et_exploration_robot/grid.hpp"

// CPP Headers
#include <cstdint>
#include <utility>

TEST(GridTest, testSettersAndGetterFunctions) {
  Grid cell;
  uint32_t height = 256;
  uint32_t width = 273;
  int8_t prob = 7;
  int num = 91939;

  cell.setGridState(height, width);
  cell.updateProbability(prob);
  cell.setFrontierStatus(true);
  cell.setClusterNumber(num);

  auto state = cell.getGridState();

  EXPECT_EQ(state.first, height);
  EXPECT_EQ(state.second, width);

  auto probability = cell.getProbability();

  EXPECT_EQ(probability, prob);
  EXPECT_TRUE(cell.getFrontierStatus());

  auto cluster = cell.getClusterNumber();
  EXPECT_EQ(cluster, num);
}
