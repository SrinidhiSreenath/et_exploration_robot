#include <ros/ros.h>
// Class header files
#include "et_exploration_robot/explorer.hpp"
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "frontier_exploration");

  ros::NodeHandle nh;
  Explorer curiosity(nh);

  ros::Rate loop_rate(10);

  ros::spinOnce();
  ros::Duration(10).sleep();

  while (ros::ok()) {
    ros::spinOnce();
    curiosity.explore();
    loop_rate.sleep();
  }

  return 0;
}
