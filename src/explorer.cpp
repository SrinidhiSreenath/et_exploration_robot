// ROS Headers
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// CPP Headers
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

// Class header files
#include "et_exploration_robot/explorer.hpp"
#include "et_exploration_robot/grid.hpp"
#include "et_exploration_robot/map.hpp"

Explorer::Explorer(ros::NodeHandle nh) : n_(nh) {
  velocityPub_ =
      n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 50);
  mapSub_ = n_.subscribe("/map", 20, &Explorer::processMap, this);
  markerPub_ = n_.advertise<visualization_msgs::MarkerArray>(
      "/visualization_marker_array", 1);
}

Explorer::~Explorer() {}

void Explorer::processMap(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  if (!mapInit_) {
    mapInit_ = true;
    myMap.initialize(map);
  } else {
    myMap.updateOccupancyMap(map);
  }
}

void Explorer::revolve() {
  vel_.linear.x = 0.0;
  vel_.linear.y = 0.0;
  vel_.linear.z = 0.0;
  vel_.angular.x = 0.0;
  vel_.angular.y = 0.0;
  vel_.angular.z = -0.628319;

  ros::Rate rate(10);

  auto current = ros::Time::now();

  ros::Duration waitTime = ros::Duration(10.0);
  ros::Time revolveTime = current + waitTime;

  while (ros::Time::now() < revolveTime) {
    velocityPub_.publish(vel_);
    rate.sleep();
  }

  vel_.linear.x = 0.0;
  vel_.linear.y = 0.0;
  vel_.linear.z = 0.0;
  vel_.angular.x = 0.0;
  vel_.angular.y = 0.0;
  vel_.angular.z = 0.0;
  velocityPub_.publish(vel_);
}

void Explorer::determineFrontiers(
    const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
        &frontierClusters) {
  // ROS_INFO_STREAM("Total clusters: " << frontierClusters.size());
  // Calculate centroid of each cluster
  for (auto cluster : frontierClusters) {
    if (cluster.size() > 20) {
      uint32_t x = 0;
      uint32_t y = 0;
      for (auto frontierPoint : cluster) {
        x += frontierPoint.first;
        y += frontierPoint.second;
      }
      x /= cluster.size();
      y /= cluster.size();

      auto frontier = std::make_pair(x, y);
      frontiers_.push_back(frontier);
    }
  }
}

void Explorer::visualizeFrontiers(
    const std::vector<std::pair<float, float>> &frontiersXY) {
  // Visualize all frontiers
  visualization_msgs::MarkerArray frontiersToViz;
  frontiersToViz.markers.resize(frontiers_.size());

  ROS_INFO_STREAM("Number of Frontiers: " << frontiersXY.size());

  for (size_t it = 0; it < frontiersXY.size(); it++) {
    // Set the frame ID and timestamp.  See the TF tutorials for information
    // on these.
    frontiersToViz.markers[it].header.frame_id = "/map";
    frontiersToViz.markers[it].header.stamp = ros::Time::now();

    // Set the namespace and id for this frontiersToViz.markers[it].  This
    // serves to create a unique ID Any marker sent with the same namespace
    // and id will overwrite the old one
    frontiersToViz.markers[it].ns = "basic_shapes";
    frontiersToViz.markers[it].id = it;

    // Set the marker type.  Initially this is CUBE, and cycles between that
    // and SPHERE, ARROW, and CYLINDER
    frontiersToViz.markers[it].type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS
    // Indigo: 3 (DELETEALL)
    frontiersToViz.markers[it].action = visualization_msgs::Marker::ADD;

    // Set the pose of the frontiersToViz.markers[it].  This is a full 6DOF
    // pose relative to the frame/time specified in the header
    frontiersToViz.markers[it].pose.position.x = frontiersXY[it].first;
    frontiersToViz.markers[it].pose.position.y = frontiersXY[it].second;
    frontiersToViz.markers[it].pose.position.z = 0;
    frontiersToViz.markers[it].pose.orientation.x = 0.0;
    frontiersToViz.markers[it].pose.orientation.y = 0.0;
    frontiersToViz.markers[it].pose.orientation.z = 0.0;
    frontiersToViz.markers[it].pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    frontiersToViz.markers[it].scale.x = 0.1;
    frontiersToViz.markers[it].scale.y = 0.1;
    frontiersToViz.markers[it].scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    frontiersToViz.markers[it].color.r = 0.0f;
    frontiersToViz.markers[it].color.g = 0.0f;
    frontiersToViz.markers[it].color.b = 1.0f;
    frontiersToViz.markers[it].color.a = 1.0;

    frontiersToViz.markers[it].lifetime = ros::Duration();
  }
  markerPub_.publish(frontiersToViz);
}

bool Explorer::isDiscardedFrontier(const std::pair<float, float> &frontier) {
  for (const auto &discardedFrontier : notReachablefrontiers_) {
    if (frontier == discardedFrontier) {
      return true;
    }
  }
  return false;
}

std::pair<float, float> Explorer::closestFrontier(
    const std::vector<std::pair<float, float>> &frontiersXY) {
  tf::StampedTransform transform;
  try {
    poseListener_.lookupTransform("/map", "/base_link", ros::Time(0),
                                  transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  auto turtleX = transform.getOrigin().x();
  auto turtleY = transform.getOrigin().y();

  auto closestFrontier =
      std::make_pair(frontiersXY[0].first, frontiersXY[0].second);

  if (frontiersXY.size() == 1) {
    return closestFrontier;
  }

  float distance = std::numeric_limits<float>::max();
  for (const auto &frontier : frontiersXY) {
    if (!isDiscardedFrontier(frontier)) {
      auto dist =
          std::hypot(turtleX - frontier.first, turtleY - frontier.second);
      if (dist < distance && dist > 1) {
        distance = dist;
        closestFrontier = std::make_pair(frontier.first, frontier.second);
      }
    }
  }
  return closestFrontier;
}

void Explorer::explore() {
  // rotate 360 in current place.
  revolve();

  // clear exisiting frontiers
  frontiers_.clear();

  // get frontier clusters from map
  auto frontierClusters = myMap.getFrontierClusters();

  // calculate frontier from clusters
  determineFrontiers(frontierClusters);

  // if frontiers exist, visualise them and calcute the closest frontier to
  // current position of the robot.
  if (frontiers_.size() != 0) {
    // Obtain frontiers in (X,Y) position
    auto frontiersXY = myMap.gridToCartesian(frontiers_);

    // visualize the frontiers
    visualizeFrontiers(frontiersXY);

    // get the closest frontier
    auto frontierToNavigate = closestFrontier(frontiersXY);

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = frontierToNavigate.first;
    goal.target_pose.pose.position.y = frontierToNavigate.second;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  } else {
    // done with exploration
  }
}
