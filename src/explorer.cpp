// ROS Headers
#include <geometry_msgs/Pose.h>
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
  markerPub_ =
      n_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
  mapSub_ = n_.subscribe("/map", 50, &Explorer::processMap, this);
}

Explorer::~Explorer() {}

void Explorer::processMap(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  ROS_INFO_STREAM("Map callback");
  if (!mapInit_) {
    mapInit_ = true;
    myMap.initialize(map);
  } else {
    myMap.updateOccupancyMap(map);
  }
}

void Explorer::revolve() {
  vel_.linear.x = 0.0;
  vel_.angular.z = -0.628319;

  auto current = ros::Time::now();

  ros::Time revolveTime = current + ros::Duration(10.0);

  while (ros::Time::now() < revolveTime && ros::ok()) {
    // ROS_INFO_STREAM("Current time: " << ros::Time::now());
    velocityPub_.publish(vel_);
  }
  vel_.angular.z = 0.0;
  velocityPub_.publish(vel_);
}

void Explorer::determineFrontiers(
    const std::vector<std::vector<std::pair<uint32_t, uint32_t>>>
        &frontierClusters) {
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

  ROS_INFO_STREAM("Starting marker block");

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

  float distance = std::numeric_limits<float>::max();

  auto closestFrontier =
      std::make_pair(frontiersXY[0].first, frontiersXY[0].second);

  for (const auto &frontier : frontiersXY) {
    auto dist = std::hypot(turtleX - frontier.first, turtleY - frontier.second);
    if (dist < distance) {
      distance = dist;
      closestFrontier = std::make_pair(frontier.first, frontier.second);
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
  } else {
    // done with exploration
  }
}
