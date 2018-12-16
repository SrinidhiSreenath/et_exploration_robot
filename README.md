# E.T. A frontier exploration robot to explore unknown environments
[![Build Status](https://travis-ci.com/SrinidhiSreenath/et_exploration_robot.svg?branch=master)](https://travis-ci.com/SrinidhiSreenath/et_exploration_robot) [![Coverage Status](https://coveralls.io/repos/github/SrinidhiSreenath/et_exploration_robot/badge.svg?branch=master)](https://coveralls.io/github/SrinidhiSreenath/et_exploration_robot?branch=master) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project aims to develop the software stack using agile software development process to demonstrate the simulation of a TurtleBot that localizes, maps and autonomously explores the environment when introduced in an unknown environment.

Robotic exploration is a problem that deals with maximizing the information of an area of interest using robots.  Exploration using robots is advantageous when humans cannot explore an environment due to inaccessibility or dangerous environmental conditions.  One of the booming areas of exploration is space exploration where robotic systems are used to explore extraterrestrial bodies.

Currently, most of the extraterrestrial exploration robots are maintained by national space agencies, but with easier and cheaper access to extraterrestrial space, the demand for robots that explore extraterrestrial terrains is going to increase and create a potential market avenue. By developing exploration robots that map unknown environment, one can cater to industries interested in logging space maps and space mining. 

The repository is a ROS package implementing a simple exploration with a TurtleBot 2 using frontier based exploration method where it uses frontiers as guidance to explore the unknown space. Frontier based exploration is a technique where frontiers are determined and robot exploration is driven by these frontiers.  Frontiers are points on the boundary region between open explored space and unexplored space.

## Personnel
[Srinidhi Sreenath](https://www.linkedin.com/in/srinidhisreenath/). I am a Mechanical engineer currently pursuing Masters in Robotics at the University of Maryland. My areas of interests include Motion Planning and Decision Making for Self Driving Vehicles.   

## Dependencies
This is a ROS package which needs 
- [ROS Kinetic](http://wiki.ros.org/kinetic) to be installed on Ubuntu 16.04. Installation instructions are outlined [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [Turtlebot](https://www.turtlebot.com/) packages are required. Run the following command to install all turtlebot related packages.
```
sudo apt-get install ros-kinetic-turtlebot*
```
- [Gazebo](http://gazebosim.org/) version 7.0.0 or above. Installation instructions can be found [here](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).
- TurtleBot [rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers) and [move_base](http://wiki.ros.org/move_base) packages are required. Run the following command to install them.
```
sudo apt install ros-kinetic-turtlebot-rviz-launchers ros-kinetic-move-base-msgs ros-kinetic-actionlib ros-kinetic-actionlib-msgs
```

### Package dependences
The following are the package dependies:
- geometry_msgs
- nav_msgs
- roscpp
- rospy
- sensor_msgs
- std_msgs
- visualization_msgs
- tf
- move_base
- move_base_msgs
- actionlib

## Build
In your desired directory, please run the following commands.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/SrinidhiSreenath/et_exploration_robot.git
cd ..
catkin_make
```
## Run
The launch file in the package needs to be launched for simulation. Please run the following commands to launch the desired nodes:
```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch et_exploration_robot explore.launch
```
## Tests
To run the goolgle unit tests and rostest integration tests, run the following command in the catkin workspace:
```
catkin_make run_tests_et_exploration_robot
```
To run the unit tests using the launch file, run the following commands in the catkin workspace after all the packages are succesfully built.
```
cd <path to catkin_ws>
source devel/setup.bash
rostest et_exploration_robot explorerTests.launch 
```
## Results
The top view of the environment is shown below:
![](https://github.com/SrinidhiSreenath/et_exploration_robot/blob/master/images/gazebo.png)

The map generated after exploration is shown below:
![](https://github.com/SrinidhiSreenath/et_exploration_robot/blob/master/images/exploration%20map.png)

## Demo
*To be updated*

## Solo Iterative Process (SIP)
Solo Iterative Process (SIP) is used in the development of the project. Test Driven Development appoach is used to comply with the short development cycle. The planning and development of the project is done in three sprints. 

[Product backlog, Iteration backlogs, Work log and Sprint Schedule](https://docs.google.com/spreadsheets/d/1y6k_Kw1-uYTfiacjPWWJsFmW3S48nC0fhaB75R_D93A/edit?usp=sharing).

[Sprint Planning Notes](https://docs.google.com/document/d/1q5BGRm5D0xjOvHy-o9cROjHJibJuXT3Z7A8dWZFaC8w/edit?usp=sharing)

## Known Issues/ Bugs
- The [move_base](http://wiki.ros.org/move_base) package is used for send frontiers as goals for the turtlebot to navigate. However, there is a lot of odometry error correction and hence the robot constantly re-orients its pose in the environment causing the map of the exploration to distort. A better localization and navigation package can be used to improve the accuracy of the exploration map being built.

- The class Explorer is a wrapper class and most of its members are private. Hence, unit tests and rostest integration tests were difficult to develop. Need to develop better tests for unit testing the Explorer class and need to develop more integration tests to increase coverage.

## Doxygen Documentation
Doxygen Documentation generation steps:
```
cd <path to package root>
mkdir Doxygen
cd Doxygen
doxygen -g <config_file_name>
```
Open configuration file and update the following:
```
PROJECT_NAME = 'et_exploration_robot'
INPUT = ../src ../include/et_exploration_robot/ ../test
```
Run and generate the documents by executing the following:
```
doxygen <config_file_name>
```

## Coverage
Install lcov
```
sudo apt-get install lcov
```
To check for coverage, execute the following commands.
```
cd ~/catkin_ws/build
lcov --directory . --capture --output-file coverage.info
lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info
lcov --list coverage.info
```
The results are shown below:
