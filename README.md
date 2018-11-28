# E.T. A frontier exploration robot to explore unknown environments
![Build Status](https://travis-ci.com/SrinidhiSreenath/ET_Exploration_Robot.svg?branch=master) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
This project aims to develop the software stack using agile software development process to demonstrate the simulation of a TurtleBot that localizes, maps and autonomously explores the environment when introduced in an unknown environment.

Robotic exploration is a problem that deals with maximizing the information of an area of interestusing robots.  Exploration using robots is advantageous when humans cannot explore an environment due to inaccessibility or dangerous environmental conditions.  One of the booming areas of exploration is space exploration where robotic systems are used to explore extraterrestrial bodies.

Currently, most of the extraterrestrial exploration robots are maintained by national space agencies, but with easier and cheaper access to extraterrestrial space, the demand for robots that explore extraterrestrial terrains is going to increase and create a potential market avenue. By developing exploration robots that map unknown environment, one can cater to industries interested in logging space maps and space mining. 

The repository is a ROS package implementing a simple exploration with a TurtleBot 2 using frontier based exploration method where it uses frontiers as guidance to explore the unknown space. Frontier based exploration is a technique where frontiers are determined and robot exploration is driven by these frontiers.  Frontiers are points on the boundary region between open explored space and unexplored space.

## Dependencies
This is a ROS package which needs 
- [ROS Kinetic](http://wiki.ros.org/kinetic) to be installed on Ubuntu 16.04. Installation instructions are outlined [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- [Turtlebot](https://www.turtlebot.com/) packages are required. Run the following command to install all turtlebot related packages.
```
sudo apt-get install ros-kinetic-turtlebot*
```
- [Gazebo](http://gazebosim.org/) version 7.0.0 or above. Installation instructions can be found [here](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1).

*package dependencies to be updated*

## Solo Iterative Process (SIP)
Solo Iterative Process (SIP) is used in the development of the project. Test Driven Development appoach is used to comply with the short development cycle. The planning and development of the project is done in three sprints. 

[Product backlog, Iteration backlogs, Work log and Sprint Schedule](https://docs.google.com/spreadsheets/d/1y6k_Kw1-uYTfiacjPWWJsFmW3S48nC0fhaB75R_D93A/edit?usp=sharing).

[Sprint Planning Notes](https://docs.google.com/document/d/1q5BGRm5D0xjOvHy-o9cROjHJibJuXT3Z7A8dWZFaC8w/edit?usp=sharing)

## Operation
*To be updated*

## Run
*To be updated*

## Test
*To be updated*

## Demo
*To be updated*

## Known Issues/ Bugs
*To be updated*

## API and other developer documentation
*To be updated*
