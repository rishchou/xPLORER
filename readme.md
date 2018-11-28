# xPLORER

[![Build Status](https://travis-ci.org/rishchou/xPLORER.svg?branch=master)](https://travis-ci.org/rishchou/xPLORER)
[![Coverage Status](https://coveralls.io/repos/github/rishchou/xPLORER/badge.svg?branch=master)](https://coveralls.io/github/rishchou/xPLORER?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Project Decription

One of the most popular research problems in robotics now-a-days is to
capture behavior of a robot in an unknown environment. We aim to develop a similar product using the turtlebot
and rgbdslam/octomap packages on ROS to develop a model where the robot can capture characteristics of an
unknown environment as shown in the figure below. The prototype product will be able to traverse unknown
spaces and create a 3D map of the environment in the form of a point cloud or an octree format. This will help the
robots to explore unmanned spaces and difficult terrains like mines etc. and create a map of the environment for
potential hazards or safe navigation. It will also help in creating 3D models of commercial spaces for virtual tours
etc. without any human intervention.

## Authors

Sprint 1:
- Driver - Rishabh Choudhary
- Navigator - Akash Atharv

## Dependencies

The project requires ROS kinetic, catkin, Gazebo and the TurtleBot package and it is developed on UBUNTU 16.04 LTS. 

To install ROS kinetic, please follow the tutorial on: 
http://wiki.ros.org/kinetic/Installation/Ubuntu

Gazebo is normally included with ROS installation

Catkin is normally installed with ROS, If not follow :
http://wiki.ros.org/catkin

To make a catkin workspace: 
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To install turtlebot packages use: 
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
sudo apt-get install ros-kinetic-turtlebot-apps
sudo apt-get install ros-kinetic-turtlebot-rviz-launchers
```
## SIP (Solo Iterative Process)
 
This project was developed following pair programming concepts and SIP. The estimated and completed tasks have been stored in the form of product backlog, Iteration backlog and work log to include the specifics of each task. The product backlog contains the set of all of the tasks to be completed for the given feature implementation. The iteration backlog includes tasks that were repeated over the course of Sprint.

Detailed SIP Enactment with product backlog,iteration backlog and work log can be found [at](https://docs.google.com/spreadsheets/d/1m1UHrcsnNCY8bqwcfVqGJSqxgKsjhua1r7esJMIEKb0/edit?usp=sharing)

Sprint notes can be found [at](https://docs.google.com/document/d/1vihsMah5-x3lxd72F6U38xuexMgioh-XDoyO9bQRQPM/edit?usp=sharing)

## To-do tasks for pair programming (Driver navigator discussion)
- [x] Add class and activity diagram 
- [x] Add defect log and release backlog
- [x] Add LICENSE file
- [x] Update SIP Logs

## License

```
Copyright (c) 2018, Akash Atharv, Rishabh Choudhary
 
Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the 
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
