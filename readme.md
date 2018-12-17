# xPLORER

[![Build Status](https://travis-ci.org/rishchou/xPLORER.svg?branch=master)](https://travis-ci.org/rishchou/xPLORER)
[![Coverage Status](https://coveralls.io/repos/github/rishchou/xPLORER/badge.svg?branch=master)](https://coveralls.io/github/rishchou/xPLORER)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

## Project Overview

One of the most popular research problems in robotics now-a-days is to
capture behavior of a robot in an unknown environment. We aim to develop a similar product using the turtlebot
and rgbdslam/octomap packages on ROS to develop a model where the robot can capture characteristics of an
unknown environment as shown in the figure below. The prototype product will be able to traverse unknown
spaces and create a 3D map of the environment in the form of a point cloud or an octree format. This will help the
robots to explore unmanned spaces and difficult terrains like mines etc. and create a map of the environment for
potential hazards or safe navigation. It will also help in creating 3D models of commercial spaces for virtual tours
etc. without any human intervention.

xPLORER is an exploration Robot by Acme Robotics which works in an unknown indoor environment and generates a real time 3D map of the region as it traverses the whole environment and keeps on extending the map to develop a 3D octomap of the enviroment. It used turtlebot package on gazebo to simulate the exploration in a custom indoor world and octomap package to integrate the 3D mapping.  

<p align="center">
  <img width="700" height="500" src="https://github.com/rishchou/xPLORER/blob/master/results/demo1.png">
</p>

## Personnel
1. Rishabh Choudhary - Graduate Student pursuing Masters of Engineering in Robotics at University of Maryland. My interests lie in computer vision, ROS, machine learning and AI.
Email - rishchou@umd.edu

2. Akash Atharv - Masters student at University of Maryland majoring in Robotics. My areas of interests are path planning algorithms, medical robotics and AI.
Email - atharv@umd.edu
## Pair programming sprint assignment

Sprint 1:
- Driver - Rishabh Choudhary
- Navigator - Akash Atharv

Sprint 2:
- Driver - Akash Atharv
- Navigator - Rishabh Choudhary

Sprint 3: 
- Driver - Rishabh Choudhary
- Navigator - Akash Atharv

## Project Workflow
The activity diagram shown below explains the working of the project. 
<p align="center">
  <img width="700" height="800" src="https://github.com/rishchou/xPLORER/blob/master/UML/Revised/Activity_Diagram_revised.jpeg">
</p>

## Dependencies

The project requires ROS kinetic, catkin, Gazebo, TurtleBot and octomap packages and it is developed on UBUNTU 16.04 LTS. 

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

To install octomap package, use the given command 
```
sudo apt-get install ros-kinetic-octomap
```

## SIP (Solo Iterative Process)
 
This project was developed following pair programming concepts and SIP. The estimated and completed tasks have been stored in the form of product backlog, Iteration backlog and work log to include the specifics of each task. The product backlog contains the set of all of the tasks to be completed for the given feature implementation. The iteration backlog includes tasks that were repeated over the course of Sprint.

Detailed SIP Enactment with product backlog,iteration backlog and work log can be found [here](https://docs.google.com/spreadsheets/d/1m1UHrcsnNCY8bqwcfVqGJSqxgKsjhua1r7esJMIEKb0/edit?usp=sharing)

Sprint notes can be found [here](https://docs.google.com/document/d/1vihsMah5-x3lxd72F6U38xuexMgioh-XDoyO9bQRQPM/edit?usp=sharing)

## To-do tasks for pair programming (Driver navigator discussion)
- [x] Add class and activity diagram 
- [x] Add defect log and release backlog
- [x] Add LICENSE file
- [x] Update SIP Logs
- [x] Add pending test cases
- [x] Fix existing bugs 
- [x] Upload demo presentation

## Operation/Demo steps

### Build the project
To build the given project, follow the steps below:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/rishchou/xPLORER.git
cd ..
catkin_make
```

### Run the demo

To run the demo, follow the given steps:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch xplorer demo.launch
```

This will open up a set of windows and terminals used to simulate the demo as shown below. A gazebo window will open up showing turtlebot moving in a custom world. There is a terminal window which shows the status of the bot alignment and whether any obstacles are detected. 
The rviz window shows the actual 3D map generation using octomap in real time as the bot moves in the indoor environment. 

<p align="center">
  <img width="600" height="400" src="https://github.com/rishchou/xPLORER/blob/master/results/demo.png">
</p>

## Presentation 
The video presentation for the given project can be found [here](https://youtu.be/rrMOxFsIhBU)

The slides for the presentation can be found [here](https://drive.google.com/file/d/1L6KJ7QDFrKCbVW0Tuh6KQ3wDBHrHpKEu/view?usp=sharing) 

## Save the generated map

To save the map generated in real time after a satisfactory region has been covered by the turtlebot, use the given command while the demo.launch is running.
rosrun octomap_server octomap_saver results/<filename.ot>

This will save the generated map as an octree file format in results directory. 

## View the generated map

The generated map can be used further for autonomous navigation and other applications. To view the generated map you need to install octovis. To install octovis enter the following command:
```
sudo apt-get install ros-kinetic-octovis
```

Now, to view the generated map enter the following command.
```
octovis <path_to_map_file>
```

## ROSBAG 

The demo.launch file has a feature of recording the rosbag for all topics except /camera topic. To record a rosbag while running the demo enter the given command in a new terminal.
```
source ~/catkin_ws/devel/setup.bash
roslaunch xplorer demo.launch record:=true
```

This will record a bag file for the simulation for default 30 secs. You can change the duration of recorded bad file by using the record_time parameter with launch file above.

The recorded rosbag will be stored in the results directory of the repository.

## Playing the BAG file

To play the ROSBAG file, go to the results directory and enter the following command.

```
cd <path_to_repo>/results
rosbag play xplorer.bag
```

## Running ROStest

Unit tests have been written for each class to test the functionality and interface using gtest and rostest. To run rostest follow the given steps:

```
cd ~/catkin_ws
catkin_make run_tests_xplorer
```

We can also run the test node using the test.launch file as shown below:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch xplorer test.launch
```

## Doxygen Documentation 

The doxygen generated documents have been added to the docs folder of the repository. A config file has been added to docs folder to generate the documentation.

To generate the doxygen documentation, use the following commands.

```
cd <path_to_repository>/docs
doxygen xplorer.config
```

## Known issues/ bugs
The octomap uses the /camera/depth/points topic to create the 3D octomap of the environment using the point cloud. Use of ROSBAG for camera topics is inefficient as the bag file increases in size exponentially within a few seconds. Alternative ways need to be found out to record camera topics.
 
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

## About the Authors

Akash Atharv is a graduate student in Robotics at University of Maryland, College Park(Expected to graduate - May 2020). His interests include Robot Path Planning, Artificial Intelligence and Medical robotics.

Rishabh Choudhary is also a graduate student in Robotics at University of Maryland, College Park(Expected to graduate - May 2020). His interests include Software Engineering and Deep Learning.
