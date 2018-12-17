/**============================================================================
 * @file       : obstacleDetector.cpp
 * @author     : Akash Atharv, Rishabh Choudhary
 * @version    : 1.0
 * @Copyright  : 3-Clause BSD

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
 * @brief      : Functional implementation for obstacle detection functionality
 *============================================================================
 */

#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include"../include/xplorer/obstacleDetector.hpp"

obstacleDetector::obstacleDetector() {
     ROS_INFO("Initializing obstacleDetector");
     // Initialize collision flag
        collision = false;
     // Subscribe to laserScan topic
        sub1 = n1.subscribe("/scan", 1000,
               &obstacleDetector::sensorCallback, this);
     // Message over dist topic is published
        disPub1 = n1.advertise<std_msgs::Float64>("/dist", 1000);
     // dist topic is subscribed
        disSub1 = n1.subscribe<std_msgs::Float64>("/dist", 1000,
                         &obstacleDetector::sensorCallbackDist, this);
}

obstacleDetector::~obstacleDetector() {
}

void obstacleDetector::sensorCallback(const sensor_msgs
                               ::LaserScan::ConstPtr& msg) {
    float val = 1000;
    for (const auto& i : msg->ranges) {
        if (i < val) {
            val = i;
        }
    }
    std_msgs::Float64 floatVal;
    floatVal.data = val;
    // Publish minimum distance in the range on /dist topic
    disPub1.publish(floatVal);
}

void obstacleDetector::sensorCallbackDist(const std_msgs
                                  ::Float64::ConstPtr& msg) {
    float val = 1;
    if ((msg -> data) < val) {
        collision = true;
    } else {
        collision = false;
    }
}

bool obstacleDetector::collisionDetect() {
        return collision;
}

void obstacleDetector::setFlag(bool val) {
        collision = val;
}



