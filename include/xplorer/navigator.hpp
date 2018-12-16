/**============================================================================
 * @file       : navigator.hpp
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
 * @brief      : Class implementation for Robot Navigation functionality
 *============================================================================
 */
#ifndef INCLUDE_XPLORER_NAVIGATOR_HPP_
#define INCLUDE_XPLORER_NAVIGATOR_HPP_


#include "geometry_msgs/Twist.h"
#include "obstacleDetector.hpp"
/**
 * @brief Class for navigation part of the robot
 */
class navigator {
 private:
// Create a ROS node handle
    ros::NodeHandle n2;
// Publisher for publishing velocity to the turtlebot
    ros::Publisher pub2;
// Declare variable to store to-be published velocities
    geometry_msgs::Twist msg;

 public:
/**
 * @brief Constructor
 */
    navigator();
/**
 * @brief Destructor
 */
    ~navigator();
/**
 * @brief Function to move the robot
 * @param flag for operation
 * @return void
 */
    void explore(int flag);
// Create object to initialize obstacleDetector class
    obstacleDetector obsDet;
};

#endif  // INCLUDE_XPLORER_NAVIGATOR_HPP_
