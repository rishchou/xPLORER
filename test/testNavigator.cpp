/**============================================================================
 * @file       : testNavigator.cpp
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
 * @brief      : Test file for testing class Navigator
 *============================================================================
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/xplorer/obstacleDetector.hpp"
#include "../include/xplorer/navigator.hpp"

/**
 * @brief      Class for testing
 */
class TestClass {
 private:
    geometry_msgs::Twist twist;
 public:
/**
 * @brief Temporary callback function 
 * @param msg Temporary message
 * @return void 
 */
    void publish(const geometry_msgs::Twist::ConstPtr& msg) {
         twist = *msg;
    }
};
/**
 * @brief Test to find if object is initialized and 
 *        the navigator program functions properly
 * @param TESTSuite Gtest framework
 * @param Navigator_Object_is_initialized Test name
 */
TEST(TESTSuite, Navigator_Object_is_initialized) {
        EXPECT_NO_FATAL_FAILURE(navigator navigator);
}
/**
 * @brief Test to find if publisher for /mobile_base/commands/velocity is working
 * @param TESTSuite Gtest framework
 * @param Publish_Test Test name
 */
TEST(TESTSuite, NavigatorPublisherTest) {
        ros::NodeHandle n7;
        TestClass test;
        navigator navigator;
        ros::Rate loop_rate(10);
        auto sub = n7.subscribe("/mobile_base/commands/velocity",
                           1000, &TestClass::publish, &test);
        EXPECT_EQ(1, sub.getNumPublishers());
}
/**
 * @brief Test to find if subscriber for /dist is working
 * @param TESTSuite Gtest framework
 * @param Subscribe_Test Test name
 */
TEST(TESTSuite, NavigatorSubscriberTest) {
        ros::NodeHandle n6;
        auto pub3 = n6.advertise<std_msgs::Float64>("/dist", 1000);
        ros::WallDuration(1).sleep();
        EXPECT_EQ(pub3.getNumSubscribers(), 1);
}
/**
 * @brief Test to find if the main function for robot movement is working
 * @param TESTSuite Gtest framework
 * @param RobotNavigatorTest Test name
 */
TEST(TESTSuite, RobotNavigatorTest) {
       ros::NodeHandle n8;
       navigator navigator;
       EXPECT_NO_FATAL_FAILURE(navigator.explore(0));
       navigator.obsDet.setFlag(true);
       EXPECT_NO_FATAL_FAILURE(navigator.explore(0));
}

