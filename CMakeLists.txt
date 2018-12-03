cmake_minimum_required(VERSION 2.8.3)
project(xplorer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xplorer src/main.cpp src/navigator.cpp src/obstacleDetector.cpp)
target_link_libraries(xplorer ${catkin_LIBRARIES})

find_package(octomap REQUIRED)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testxplorer launch/test.launch test/main.cpp test/testNavigator.cpp test/testObstacleDetector.cpp src/navigator.cpp src/obstacleDetector.cpp)
  target_link_libraries(testxplorer ${catkin_LIBRARIES})
endif()
