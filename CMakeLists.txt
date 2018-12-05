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

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Coveralls support
option(ROS_CI_COVERALLS "Turn on coveralls support" OFF)
option(ROS_CI_COVERALLS_UPLOAD "Upload the generated coveralls json" OFF)

if($ENV{ROS_CI_COVERAGE})
  set(CMAKE_BUILD_TYPE Debug)
  set(ROS_CI_COVERALLS ON)
  set(ROS_CI_COVERALLS_UPLOAD ON)
endif()

if(ROS_CI_COVERALLS)
  include(Coveralls)
  coveralls_turn_on_coverage()
  message(STATUS "coveralls on")
endif()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(xplorer src/main.cpp src/navigator.cpp src/obstacleDetector.cpp)
target_link_libraries(xplorer ${catkin_LIBRARIES})

find_package(octomap REQUIRED)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testxplorer launch/test.launch test/main.cpp test/testNavigator.cpp test/testObstacleDetector.cpp)
  target_link_libraries(testxplorer ${catkin_LIBRARIES})
  add_dependencies(testxplorer xplorer ${catkin_EXPORTED_TARGETS})
endif()

if (ROS_CI_COVERALLS)
  set(COV_FILES)
  file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
  list(APPEND COV_FILES ${SRCS})
  file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
  list(APPEND COV_FILES ${HDRS})
  coveralls_setup("${COV_FILES}" ${ROS_CI_COVERALLS_UPLOAD})
endif()

