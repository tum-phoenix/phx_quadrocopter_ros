cmake_minimum_required(VERSION 2.8.3)
project(phx_odometry)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs nav_msgs tf2_ros )

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_node src/odometry_node.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES})