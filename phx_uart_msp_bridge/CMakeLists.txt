cmake_minimum_required(VERSION 2.8.3)
project(phx_uart_msp_bridge)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  diagnostic_msgs
  message_generation
  tf
  tf2_ros
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Motor.msg
   RemoteControl.msg
   Servo.msg
   Battery.msg
   Cycletime.msg
   Diagnostics.msg
   Status.msg
   Altitude.msg
   Attitude.msg
   Autonomous.msg
   AutoPilotCmd.msg
   LEDstrip.msg
   LED.msg
   PID.msg
   PID_cleanflight.msg
   WayPoint.msg
   WayPoints.msg
   Management.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   sensor_msgs
   geometry_msgs
   diagnostic_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phx_uart_msp_bridge
   CATKIN_DEPENDS  rospy std_msgs sensor_msgs geometry_msgs diagnostic_msgs message_runtime tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp library
 add_library(phx_serial_com
   src/serial_com.cpp
 )

## Declare a cpp executable
add_executable(phx_uart_msp_bridge_node_naze src/phx_uart_msp_bridge_node_naze.cpp)
add_executable(phx_uart_msp_bridge_node_naze_mod src/phx_uart_msp_bridge_node_naze_mod.cpp)
add_executable(phx_uart_msp_bridge_node_marvicRCaltitudeServo src/phx_uart_msp_bridge_node_marvicRCaltitudeServo.cpp)
add_executable(phx_uart_msp_bridge_node_fully360 src/phx_uart_msp_bridge_node_fully360.cpp)
add_executable(phx_tf_copter_to_copter_stabilized src/transform_copter_stabilized_to_copter.cpp)

add_dependencies(phx_uart_msp_bridge_node_naze ${PROJECT_NAME}_gencpp)
add_dependencies(phx_tf_copter_to_copter_stabilized ${PROJECT_NAME}_gencpp)


target_link_libraries(phx_uart_msp_bridge_node_naze
   ${catkin_LIBRARIES}
   phx_serial_com
)


target_link_libraries(phx_uart_msp_bridge_node_naze_mod
   ${catkin_LIBRARIES}
   phx_serial_com
)


target_link_libraries(phx_uart_msp_bridge_node_marvicRCaltitudeServo
   ${catkin_LIBRARIES}
   phx_serial_com
)

target_link_libraries(phx_tf_copter_to_copter_stabilized
   ${catkin_LIBRARIES}
)


target_link_libraries(phx_uart_msp_bridge_node_fully360
   ${catkin_LIBRARIES}
   phx_serial_com
)

target_link_libraries(phx_tf_copter_to_copter_stabilized
   ${catkin_LIBRARIES}
)