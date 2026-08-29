cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_robot_rc)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs joy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs 
  DEPENDS
)

catkin_install_python(PROGRAMS
  scripts/turtlebot_teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



