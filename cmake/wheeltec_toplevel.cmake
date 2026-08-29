cmake_minimum_required(VERSION 3.5)

# A normal catkin workspace points CMakeLists.txt at one ROS installation.
# This indirection keeps the repository usable with both supported distros.
set(_wheeltec_supported_distros melodic noetic)

if(DEFINED ENV{WHEELTEC_ROS_DISTRO} AND DEFINED ENV{ROS_DISTRO}
   AND NOT "$ENV{WHEELTEC_ROS_DISTRO}" STREQUAL ""
   AND NOT "$ENV{ROS_DISTRO}" STREQUAL ""
   AND NOT "$ENV{WHEELTEC_ROS_DISTRO}" STREQUAL "$ENV{ROS_DISTRO}")
  message(FATAL_ERROR
    "WHEELTEC_ROS_DISTRO='$ENV{WHEELTEC_ROS_DISTRO}' conflicts with the "
    "already loaded ROS_DISTRO='$ENV{ROS_DISTRO}'. Open a clean terminal "
    "before switching ROS distributions.")
endif()

if(DEFINED ENV{WHEELTEC_ROS_DISTRO} AND NOT "$ENV{WHEELTEC_ROS_DISTRO}" STREQUAL "")
  set(_wheeltec_ros_distro "$ENV{WHEELTEC_ROS_DISTRO}")
elseif(DEFINED ENV{ROS_DISTRO} AND NOT "$ENV{ROS_DISTRO}" STREQUAL "")
  set(_wheeltec_ros_distro "$ENV{ROS_DISTRO}")
elseif(EXISTS "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake")
  set(_wheeltec_ros_distro noetic)
elseif(EXISTS "/opt/ros/melodic/share/catkin/cmake/toplevel.cmake")
  set(_wheeltec_ros_distro melodic)
else()
  message(FATAL_ERROR
    "No supported ROS 1 installation was found. Install ROS Melodic/Noetic, "
    "or set WHEELTEC_ROS_DISTRO before running catkin_make.")
endif()

if(NOT _wheeltec_ros_distro IN_LIST _wheeltec_supported_distros)
  message(FATAL_ERROR
    "Unsupported ROS distribution '${_wheeltec_ros_distro}'. "
    "This repository supports: melodic; noetic.")
endif()

set(_wheeltec_catkin_toplevel
  "/opt/ros/${_wheeltec_ros_distro}/share/catkin/cmake/toplevel.cmake")
if(NOT EXISTS "${_wheeltec_catkin_toplevel}")
  message(FATAL_ERROR
    "WHEELTEC_ROS_DISTRO/ROS_DISTRO selects '${_wheeltec_ros_distro}', but "
    "${_wheeltec_catkin_toplevel} does not exist.")
endif()

message(STATUS "WheelTec workspace ROS distribution: ${_wheeltec_ros_distro}")
include("${_wheeltec_catkin_toplevel}")
