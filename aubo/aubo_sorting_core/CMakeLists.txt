cmake_minimum_required(VERSION 3.0.2)
project(aubo_sorting_core)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  aubo_perception
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  rospy
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  tf
  trajectory_msgs
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES color_sorting_task
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    aubo_perception
    control_msgs
    geometry_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    rospy
    sensor_msgs
    shape_msgs
    std_msgs
    std_srvs
    tf
    trajectory_msgs
    urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(color_sorting_task src/color_sorting_task.cpp)
add_dependencies(color_sorting_task ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_sorting_task ${catkin_LIBRARIES})

add_executable(color_sorting_task_cpp src/color_sorting_task_node.cpp)
add_dependencies(color_sorting_task_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_sorting_task_cpp color_sorting_task ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS color_sorting_task color_sorting_task_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
