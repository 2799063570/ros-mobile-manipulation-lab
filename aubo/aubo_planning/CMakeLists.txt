cmake_minimum_required(VERSION 3.0.2)
project(aubo_planning)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  shape_msgs
  tf2
  tf2_geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
    shape_msgs
    tf2
    tf2_geometry_msgs
    trajectory_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(gripper_control_demo src/gripper_control_demo.cpp)
target_link_libraries(gripper_control_demo ${catkin_LIBRARIES})

add_executable(pick_place_demo src/pick_place_demo.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES})

install(TARGETS gripper_control_demo pick_place_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
