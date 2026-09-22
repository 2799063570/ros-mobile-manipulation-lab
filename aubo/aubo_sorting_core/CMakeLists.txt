cmake_minimum_required(VERSION 3.0.2)
project(aubo_sorting_core)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  aubo_perception
  control_msgs
  dynamic_reconfigure
  geometry_msgs
  inspire_gripper
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

generate_dynamic_reconfigure_options(cfg/SortingTask.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES color_sorting_task
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    aubo_perception
    control_msgs
    dynamic_reconfigure
    geometry_msgs
    inspire_gripper
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

add_library(color_sorting_task
  src/sorting_task.cpp
  src/task_parameters.cpp
  src/target_tracking.cpp
  src/instance_tracking.cpp
  src/workspace_manager.cpp
  src/motion_executor.cpp
  src/grasp_attachment.cpp
  src/pick_place.cpp
  src/task_utils.cpp
)
add_dependencies(color_sorting_task ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_sorting_task ${catkin_LIBRARIES})

add_executable(sorting_task_cpp src/sorting_task_node.cpp)
add_dependencies(sorting_task_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(sorting_task_cpp color_sorting_task ${catkin_LIBRARIES})

# Backward-compatible executable for existing rosrun commands and custom launch files.
add_executable(color_sorting_task_cpp src/sorting_task_node.cpp)
target_compile_definitions(color_sorting_task_cpp PRIVATE
  AUBO_SORTING_NODE_NAME="color_sorting_task_cpp")
add_dependencies(color_sorting_task_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(color_sorting_task_cpp color_sorting_task ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS color_sorting_task sorting_task_cpp color_sorting_task_cpp
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

if(CATKIN_ENABLE_TESTING)
  add_executable(instance_queue_test test/instance_queue_test.cpp)
  add_test(NAME instance_queue_test COMMAND instance_queue_test)
  catkin_add_gtest(height_recovery_test test/height_recovery_test.cpp)
  catkin_add_gtest(observation_tf_test test/observation_tf_test.cpp)
  if(TARGET observation_tf_test)
    target_link_libraries(observation_tf_test ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test)
endif()
