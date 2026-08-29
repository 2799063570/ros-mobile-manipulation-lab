cmake_minimum_required(VERSION 3.0.2)
project(aubo_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_sensor_msgs
)

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters message_runtime roscpp rospy sensor_msgs std_msgs std_srvs tf tf2_ros tf2_sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(workspace_cloud_filter_node src/workspace_cloud_filter_node.cpp)
target_compile_features(workspace_cloud_filter_node PRIVATE cxx_std_14)
target_link_libraries(workspace_cloud_filter_node ${catkin_LIBRARIES})
add_dependencies(workspace_cloud_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/color_object_detector.py
  scripts/rgbd_visual_target_node.py
  scripts/yolo_rgbd_target_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS workspace_cloud_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
