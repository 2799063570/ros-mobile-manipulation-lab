cmake_minimum_required(VERSION 3.0.2)
project(aubo_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  tf
)

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs message_runtime rospy sensor_msgs std_msgs tf
)

catkin_install_python(PROGRAMS
  scripts/color_object_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
