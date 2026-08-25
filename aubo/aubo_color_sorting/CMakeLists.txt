cmake_minimum_required(VERSION 3.0.2)
project(aubo_color_sorting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
find_package(gazebo REQUIRED)

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  LIBRARIES aubo_color_sorting_grasp_plugin
  CATKIN_DEPENDS gazebo_ros geometry_msgs message_runtime roscpp rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(aubo_color_sorting_grasp_plugin SHARED
  src/grasp_attach_world_plugin.cpp)
add_dependencies(aubo_color_sorting_grasp_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aubo_color_sorting_grasp_plugin
  ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/color_object_detector.py
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS aubo_color_sorting_grasp_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
