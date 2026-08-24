cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_sorting)

set(CMAKE_AUTOMOC ON)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  pluginlib
  roscpp
  rviz
  std_msgs
  std_srvs
)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  LIBRARIES aubo_mobile_sorting_panel aubo_mobile_sorting_grasp_plugin
  CATKIN_DEPENDS gazebo_ros pluginlib roscpp rviz std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(aubo_mobile_sorting_panel
  include/aubo_mobile_sorting/sorting_panel.h
  src/sorting_panel.cpp
)
target_link_libraries(aubo_mobile_sorting_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_library(aubo_mobile_sorting_grasp_plugin SHARED
  src/grasp_attach_world_plugin.cpp
)
target_link_libraries(aubo_mobile_sorting_grasp_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS aubo_mobile_sorting_panel aubo_mobile_sorting_grasp_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES README.md plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
