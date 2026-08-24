cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_sorting)

set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  rviz
  std_msgs
  std_srvs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

catkin_package(
  LIBRARIES aubo_mobile_sorting_panel
  CATKIN_DEPENDS pluginlib roscpp rviz std_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(aubo_mobile_sorting_panel
  include/aubo_mobile_sorting/sorting_panel.h
  src/sorting_panel.cpp
)
target_link_libraries(aubo_mobile_sorting_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

catkin_install_python(PROGRAMS
  scripts/color_sorting_task.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS aubo_mobile_sorting_panel
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
