cmake_minimum_required(VERSION 3.0.2)
project(aubo_mobile_nav_sorting)

set(CMAKE_AUTOMOC ON)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  geometry_msgs
  move_base_msgs
  pluginlib
  roscpp
  rviz
  std_msgs
  std_srvs
  tf
)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

generate_dynamic_reconfigure_options(
  cfg/NavSorting.cfg
)

catkin_package(
  LIBRARIES aubo_mobile_nav_sorting_panel
  CATKIN_DEPENDS actionlib actionlib_msgs dynamic_reconfigure geometry_msgs
    move_base_msgs pluginlib roscpp rviz std_msgs std_srvs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(aubo_mobile_nav_sorting_panel
  include/aubo_mobile_nav_sorting/nav_sorting_panel.h
  src/nav_sorting_panel.cpp
)
add_dependencies(aubo_mobile_nav_sorting_panel
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(aubo_mobile_nav_sorting_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_executable(nav_sorting_mission_cpp
  src/navigation_sorting_mission.cpp
)
add_dependencies(nav_sorting_mission_cpp
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(nav_sorting_mission_cpp PRIVATE cxx_std_14)
target_link_libraries(nav_sorting_mission_cpp
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/publish_initial_pose.py
  scripts/nav_sorting_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY cfg config launch maps worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS aubo_mobile_nav_sorting_panel nav_sorting_mission_cpp
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
