cmake_minimum_required(VERSION 3.0.2)
project(aubo_handeye_validation)

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
  INCLUDE_DIRS include
  LIBRARIES handeye_verify_panel
  CATKIN_DEPENDS pluginlib roscpp rviz std_msgs std_srvs
)

add_library(handeye_verify_panel
  include/aubo_handeye_validation/handeye_verify_panel.h
  src/handeye_verify_panel.cpp
)
target_compile_features(handeye_verify_panel PRIVATE cxx_std_14)
target_include_directories(handeye_verify_panel PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(handeye_verify_panel ${catkin_LIBRARIES} Qt5::Widgets)

catkin_install_python(PROGRAMS
  scripts/validate_handeye.py
  scripts/handeye_yolo_verify.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch docs validation
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS handeye_verify_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
