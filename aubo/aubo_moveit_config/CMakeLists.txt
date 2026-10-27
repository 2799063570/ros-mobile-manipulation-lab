cmake_minimum_required(VERSION 3.1.3)
project(aubo_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
)

catkin_package(
  LIBRARIES aubo_moveit_request_adapters
  CATKIN_DEPENDS moveit_core roscpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(aubo_moveit_request_adapters src/zero_boundary_acceleration.cpp)
target_link_libraries(aubo_moveit_request_adapters ${catkin_LIBRARIES})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS aubo_moveit_request_adapters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES aubo_moveit_request_adapters_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
