cmake_minimum_required(VERSION 3.0.2)
project(aubo_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS gazebo_ros roscpp std_msgs)
find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES aubo_grasp_attach_plugin
  CATKIN_DEPENDS gazebo_ros roscpp std_msgs
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(aubo_grasp_attach_plugin SHARED src/grasp_attach_world_plugin.cpp)
add_dependencies(aubo_grasp_attach_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(aubo_grasp_attach_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS aubo_grasp_attach_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
