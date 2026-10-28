cmake_minimum_required(VERSION 3.1.3)
project(aubo_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
)
find_package(ruckig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aubo_moveit_request_adapters
  CATKIN_DEPENDS moveit_core roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(aubo_moveit_request_adapters src/zero_boundary_acceleration.cpp src/limit_spline_dynamics.cpp src/straight_path_timing.cpp)
target_compile_features(aubo_moveit_request_adapters PRIVATE cxx_std_17)
target_link_libraries(aubo_moveit_request_adapters ${catkin_LIBRARIES} ruckig::ruckig)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS aubo_moveit_request_adapters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES aubo_moveit_request_adapters_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(spline_dynamics_test test/spline_dynamics.test test/spline_dynamics_test.cpp)
  if(TARGET spline_dynamics_test)
    target_compile_features(spline_dynamics_test PRIVATE cxx_std_14)
    target_link_libraries(spline_dynamics_test aubo_moveit_request_adapters ${catkin_LIBRARIES})
  endif()
endif()
