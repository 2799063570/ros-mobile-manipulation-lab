cmake_minimum_required(VERSION 3.0.2)
project(aubo_vla)
find_package(catkin REQUIRED)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  # The check_* scripts manipulate a live Gazebo instance and remain explicit
  # deployment checks; only the hermetic loopback service test runs by default.
  catkin_add_nosetests(tests/test_service.py)
endif()

catkin_install_python(PROGRAMS scripts/observe_bridge.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
