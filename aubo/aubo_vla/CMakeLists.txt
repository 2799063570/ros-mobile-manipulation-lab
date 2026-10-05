cmake_minimum_required(VERSION 3.0.2)
project(aubo_vla)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  # The check_* scripts manipulate a live Gazebo instance and remain explicit
  # deployment checks; hermetic service/action/dataset tests run by default.
  catkin_add_nosetests(tests/test_service.py)
  catkin_add_nosetests(tests/test_actions.py)
  catkin_add_nosetests(tests/test_dataset.py)
  catkin_add_nosetests(tests/test_timing.py)
  catkin_add_nosetests(tests/test_bridge.py)
  catkin_add_nosetests(tests/test_recorder.py)
  catkin_add_nosetests(tests/test_shadow.py)
endif()

catkin_install_python(PROGRAMS scripts/observe_bridge.py scripts/record_episode.py
  scripts/shadow_preview.py scripts/inspect_episode.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md docs/DEBUGGING.md docs/ACTION_SCHEMA.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs)
