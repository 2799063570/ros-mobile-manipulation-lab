cmake_minimum_required(VERSION 3.0.2)
project(aubo_vla)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  # check_* 脚本会操作实际 Gazebo 场景，只在显式联调时执行；
  # 默认注册无需启动仿真的服务、动作规范和数据集回归测试。
  catkin_add_nosetests(tests/test_service.py)
  catkin_add_nosetests(tests/test_actions.py)
  catkin_add_nosetests(tests/test_dataset.py)
  catkin_add_nosetests(tests/test_timing.py)
  catkin_add_nosetests(tests/test_bridge.py)
  catkin_add_nosetests(tests/test_recorder.py)
  catkin_add_nosetests(tests/test_shadow.py)
  catkin_add_nosetests(tests/test_grasp_check.py)
  catkin_add_nosetests(tests/test_automation.py)
endif()

catkin_install_python(PROGRAMS scripts/observe_bridge.py scripts/record_episode.py
  scripts/shadow_preview.py scripts/inspect_episode.py scripts/run_grasp_demo.py
  scripts/execute_grasp_workflow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md docs/DEBUGGING.md docs/ACTION_SCHEMA.md docs/EXECUTION_FLOW.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs)
