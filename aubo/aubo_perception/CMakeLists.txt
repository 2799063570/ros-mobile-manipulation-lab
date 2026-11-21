cmake_minimum_required(VERSION 3.0.2)
project(aubo_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_sensor_msgs
)

catkin_python_setup()

add_message_files(FILES
  DetectedObject.msg
  DetectedObjectArray.msg
  YoloDetection.msg
  YoloDetectionArray.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters message_runtime roscpp rospy sensor_msgs std_msgs std_srvs tf tf2_ros tf2_sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(workspace_cloud_filter_node src/workspace_cloud_filter_node.cpp)
target_compile_features(workspace_cloud_filter_node PRIVATE cxx_std_14)
target_link_libraries(workspace_cloud_filter_node ${catkin_LIBRARIES})
add_dependencies(workspace_cloud_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

option(AUBO_ENABLE_TENSORRT_YOLO "Build the CUDA/TensorRT OBB detector" OFF)
if(AUBO_ENABLE_TENSORRT_YOLO)
  find_package(CUDA REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
  set(TENSORRT_ROOT "" CACHE PATH "TensorRT SDK root when installed outside system paths")
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS ${TENSORRT_ROOT} PATH_SUFFIXES include)
  find_library(TENSORRT_LIBRARY NAMES nvinfer HINTS ${TENSORRT_ROOT} PATH_SUFFIXES lib lib64)
  if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
    message(FATAL_ERROR "TensorRT 10+ C++ SDK not found. Set TENSORRT_ROOT or install the SDK system-wide.")
  endif()

  add_executable(tensorrt_yolo_node src/tensorrt_yolo_node.cpp)
  target_compile_features(tensorrt_yolo_node PRIVATE cxx_std_17)
  target_include_directories(tensorrt_yolo_node PRIVATE ${TENSORRT_INCLUDE_DIR} ${CUDA_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(tensorrt_yolo_node ${catkin_LIBRARIES} ${TENSORRT_LIBRARY} ${CUDA_CUDART_LIBRARY} ${OpenCV_LIBS})
  add_dependencies(tensorrt_yolo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  install(TARGETS tensorrt_yolo_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()

catkin_install_python(PROGRAMS
  scripts/check_rgbd_camera.py
  scripts/move_to_observation.py
  scripts/color_object_detector.py
  scripts/rgbd_visual_target_node.py
  scripts/ultralytics_yolo_node.py
  scripts/yolo_rgbd_target_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/ultralytics_yolo_node.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config environment launch msg models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS workspace_cloud_filter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES README.md D435I_DEBUG.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
