cmake_minimum_required(VERSION 3.5)
project(aubo_sdk)

find_package(catkin REQUIRED COMPONENTS roslib)
find_package(Threads REQUIRED)

set(AUBO_SDK_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/aubo_driver")
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(AUBO_SDK_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/lib/lib64/aubocontroller")
  set(AUBO_SDK_RUNTIME_DIRS
    "${PROJECT_SOURCE_DIR}/lib/lib64/aubocontroller"
    "${PROJECT_SOURCE_DIR}/lib/lib64/log4cplus"
    "${PROJECT_SOURCE_DIR}/lib/lib64/config")
  set(AUBO_SDK_DEMO_INSTALL_RPATH
    "\$ORIGIN/../../share/aubo_sdk/lib/lib64/aubocontroller;\$ORIGIN/../../share/aubo_sdk/lib/lib64/log4cplus;\$ORIGIN/../../share/aubo_sdk/lib/lib64/config")
else()
  set(AUBO_SDK_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/lib/lib32")
  set(AUBO_SDK_RUNTIME_DIRS "${PROJECT_SOURCE_DIR}/lib/lib32")
  set(AUBO_SDK_DEMO_INSTALL_RPATH "\$ORIGIN/../../share/aubo_sdk/lib/lib32")
endif()
set(AUBO_SDK_CONTROLLER_LIB "${AUBO_SDK_LIBRARY_DIR}/libauborobotcontroller.so")

if(NOT EXISTS "${AUBO_SDK_CONTROLLER_LIB}")
  message(FATAL_ERROR "AUBO SDK library not found: ${AUBO_SDK_CONTROLLER_LIB}")
endif()

# The vendor SDK was built against protobuf 2.6 (SONAME libprotobuf.so.9).
# Bionic and Focal do not provide an ABI-compatible replacement under the
# current protobuf SONAME, so fail during configuration with an actionable
# message instead of emitting hundreds of undefined references at link time.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  find_file(AUBO_PROTOBUF9_LIBRARY NAMES libprotobuf.so.9
    PATHS /lib /usr/lib /usr/local/lib
    PATH_SUFFIXES x86_64-linux-gnu)
  if(NOT AUBO_PROTOBUF9_LIBRARY)
    message(FATAL_ERROR
      "AUBO SDK requires libprotobuf.so.9. Install Ubuntu package "
      "libprotobuf9v5 as documented in ROS1_COMPATIBILITY.md.")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roslib
  CFG_EXTRAS aubo_sdk-extras.cmake
)
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/logfiles")

function(add_aubo_sdk_demo target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_14)
  target_include_directories(${target} PRIVATE ${AUBO_SDK_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  target_link_libraries(${target} ${AUBO_SDK_CONTROLLER_LIB} ${AUBO_PROTOBUF9_LIBRARY} ${catkin_LIBRARIES} Threads::Threads)
  set_target_properties(${target} PROPERTIES
    BUILD_RPATH "${AUBO_SDK_RUNTIME_DIRS}"
    INSTALL_RPATH "${AUBO_SDK_DEMO_INSTALL_RPATH}"
    # Preserve a transitive DT_RPATH so libauborobotcontroller can resolve its
    # bundled libconfig/log4cplus dependencies without LD_LIBRARY_PATH.
    LINK_FLAGS "-Wl,--disable-new-dtags")
endfunction()

add_aubo_sdk_demo(sdk_test src/sdk_test.cpp)
add_aubo_sdk_demo(joint_move_demo src/joint_move_demo.cpp)

install(TARGETS sdk_test joint_move_demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(DIRECTORY config lib DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
