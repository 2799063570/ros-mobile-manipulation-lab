# libcontrolAPI_INCLUDE_DIRS - the libcontrolAPI include directories
# libcontrolAPI_LIBS - link these to use libauborobotcontroller

file(COPY config
	DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

set(_AUBO_SDK_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

find_path(libcontrolAPI_INCLUDE_DIR
	NAMES serviceinterface.h
	PATHS ${_AUBO_SDK_ROOT}/include/aubo_driver
	NO_DEFAULT_PATH
)

find_library(libauborobotcontroller
	NAMES auborobotcontroller
	PATHS ${_AUBO_SDK_ROOT}/lib/lib64/aubocontroller ${_AUBO_SDK_ROOT}/lib/lib32
	NO_DEFAULT_PATH
)

find_library(libev
	NAMES ev
	PATHS ${_AUBO_SDK_ROOT}/lib/lib64 ${_AUBO_SDK_ROOT}/lib/lib32
	NO_DEFAULT_PATH
)

set(libcontrolAPI_INCLUDE_DIRS ${libcontrolAPI_INCLUDE_DIR})
set(libcontrolAPI_LIBS ${libauborobotcontroller})


if(libcontrolAPI_INCLUDE_DIRS)
	message(STATUS "Found Control API include dir: ${libcontrolAPI_INCLUDE_DIRS}")
else(libcontrolAPI_INCLUDE_DIRS)
	message(STATUS "Could NOT find Control API headers.")
endif(libcontrolAPI_INCLUDE_DIRS)


if(libcontrolAPI_LIBS)
	message(STATUS "Found Control API library: ${libcontrolAPI_LIBS}")
else(libcontrolAPI_LIBS)
	message(STATUS "Could NOT find libcontrolAPI library.")
endif(libcontrolAPI_LIBS)

if(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
	set(libcontrolAPI_FOUND TRUE)
else(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
	set(libcontrolAPI_FOUND FALSE)
	if(libcontrolAPI_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find Control API.")
	endif(libcontrolAPI_FIND_REQUIRED)
endif(libcontrolAPI_INCLUDE_DIRS AND libcontrolAPI_LIBS)
