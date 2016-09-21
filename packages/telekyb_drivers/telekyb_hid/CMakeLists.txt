# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_hid)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines telekyb_base sensor_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

SET(HIDAPI_INCLUDE_DIR $ENV{HIDAPI_INCLUDE_DIR})
IF (NOT DEFINED HIDAPI_INCLUDE_DIR)
	MESSAGE(FATAL_ERROR "Please point to the direction of the hidapi installation")	
ENDIF()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
# set(EIGEN_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
# IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
# 	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
# ENDIF()


# add Source Dir as Definition
add_definitions(-DTELEKYB_HID_PROJECT_SOURCE_DIR='\"${PROJECT_SOURCE_DIR}\"')

#common commands for building c++ executables and libraries
add_library(${PROJECT_NAME}
	src/JoystickOptions.cpp
	src/JoystickConfig.cpp
	src/Joystick.cpp
	src/HIDDevice.cpp
)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
    	${EIGEN_INCLUDE_DIRS}
	${HIDAPI_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)
message(STATUS "hidapi include dir ${HIDAPI_INCLUDE_DIR}")
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

add_executable(hidtest src/hidtest.cpp)
add_executable(tJoy 
	src/joy.cpp
)
target_link_libraries(tJoy ${PROJECT_NAME})
target_link_libraries(tJoy ${Boost_LIBRARIES})

add_executable(tJoyConfig 
	src/joyconfig.cpp
)
target_link_libraries(tJoyConfig ${PROJECT_NAME})


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp telekyb_defines telekyb_base hidapi_wrapper yaml_wrapper sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
