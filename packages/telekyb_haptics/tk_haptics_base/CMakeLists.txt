# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_haptics_base)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib telekyb_defines telekyb_base tk_haptics_msgs cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
    	${EIGEN_INCLUDE_DIRS}
    	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
ENDIF()

# Haptic Controller

set(HapticControllerSRC
	src/HapticControllers/SampleController.cpp
)

add_library(haptic_controllers ${HapticControllerSRC})


catkin_package(
    DEPENDS roscpp pluginlib telekyb_defines telekyb_base tk_haptics_msgs
    CATKIN_DEPENDS telekyb_defines telekyb_base tk_haptics_msgs
    INCLUDE_DIRS include
    LIBRARIES haptic_controllers
)


