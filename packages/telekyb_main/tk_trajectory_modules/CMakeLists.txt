# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_trajectory_modules)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs telekyb_srvs tk_state tk_trajctrl tk_obstacle geometry_msgs tk_trajprocessor cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set(ODEINTV2_INCLUDE_DIRS $ENV{ODEINT_V2_INCLUDE_DIR})
#IF (NOT DEFINED ODEINTV2_INCLUDE_DIRS)
#        MESSAGE(FATAL_ERROR "Please point to the direction of the odeint v2 installation")
#ENDIF()

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${ODEINTV2_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(trajectory_modules_plugin
	src/trajmodules/VMSaturation.cpp
#	src/trajmodules/VMTrajectorySmoothing.cpp
	src/trajmodules/XMModeCheck.cpp
#	src/trajmodules/AMTrajectorySmoothing.cpp
	src/trajmodules/VMObsAvoid.cpp
	src/trajmodules/PMPositionError.cpp
	src/trajmodules/PMObsAvoid.cpp
#	src/trajmodules/PMTrajectorySmoothing.cpp
	src/trajmodules/VMBinOccupancyObsAvoid.cpp
)


add_dependencies(trajectory_modules_plugin
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)


target_link_libraries(trajectory_modules_plugin
	${Boost_LIBRARIES}
	${YAMLCPP_LIBRARY}
	${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp Boost pluginlib
    CATKIN_DEPENDS  telekyb_base telekyb_defines telekyb_msgs telekyb_srvs tk_state tk_trajctrl tk_obstacle tk_trajprocessor geometry_msgs
    INCLUDE_DIRS include
    LIBRARIES trajectory_modules_plugin
)


