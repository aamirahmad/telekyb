# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_traj_controllers)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_param_estimator tk_draft_msgs tk_state tk_trajctrl tk_ctrlalgo cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

#set(ODEINTV2_INCLUDE_DIRS $ENV{ODEINT_V2_INCLUDE_DIR})
#IF (NOT DEFINED ODEINTV2_INCLUDE_DIRS)
#        MESSAGE(FATAL_ERROR "Please point to the direction of the odeint v2 installation")
#ENDIF()

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${ODEINTV2_INCLUDE_DIRS}
)

add_library(trajectory_trackers_plugin
	src/TrajectoryTrackers/SMURFTrajectoryTracker.cpp
	src/TrajectoryTrackers/SMURFTrajectoryTrackerExtForceCompensated.cpp
	src/TrajectoryTrackers/OmegaNHFCTrajectoryTracker.cpp
	src/TrajectoryTrackers/AdaptiveTrajectoryTracker.cpp
	src/TrajectoryTrackers/StandardTrajectoryTracker.cpp
	src/TrajectoryTrackers/PositionForceTrajectoryTracker.cpp
	src/TrajectoryTrackers/EurocTrajectoryTracker.cpp
	src/TrajectoryTrackers/TeleoperationTrajectoryTracker.cpp
	src/TrajectoryTrackers/GeometricTrajectoryTracker.cpp
	src/TrajectoryTrackers/TubeMPCTrajectoryTracker.cpp
	src/TrajectoryTrackers/RoMPCTrajectoryTracker.cpp
	src/TrajectoryTrackers/RoBSCTrajectoryTracker.cpp
)

add_dependencies(trajectory_trackers_plugin
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(trajectory_trackers_plugin
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base telekyb_defines telekyb_msgs tk_param_estimator tk_draft_msgs tk_state tk_ctrlalgo tk_trajctrl
    INCLUDE_DIRS include
    LIBRARIES trajectory_trackers_plugin
)


