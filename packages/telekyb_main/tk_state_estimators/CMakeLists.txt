# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_state_estimators)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_draft_msgs sensor_msgs tk_state tf tf_conversions pluginlib cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)

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
	${ODEINTV2_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(state_estimators_plugin
#	src/StateEstimators/ViconVisionStateEstimator.cpp
#	src/StateEstimators/VelocityOnlyStateEstimator.cpp
#	src/StateEstimators/DVOStateEstimator.cpp
#	src/StateEstimators/MeasureHandler.cpp
#	src/StateEstimators/KalmanStateEstimator.cpp
# 	src/StateEstimators/ViconImuStateEstimator.cpp
	src/StateEstimators/ViconStateEstimator.cpp
# 	src/StateEstimators/ComplementaryStateEstimator.cpp
#	src/StateEstimators/KalmanStateEstimator/ViconHandler.cpp
#	src/StateEstimators/KalmanStateEstimator/KalmanStateEstimator.cpp
	src/StateEstimators/PassThroughStateEstimator.cpp
#	src/StateEstimators/SSXStateEstimator.cpp
	src/StateEstimators/KalmanPoseImuStateEstimator.cpp
	src/StateEstimators/EurocGroundTruthStateEstimator.cpp
	src/StateEstimators/EurocStateEstimator.cpp
)

add_dependencies(state_estimators_plugin
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)


target_link_libraries(state_estimators_plugin
       ${Boost_LIBRARIES}
       ${YAMLCPP_LIBRARY}
       ${catkin_LIBRARIES}
       ${EIGEN_LIBRARIES}
)


catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS  telekyb_base telekyb_defines telekyb_msgs tk_draft_msgs sensor_msgs tk_state tf tf_conversions pluginlib
    INCLUDE_DIRS include
    LIBRARIES state_estimators_plugin
)


