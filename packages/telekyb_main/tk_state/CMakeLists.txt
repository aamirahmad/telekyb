# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_state)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
	set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_draft_msgs sensor_msgs tf pluginlib cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
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
	#${ODEINTV2_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/tk_state/StateEstimator.cpp
	src/tk_state/StateEstimatorControllerOptions.cpp
	src/tk_state/StateEstimatorController.cpp
#	src/tk_state/EigenTools.cpp
)

add_dependencies(${PROJECT_NAME}
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)

#add_executable(stateEstimater_node src/test/StateEstimTester.cpp)
#target_link_libraries(stateEstimater_node ${PROJECT_NAME})

catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS  telekyb_base telekyb_defines telekyb_msgs tk_draft_msgs sensor_msgs tf pluginlib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


