# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_behavior)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_trajctrl tk_trajprocessor tk_state pluginlib cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)
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

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
#	${ODEINTV2_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_library(behaviors
	src/behaviors/TakeOff.cpp
	src/behaviors/Hover.cpp
	src/behaviors/NormalBrake.cpp
	src/behaviors/Land.cpp
	src/behaviors/Ground.cpp
	src/behaviors/EmergencyLand.cpp
)

add_library(${PROJECT_NAME}
	src/tk_behavior/BehaviorControllerInterface.cpp
	src/tk_behavior/Behavior.cpp
	src/tk_behavior/SystemBehaviorContainer.cpp
	src/tk_behavior/BehaviorInterface.cpp
	src/tk_behavior/BehaviorContainer.cpp
	src/tk_behavior/ActiveBehaviorContainer.cpp
	src/tk_behavior/BehaviorController.cpp
	src/tk_behavior/BehaviorControllerOptions.cpp
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


target_link_libraries(behaviors
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

#add_executable(behaviorTest_node src/test/BehaviorTester.cpp)


#target_link_libraries(behaviorTest_node
#        ${Boost_LIBRARIES}
#        ${YAMLCPP_LIBRARY}
#        ${catkin_LIBRARIES}
#	${PROJECT_NAME}
#)


catkin_package(
    DEPENDS roscpp Boost pluginlib
    CATKIN_DEPENDS telekyb_base telekyb_defines telekyb_msgs telekyb_srvs tk_state tk_trajctrl tk_trajprocessor
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} behaviors
)

