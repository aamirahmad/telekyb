cmake_minimum_required(VERSION 2.4.6)
project(tk_mk_tools)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_param_estimator tk_draft_msgs tk_state tk_trajctrl tk_ctrlalgo cmake_modules tk_behavior telekyb_interface)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${ODEINTV2_INCLUDE_DIRS}
)


# experiment
set(CalibratorSRCS
	src/Calibrator/Main.cpp
	src/Calibrator/Calibrator.cpp
)
add_executable(tk_calibrator ${CalibratorSRCS})

add_dependencies(tk_calibrator
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(tk_calibrator
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)




# Behavior Plugins
set(BehaviorsSRCS
	src/Behavior/Calibrator.cpp
)
add_library(be_mk_tools ${BehaviorsSRCS})


add_dependencies(be_mk_tools
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(be_mk_tools
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)



catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base telekyb_defines telekyb_msgs tk_param_estimator tk_draft_msgs tk_state tk_ctrlalgo tk_trajctrl tk_behavior telekyb_interface
    INCLUDE_DIRS src
    LIBRARIES be_mk_tools
)
