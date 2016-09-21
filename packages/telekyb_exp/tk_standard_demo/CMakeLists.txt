cmake_minimum_required(VERSION 2.4.6)
project(tk_standard_demo)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base tk_draft_msgs telekyb_interface telekyb_joystick cmake_modules )
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
set(tk_standard_demoSRC
	src/fixedFormation/Main.cpp
	src/fixedFormation/FormationControl.cpp
	src/fixedFormation/FormationControlElement.cpp
)
add_executable(tk_standard_demo ${tk_standard_demoSRC})

add_dependencies(tk_standard_demo
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(tk_standard_demo
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)




# Behavior Plugins
# set(BehaviorsSRCS
# 	src/Behavior/Calibrator.cpp
# )
# add_library(be_mk_tools ${BehaviorsSRCS})
# 
# 
# add_dependencies(be_mk_tools
#         telekyb_msgs_generate_messages_cpp
#         telekyb_srvs_generate_messages_cpp
#         tk_draft_msgs_generate_messages_cpp
# )
# 
# target_link_libraries(be_mk_tools
#         ${Boost_LIBRARIES}
#         ${YAMLCPP_LIBRARY}
#         ${catkin_LIBRARIES}
# )



catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base tk_draft_msgs telekyb_interface telekyb_joystick cmake_modules 
    INCLUDE_DIRS src
    LIBRARIES be_mk_tools
)
