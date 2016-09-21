cmake_minimum_required(VERSION 2.8.3)
project(tk_exp_teleoperation)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs telekyb_msgs telekyb_base telekyb_interface cmake_modules)

## System dependencies are found with CMake's conventions
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
#	${PROJECT_SOURCE_DIR}/include
#    ${PROJECT_SOURCE_DIR}
    ${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(tk_exp_teleoperation
#   src/${PROJECT_NAME}/tk_exp_teleoperation.cpp
# )

## Declare a cpp executable
add_executable(tk_exp_teleoperation
    src/Main.cpp
    src/TeleopExperiment.cpp)

add_dependencies(tk_exp_teleoperation
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(tk_exp_teleoperation_node tk_exp_teleoperation_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tk_exp_teleoperation
    ${YAMLCPP_LIBRARY}
    ${catkin_LIBRARIES}
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tk_exp_teleoperation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

