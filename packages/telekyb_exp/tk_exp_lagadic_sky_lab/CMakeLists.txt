# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_exp_lagadic_sky_lab)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs telekyb_base telekyb_interface cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

#message(FATAL_ERROR ${PROJECT_SOURCE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

add_executable(tk_exp_lagadic_sky_lab
	src/Main.cpp
	src/Experiment.cpp
	#src/FormationControl.cpp
	#src/FormationControlElement.cpp
)

add_dependencies(tk_exp_lagadic_sky_lab
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(tk_exp_lagadic_sky_lab
        #${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)


catkin_package(
    DEPENDS roscpp sensor_msgs
    CATKIN_DEPENDS telekyb_base telekyb_interface
    INCLUDE_DIRS include
    LIBRARIES 
)


