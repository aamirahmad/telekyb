# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_parameter_estimators)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines telekyb_base tk_draft_msgs pluginlib tk_param_estimator cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)


#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
)


add_library(parameter_estimators_plugin
	src/ParamEstimators/StandardMassEstimator.cpp
 	src/ParamEstimators/NoMassEstimator.cpp
 	src/ParamEstimators/StandardInertiaMatrixEstimator.cpp
)

add_dependencies(parameter_estimators_plugin
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(parameter_estimators_plugin
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_defines telekyb_base tk_param_estimator
    INCLUDE_DIRS include
    LIBRARIES parameter_estimators_plugin
)

