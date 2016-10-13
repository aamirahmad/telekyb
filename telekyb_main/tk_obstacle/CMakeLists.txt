# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_obstacle)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_calculus pluginlib cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
find_package(octomap QUIET)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
)





# create ${PROJECT_NAME} library
add_library(${PROJECT_NAME}
	src/obs_avoidance/ObstacleAvoidancePotential.cpp
	src/obs_detection/ObstacleProviderController.cpp
	src/obs_detection/ObstacleProvider.cpp
	src/obs_detection/ObstacleProviderControllerOptions.cpp
	src/obs_detection/ObstacleProviderContainer.cpp
)
add_dependencies(${PROJECT_NAME}
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)
# link ${PROJECT_NAME} library
target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)




# collect sources for ${PROJECT_NAME} library
set (TK_OBSTACLE_SRC
	src/obs_providers/ExternalObsPoint.cpp
	src/obs_providers/ExternalBox.cpp
	src/obs_providers/ConstantPoints.cpp
	src/obs_providers/ExternalFace.cpp
	src/obs_providers/SurroundingBox.cpp
)
if(octomap_FOUND)
	set(TK_OBSTACLE_SRC
		${TK_OBSTACLE_SRC}
		src/obs_providers/Octomap.cpp
	)
else(octomap_FOUND)
	message(WARNING "octomap NOT FOUND. Octomap obstacle provider will not be built.")
endif(octomap_FOUND)

# create ${PROJECT_NAME} library
add_library(obstacle_providers
	${TK_OBSTACLE_SRC}
)
# link ${PROJECT_NAME} library
target_link_libraries(obstacle_providers
	${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)
if(octomap_FOUND)
	target_link_libraries(${PROJECT_NAME} ${OCTOMAP_LIBRARIES})
endif(octomap_FOUND)






# create obstacle_provider executable
add_executable(obstacle_provider
	src/Main.cpp
)
target_link_libraries(obstacle_provider
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)





catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base telekyb_calculus pluginlib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} obstacle_providers
)

