# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_gps)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs telekyb_base)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

#message(FATAL_ERROR ${PROJECT_SOURCE_DIR})

include_directories(
	${PROJECT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
)



#add_executable(vicongeographic2Cartesian
#	src/geographic2Cartesian.cpp
#)

#target_link_libraries(vicongeographic2Cartesian
#        ${catkin_LIBRARIES}
#)



add_executable(gpsDataLogger
	src/gpsDataLogger.cpp
)
add_executable(gpscat_node
	src/gpscat_node.cpp
)

target_link_libraries(gpsDataLogger
        ${catkin_LIBRARIES}
	${YAMLCPP_LIBRARY}
)
target_link_libraries(gpscat_node
        ${catkin_LIBRARIES}
	${YAMLCPP_LIBRARY}
)

catkin_package(
    DEPENDS roscpp geometry_msgs 
    CATKIN_DEPENDS telekyb_base
    INCLUDE_DIRS
    LIBRARIES 
)

