# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_vicon_emulator)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#message(FATAL_ERROR ${PROJECT_SOURCE_DIR})

include_directories(
#	${PROJECT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)

add_executable(telekyb_vicon_emulator
	src/ViconEmulator.cpp
)

target_link_libraries(telekyb_vicon_emulator
        ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp geometry_msgs 
    CATKIN_DEPENDS 
    INCLUDE_DIRS
    LIBRARIES
)

