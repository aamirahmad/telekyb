# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_cyber_motion)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs telekyb_base)

find_package(Boost REQUIRED COMPONENTS thread)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)



# Delegate
# CyberMotion srcs executable
set(CyberMotionSRCS
	src/Main.cpp
	src/CyberMotionxPCBridge.cpp
	src/CyberMotionxPCOptions.cpp
)

add_executable(CyberMotion ${CyberMotionSRCS})

target_link_libraries(CyberMotion
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp sensor_msgs telekyb_base
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
