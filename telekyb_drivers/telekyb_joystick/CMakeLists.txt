# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_joystick)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines telekyb_base sensor_msgs cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Look for <linux/joystick.h>
include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)


include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
    	${EIGEN_INCLUDE_DIR}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


if(HAVE_LINUX_JOYSTICK_H)
add_library(${PROJECT_NAME}
	src/Joystick.cpp
	src/BaseJoystick.cpp
	src/JoystickOptions.cpp
)


add_executable(tJoy_node 
	src/joy.cpp
)
target_link_libraries(tJoy_node 
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)

add_dependencies(${PROJECT_NAME}
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)


else(HAVE_LINUX_JOYSTICK_H)
  message("Warning: no <linux/joystick.h>; won't build joy node")
endif(HAVE_LINUX_JOYSTICK_H)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS roscpp 
    CATKIN_DEPENDS telekyb_defines telekyb_base sensor_msgs
    INCLUDE_DIRS 
    LIBRARIES 
)

