# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_handjoystick)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs telekyb_base sensor_msgs cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
)

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
ENDIF()

add_library(${PROJECT_NAME}
	src/Main.cpp
	src/HandJoystick.cpp
	src/HandJoystickOptions.cpp
)

add_dependencies(${PROJECT_NAME}
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)

add_executable(tk_handjoystick_node
	src/Main.cpp
	src/HandJoystick.cpp
	src/HandJoystickOptions.cpp
)

target_link_libraries(tk_handjoystick_node 
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
	${YAMLCPP_LIBRARY}
	${PROJECT_NAME}
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp geometry_msgs telekyb_base sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

