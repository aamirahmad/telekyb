# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_exception)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
ENDIF()

set(ODEINTV2_INCLUDE_DIRS $ENV{ODEINT_V2_INCLUDE_DIR})
IF (NOT DEFINED ODEINTV2_INCLUDE_DIRS)
        MESSAGE(FATAL_ERROR "Please point to the direction of the odeint v2 installation")
ENDIF()

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


#add_library(${PROJECT_NAME}
#)
if(YAMLCPP_FOUND)
	 include_directories(${YAMLCPP_INCLUDE_DIR})
ENDIF()

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
ENDIF()

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)


catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS  telekyb_base telekyb_defines telekyb_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

