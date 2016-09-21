# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_ctrlalgo)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -march=native")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines telekyb_base cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	src/YawControl.cpp
	src/PositionControl.cpp
	src/PositionForceControl.cpp
	src/OutputControl.cpp

	#src/TubeMPCCascade.cpp
	#src/MPC/OCPsolver.cpp	
	#src/MPC/ldl.c 
        #src/MPC/matrix_support.c 
        #src/MPC/solver.c 
	#src/MPC/util.c

	#src/TRO_RoMPC.cpp
	#src/RoMPC_20step/OCPsolver.cpp	
	#src/RoMPC_20step/ldl.c 
        #src/RoMPC_20step/matrix_support.c 
        #src/RoMPC_20step/solver.c 
	#src/RoMPC_20step/util.c

	src/TRO_RoBSC.cpp 	
	src/TrajPlanning/TrajOCPsolver.cpp	
	src/TrajPlanning/ldl.c 
        src/TrajPlanning/matrix_support.c 
        src/TrajPlanning/solver.c 
	src/TrajPlanning/util.c
)

add_dependencies(${PROJECT_NAME}
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
       ${Boost_LIBRARIES}
       ${YAMLCPP_LIBRARY}
       ${catkin_LIBRARIES}
)

catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_defines telekyb_base
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


