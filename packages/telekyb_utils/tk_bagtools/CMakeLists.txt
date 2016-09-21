# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_bagtools)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp rosbag telekyb_base sensor_msgs tk_haptics_msgs telekyb_msgs tk_draft_msgs sensor_msgs cmake_modules)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(YamlCpp REQUIRED)
find_package(Matlab REQUIRED)
find_package(Eigen REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the EIGEN3 installation")	
#ENDIF()

#set(MATLAB_INCLUDE_DIRS $ENV{MATLAB_INCLUDE_DIR})
#IF (NOT DEFINED MATLAB_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of your MATLAB installation")	
#ENDIF()

#message(${MATLAB_INCLUDE_DIRS})
#message(${MATLAB_INCLUDE_LIBARARIES})

include_directories(
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${MATLAB_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

# Mat data structures

set(MatDataStructuresSRCS
	src/MatStructs/MatStruct.cpp
	src/MatStructs/MatTKStateStruct.cpp
	src/MatStructs/MatTKTrajectoryStruct.cpp
	src/MatStructs/MatPoseStampedStruct.cpp
	src/MatStructs/MatMassStampedStruct.cpp
	src/MatStructs/MatTKSmallImuStruct.cpp
	src/MatStructs/MatTKTTCommandsStruct.cpp
	src/MatStructs/MatTKMotorCommandsStruct.cpp
	)

add_library(mat_data_structures ${MatDataStructuresSRCS})

add_dependencies(mat_data_structures
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)

add_executable(BagExport 
	src/BagExport.cpp
	src/BagExportOptions.cpp
	src/BagFile.cpp
	src/ROSBagMsgs/ROSBagBaseMsg.cpp
	)

add_dependencies(BagExport
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(BagExport 
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${YAMLCPP_LIBRARY}
	${MATLAB_LIBRARIES}
	mat_data_structures
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS #eigen3
    CATKIN_DEPENDS roscpp rosbag telekyb_base sensor_msgs tk_haptics_msgs telekyb_msgs tk_draft_msgs sensor_msgs
    INCLUDE_DIRS #include
    LIBRARIES # TODO
)


