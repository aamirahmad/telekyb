# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_smurf_interface)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp tk_draft_msgs telekyb_msgs telekyb_base telekyb_serial cmake_modules)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the EIGEN3 installation")	
#ENDIF()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
    	${EIGEN_INCLUDE_DIR}
    	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


# Delegate
# Smurf Interface srcs Library
add_library(${PROJECT_NAME}
	src/smurf_interface/SerialIMUDevice.cpp
	src/smurf_interface/SerialIMUDeviceOptions.cpp
	src/smurf_interface/IMUDataProcessor.cpp
	src/smurf_interface/IMUDataProcessorOptions.cpp
	src/smurf_interface/IMUCalibrator.cpp
	src/smurf_interface/SerialCommandDevice.cpp
	src/smurf_interface/SerialCommandDeviceOptions.cpp
	src/smurf_interface/SmurfInterfaceOptions.cpp
)

add_executable(smurf_interface
	 src/smurf_interface/Main.cpp
)

add_dependencies(${PROJECT_NAME}
telekyb_msgs_generate_messages_cpp
telekyb_srvs_generate_messages_cpp
tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(smurf_interface
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${Boost_LIBRARIES}
	${PROJECT_NAME}
)

catkin_package(
    DEPENDS roscpp YamlCpp Boost eigen3
    CATKIN_DEPENDS  tk_draft_msgs telekyb_msgs telekyb_base telekyb_serial
    INCLUDE_DIRS 
    LIBRARIES ${PROJECT_NAME}
)


