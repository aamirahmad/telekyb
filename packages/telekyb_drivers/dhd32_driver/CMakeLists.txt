# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(dhd32_driver)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base tk_haptics_base telekyb_defines )
find_package(Boost REQUIRED COMPONENTS thread)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the EIGEN3 installation")	
#ENDIF()

catkin_package(
    DEPENDS roscpp dhd32_wrapper
    CATKIN_DEPENDS telekyb_defines telekyb_base tk_haptics_base
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
#	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
	src/Main.cpp
	src/DHDDevice.cpp
	src/DHDDeviceOptions.cpp
	src/DHDDriver.cpp
)

target_link_libraries(${PROJECT_NAME} 
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
)
add_executable(dhd_driver src/DHDDriver.cpp src/DHDDevice.cpp)

target_link_libraries(dhd_driver 
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)


