# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_serial)

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines)
find_package(Boost QUIET REQUIRED COMPONENTS thread)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


set(SerialSRCS
	src/${PROJECT_NAME}/XBee.cpp
	src/${PROJECT_NAME}/SerialDevice.cpp
	src/${PROJECT_NAME}/ThreadedSerialDevice.cpp
	src/${PROJECT_NAME}/SerialException.cpp
	src/${PROJECT_NAME}/SerialHelper.cpp
)

# ${PROJECT_NAME}/SetSerialClone.cpp


# Build library
add_library(${PROJECT_NAME} 
	${SerialSRCS}
)


# Build test programs
add_executable(test_serial 
	src/test_serial/test_serial.cpp
)
target_link_libraries(test_serial 
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

add_executable(test_threadedserial 
	src/test_serial/test_threadedserial.cpp
)
target_link_libraries(test_threadedserial 
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

catkin_package(
    DEPENDS roscpp 
    CATKIN_DEPENDS telekyb_defines
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


