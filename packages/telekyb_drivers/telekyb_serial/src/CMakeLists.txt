#Objects

set(SerialSRCS
	${PROJECT_NAME}/XBee.cpp
	${PROJECT_NAME}/SerialDevice.cpp
	${PROJECT_NAME}/ThreadedSerialDevice.cpp
	${PROJECT_NAME}/SerialException.cpp
	${PROJECT_NAME}/SerialHelper.cpp
)

# ${PROJECT_NAME}/SetSerialClone.cpp

rosbuild_add_boost_directories()

# Build library
rosbuild_add_library(${PROJECT_NAME} ${SerialSRCS})
rosbuild_link_boost(${PROJECT_NAME} thread)

# Build test programs
rosbuild_add_executable(test_serial test_serial/test_serial.cpp)
target_link_libraries(test_serial ${PROJECT_NAME})

rosbuild_add_executable(test_threadedserial test_serial/test_threadedserial.cpp)
target_link_libraries(test_threadedserial ${PROJECT_NAME})