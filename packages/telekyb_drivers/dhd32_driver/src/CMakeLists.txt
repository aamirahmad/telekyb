
rosbuild_add_boost_directories()


rosbuild_add_executable(${PROJECT_NAME} 
	Main.cpp
	DHDDevice.cpp
	DHDDeviceOptions.cpp
	DHDDriver.cpp
)

rosbuild_link_boost(${PROJECT_NAME} thread)
