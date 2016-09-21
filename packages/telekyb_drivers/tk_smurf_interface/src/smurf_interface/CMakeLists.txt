# Smurf Interface srcs Library

set(SmurfInterfaceSRCS
	Main.cpp
	SerialIMUDevice.cpp
	SerialIMUDeviceOptions.cpp
	IMUDataProcessor.cpp
	IMUDataProcessorOptions.cpp
	IMUCalibrator.cpp
	SerialCommandDevice.cpp
	SerialCommandDeviceOptions.cpp
	SmurfInterfaceOptions.cpp
)

rosbuild_add_executable(smurf_interface ${SmurfInterfaceSRCS})
#rosbuild_link_boost(${PROJECT_NAME} filesystem regex system thread)
