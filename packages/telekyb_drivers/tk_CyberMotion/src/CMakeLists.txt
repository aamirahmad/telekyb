# CyberMotion srcs executable
set(CyberMotionSRCS
	Main.cpp
	CyberMotionxPCBridge.cpp
	CyberMotionxPCOptions.cpp
)

rosbuild_add_executable(CyberMotion ${CyberMotionSRCS})
rosbuild_link_boost(CyberMotion thread)
rosbuild_link_boost(CyberMotion system thread)
