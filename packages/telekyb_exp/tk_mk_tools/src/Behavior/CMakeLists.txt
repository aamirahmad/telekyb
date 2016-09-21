# Behavior Plugins

set(BehaviorsSRCS
	Calibrator.cpp
)

rosbuild_add_library(be_mk_tools ${BehaviorsSRCS})
#target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME})
