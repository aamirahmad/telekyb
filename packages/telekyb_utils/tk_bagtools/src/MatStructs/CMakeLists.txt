# Mat data structures

set(MatDataStructuresSRCS
	MatStruct.cpp
	MatTKStateStruct.cpp
	MatTKTrajectoryStruct.cpp
	MatPoseStampedStruct.cpp
	MatMassStampedStruct.cpp
	MatTKSmallImuStruct.cpp
	MatTKTTCommandsStruct.cpp
	MatTKMotorCommandsStruct.cpp
	)

rosbuild_add_library(mat_data_structures ${MatDataStructuresSRCS})
