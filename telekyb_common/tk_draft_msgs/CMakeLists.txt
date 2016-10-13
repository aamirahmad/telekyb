# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_draft_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation  std_msgs geometry_msgs telekyb_msgs)


add_message_files(FILES
	TKBLCommands.msg
	TKSpline.msg
	TKSphere.msg
	SMURFIntegTerms.msg
	SegmentDefinition.msg
	ClosestPointHit.msg
	TKSmallImu.msg
	TKSmallImuAng.msg
	ArdroneNavdata.msg
	proc_data.msg
	AdjacencyMatrix.msg
	MassStamped.msg
	TKLines.msg
	SegmentObstacles.msg
	ForceTorque.msg
	TKSmoothTraj.msg
	LeapFingerTip.msg
	PvtolArmState.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs telekyb_msgs)

catkin_package(
    CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs telekyb_msgs
)


