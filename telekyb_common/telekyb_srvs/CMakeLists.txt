# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation telekyb_msgs)


add_service_files(FILES
	BoolInput.srv
	BehaviorOutput.srv
	MKValueInputOutput.srv
	StringInput.srv
	StringArrayOutput.srv
	GetOptionMapping.srv
	DHDToggleForce.srv
	BehaviorInputOutput.srv
	BehaviorInput.srv
	StringOutput.srv
	IntArrayInput.srv
	BinOccupancySrv.srv
)

generate_messages(DEPENDENCIES telekyb_msgs)

catkin_package(
    CATKIN_DEPENDS  message_runtime telekyb_msgs message_generation
)


