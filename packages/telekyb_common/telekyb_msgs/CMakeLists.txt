# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation  std_msgs geometry_msgs)


add_message_files(FILES
StampedPointArray.msg
TKMotorCommands.msg
Behavior.msg
MKValue.msg
TKTTCommands.msg
TKGRCommands.msg
TKSetPointCommands.msg
TKTrajectory.msg
TKCommands.msg
MKValues.msg
TKState.msg
TKLLCommands.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    CATKIN_DEPENDS  message_runtime std_msgs geometry_msgs 
)


