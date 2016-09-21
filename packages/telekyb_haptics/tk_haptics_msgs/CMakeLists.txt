# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_haptics_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation telekyb_msgs)


#uncomment if you have defined messages
add_message_files(FILES
  TKHapticOutput.msg
)
#uncomment if you have defined services


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES geometry_msgs std_msgs telekyb_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS  message_runtime telekyb_msgs
)
