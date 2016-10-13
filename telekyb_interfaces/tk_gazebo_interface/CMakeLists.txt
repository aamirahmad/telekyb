cmake_minimum_required(VERSION 2.8.3)
project(tk_gazebo_interface)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  telekyb_msgs
  telekyb_interface
  telekyb_base
  cmake_modules
)

find_package(mav_msgs QUIET)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




# message(FATAL_ERROR ${catkin_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MAV_MSGS_INCLUDE_DIRS}
  /home/simclient/telekyb_ws/devel/include
)


if(mav_msgs_FOUND)
	# Extend with executables for the subtasks, if necessary.
	add_executable(${PROJECT_NAME}
		src/solution.cpp
	)
	add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(${PROJECT_NAME}
		${catkin_LIBRARIES}
		${MAV_MSGS_LIBRARIES}
	)
# 	message(${PROJECT_NAME}" will be built.")
else(mav_msgs_FOUND)
	message(WARNING "Missing package 'mav_msgs' " ${PROJECT_NAME}" will NOT be built.")
endif(mav_msgs_FOUND)


catkin_package(  
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs telekyb_msgs
  )

