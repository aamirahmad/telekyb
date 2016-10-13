cmake_minimum_required(VERSION 2.8.3)
project(telekyb_gazebo)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  rosbag
  gazebo_msgs
  tf_conversions
  telekyb_msgs
  telekyb_srvs
)

find_package(mav_msgs QUIET)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(mav_msgs_FOUND)
  add_executable(qc13GazeboInterface src/qc13GazeboInterface.cpp)
  target_link_libraries(qc13GazeboInterface ${catkin_LIBRARIES})

  add_dependencies(qc13GazeboInterface
	  telekyb_msgs_generate_messages_cpp
	  telekyb_srvs_generate_messages_cpp
	  tk_draft_msgs_generate_messages_cpp
  )

  add_executable(qcfireflyGazeboInterface src/qcfireflyGazeboInterface.cpp)
  target_link_libraries(qcfireflyGazeboInterface ${catkin_LIBRARIES})
  
  add_dependencies(qcfireflyGazeboInterface
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
  )
else(mav_msgs_FOUND)
  message(WARNING "Missing package 'mav_msgs', this package will not be built!")
endif(mav_msgs_FOUND)




catkin_package(  
  CATKIN_DEPENDS 
    geometry_msgs 
    mav_msgs
    roscpp 
    sensor_msgs 
    rosbag
    gazebo_msgs
    tf_conversions
    telekyb_msgs
    telekyb_srvs
  DEPENDS Eigen
)
