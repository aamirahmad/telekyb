# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_exp_euroc)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs telekyb_base telekyb_interface cmake_modules  	)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

find_package(octomap QUIET)
find_package(ompl QUIET)
find_package(visualization_msgs)


find_package(mav_msgs QUIET)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



#message(FATAL_ERROR ${YAMLCPP_INCLUDE_DIR})

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	${OMPL_INCLUDE_DIRS}
	${VISUALIZATION_MSGS_INCLUDE_DIRS}
)

# exp for task 3
add_executable(exp_t3_1
	src/t3_1/Main.cpp
	src/t3_1/Experiment.cpp
)

add_dependencies(exp_t3_1
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(exp_t3_1
        #${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)





if(mav_msgs_FOUND)
# exp for task 4
	add_executable(exp_t4
		src/t4/Main.cpp
		src/t4/Experiment.cpp
	)
	
	add_dependencies(exp_t4
	        telekyb_msgs_generate_messages_cpp
	        telekyb_srvs_generate_messages_cpp
	        tk_draft_msgs_generate_messages_cpp
	)
	
	target_link_libraries(exp_t4
	        #${Boost_LIBRARIES}
	        ${YAMLCPP_LIBRARY}
	        ${catkin_LIBRARIES}
	)

	if(octomap_FOUND AND ompl_FOUND AND visualization_msgs_FOUND AND false)
		# exp for task 4_3
		add_executable(exp_t4_3
			src/t4_3/Main.cpp
			src/t4_3/Experiment.cpp
		)
	
		add_dependencies(exp_t4_3
		        telekyb_msgs_generate_messages_cpp
		        telekyb_srvs_generate_messages_cpp
		        tk_draft_msgs_generate_messages_cpp
		       # path_planner_astar_generate_messages_cpp
		)
		target_link_libraries(exp_t4_3
		       #${Boost_LIBRARIES}
		        ${YAMLCPP_LIBRARY}
		        ${catkin_LIBRARIES}
			${OCTOMAP_LIBRARIES}
			${OMPL_LIBRARIES}
			${VISUALIZATION_MSGS_LIBRARIES}
		)
	else(octomap_FOUND AND ompl_FOUND AND visualization_msgs_FOUND AND false)
# 		message(WARNING "Missing package 'octomap' and/or 'ompl', experiment exp_t4_3 for Euroc and simulation with gazebo will not be built.")
	endif(octomap_FOUND AND ompl_FOUND AND visualization_msgs_FOUND AND false)


else(mav_msgs_FOUND)
	message(WARNING "Missing package 'mav_msgs', exp_t4 will NOT be built.")
endif(mav_msgs_FOUND)









catkin_package(
    DEPENDS roscpp sensor_msgs
    CATKIN_DEPENDS telekyb_base telekyb_interface
    INCLUDE_DIRS
    LIBRARIES 
)


