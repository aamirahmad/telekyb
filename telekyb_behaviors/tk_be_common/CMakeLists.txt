# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_be_common)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_behavior telekyb_interface cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(TELEKYB_EXTERNAL_LIBS $ENV{TELEKYB_EXTERNAL_LIBS})
# build the mpc library
#include(ExternalProject)
#ExternalProject_Add(cmpc_library
#  PREFIX ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  DOWNLOAD_COMMAND ""
#  TMP_DIR      ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  STAMP_DIR    ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  DOWNLOAD_DIR ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  SOURCE_DIR   ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  CONFIGURE_COMMAND ""
#  INSTALL_DIR  ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc
#  INSTALL_COMMAND ""
#  BUILD_IN_SOURCE 1
#  BUILD_COMMAND make -j4
#)

include_directories(include
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIR}
        ${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${TELEKYB_EXTERNAL_LIBS}
)

#ExternalProject_Get_Property(cmpc_library install_dir)
SET(MUAOMPC_STATIC_LIB ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc/libcmpc.a)
if(EXISTS ${MUAOMPC_STATIC_LIB})
	add_library(cmpc_lib STATIC IMPORTED)
	set_property(TARGET cmpc_lib PROPERTY IMPORTED_LOCATION ${TELEKYB_EXTERNAL_LIBS}/muaoMPC/cmpc/libcmpc.a)
	add_dependencies(cmpc_lib cmpc_library)


	# build the pathfollow library seperatly 
	# so the c lib is not confused with others
	add_library(tk_be_common_mpcPathFollow
		src/mpcPathFollow.cpp
	)

	add_dependencies(tk_be_common_mpcPathFollow
	        telekyb_msgs_generate_messages_cpp
	        telekyb_srvs_generate_messages_cpp
        	tk_draft_msgs_generate_messages_cpp
	)

	target_link_libraries(tk_be_common_mpcPathFollow
        	${Boost_LIBRARIES}
	        ${YAMLCPP_LIBRARY}
        	${catkin_LIBRARIES}
		cmpc_lib
	)
else()
	MESSAGE(AUTHOR_WARNING "mpcPathfollow will not be build, the needed libmpc.a is missing in the external libraries folder. 
This will cause the mpcPathfollow behavior fail to load. If you need the behavior make sure that the corresponding library is build.")
endif()


add_library(${PROJECT_NAME}
	src/DynamicFlyTo.cpp
#	src/FiltFlyTo.cpp
	src/HumanOperator.cpp
	src/Joystick.cpp
	src/LinearFlyTo.cpp
	src/SmoothLinearFlyTo.cpp
	src/PowerFourLinearTrajectoryFlyTo.cpp
	src/Omega6Joy.cpp
	src/RoboCentricOmega6Joy.cpp
#	src/RCJoy.cpp
	src/TrajectoryForceInput.cpp
	src/TrajectoryInput.cpp
	src/TrajPlayback.cpp
	src/ViconFreeLand.cpp
	src/FixedPointHover.cpp
	src/Teleoperation.cpp
# 	src/EmergencyEscape.cpp
# 	src/EmergencyEscape_Calibrate.cpp
)
#MESSAGE(FATAL_ERROR "${CATKIN_DEVEL_PREFIX}")

#set_property(TARGET mpcFollowersoimported PROPERTY IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libtk_be_common_mpcPathFollow.so)

# add_library(tk_be_common_emergencyescape
# 		src/EmergencyEscape/Reward.cpp
# 		src/EmergencyEscape/Controller.cpp
# 		src/EmergencyEscape/LinearModel.cpp
# 		src/EmergencyEscape/state.cpp
# 		src/EmergencyEscape/auxiliary.cpp
# )
# 
# target_link_libraries(tk_be_common_emergencyescape
# 	${roscpp_LIBRARIES})

add_dependencies(${PROJECT_NAME}
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages_cpp
)

#MESSAGE(FATAL_ERROR "${catkin_LIBRARIES}")

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
#         tk_be_common_emergencyescape
)




catkin_package(
    DEPENDS roscpp YamlCpp Boost eigen3
    CATKIN_DEPENDS  telekyb_base telekyb_defines telekyb_msgs tk_behavior telekyb_interface
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


