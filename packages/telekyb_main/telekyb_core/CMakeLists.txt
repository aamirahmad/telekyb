# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_core)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()

find_package(YamlCpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_defines telekyb_msgs tk_trajctrl tk_trajprocessor tk_behavior tk_state sensor_msgs cmake_modules dynamic_reconfigure)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()

#set(ODEINTV2_INCLUDE_DIRS $ENV{ODEINT_V2_INCLUDE_DIR})
#IF (NOT DEFINED ODEINTV2_INCLUDE_DIRS)
#        MESSAGE(FATAL_ERROR "Please point to the direction of the odeint v2 installation")
#ENDIF()

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
#	${ODEINTV2_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	src/Utils/LLJoystick/LLJoystickOptions.cpp
	src/telekyb_core/TeleKybCoreOptions.cpp
	src/telekyb_core/TeleKybCore.cpp
	src/telekyb_core/TeleKybCoreInterface.cpp
)

add_dependencies(${PROJECT_NAME}
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)


generate_dynamic_reconfigure_options(
  cfg/telekyb_core.cfg
)

if(YAMLCPP_FOUND)
	 include_directories(${YAMLCPP_INCLUDE_DIR})
ENDIF()

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
ENDIF()


add_executable(telekyb_core_node
	src/telekyb_core/Main.cpp
	src/telekyb_core/TeleKybCore.cpp
	src/telekyb_core/TeleKybCoreInterface.cpp
	src/telekyb_core/TeleKybCoreOptions.cpp
)

add_dependencies(telekyb_core_node ${PROJECT_NAME}_gencfg)

target_link_libraries(telekyb_core_node
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)


add_executable(llJoystick
	src/Utils/LLJoystick/LLJoystick.cpp
	src/Utils/LLJoystick/LLJoystickOptions.cpp
)
target_link_libraries(llJoystick
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)


catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base telekyb_defines telekyb_msgs tk_trajctrl tk_trajprocessor tk_behavior tk_state sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

