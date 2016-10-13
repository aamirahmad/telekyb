# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_base)

cmake_policy(SET CMP0022 OLD)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

#message(${CMAKE_MODULE_PATH})
#message(FATAL_ERROR $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})


find_package(catkin REQUIRED COMPONENTS roscpp telekyb_defines telekyb_msgs telekyb_srvs cmake_modules)
#set(Boost_NO_SYSTEM_PATHS true)
if (DEFINED $ENV{BOOST_ROOT})
        set(BOOST_ROOT $ENV{BOOST_ROOT})
#	set(BOOST_LIBRARYDIR ${BOOST_ROOT}/lib)
endif()
if (DEFINED $ENV{YAMLCPP_ROOT})
        set(YAMLCPP_ROOT $ENV{YAMLCPP_ROOT})
endif()
find_package(Boost QUIET REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)
#set(EIGEN3_INCLUDE_DIRS $ENV{EIGEN3_INCLUDE_DIR})
#IF (NOT DEFINED EIGEN3_INCLUDE_DIRS)
#	MESSAGE(FATAL_ERROR "Please point to the direction of the eigen3 installation")	
#ENDIF()
find_package(YamlCpp REQUIRED)

#MESSAGE(FATAL_ERROR ${PROJECT_SOURCE_DIR}/include/telekyb_base/Options/)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${prefix}/include")
SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS_INIT} -Wl,-rpath,${prefix}/lib -L${prefix}/lib -ltelekyb_base"
     CACHE STRING "Flags used by the linker during the creation of modules.") 

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	include/
	src/Base/BaseSingleton.cpp
	src/Estimators/VarianceEstimator.cpp
	src/File/File.cpp
	src/Tools/XmlRpcHelper.cpp
	src/Tools/YamlHelper.cpp
	src/Options/RawOptionsContainer.cpp
	src/Options/BaseOption.cpp
	src/Options/CommonOptions.cpp
	src/Options/OptionContainer.cpp
	include/telekyb_base/Options/BoundsOption.hpp
	src/ROS/ROSModule.cpp
	src/ROS/ROSBaseOption.cpp
	src/ROS/ROSOptionContainer.cpp
	src/ROS/ROSOptionController.cpp
	src/Time/Time.cpp
	src/Time/Timer.cpp
	src/Time/Duration.cpp
	src/Time/ThreadedCBTimer.cpp
	src/Filter/QuadrupleIntegrator.cpp
	src/Filter/FirFilter.cpp
	src/Filter/OneEuroFilter.cpp
	src/Filter/IIRFilter.cpp
	src/TeleKyb.cpp
	src/Messages/TKTrajectory.cpp
	src/Messages/TKState.cpp
	src/Spaces/Angle.cpp
)

add_dependencies(${PROJECT_NAME}
	telekyb_msgs_generate_messages_cpp
	telekyb_srvs_generate_messages_cpp
	tk_draft_msgs_generate_messages
)

target_link_libraries(${PROJECT_NAME}	
        ${catkin_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${Boost_LIBRARIES}
	${PROJECT_NAME}
)


add_executable(basetest 
	src/TestClass.cpp
)

#message(${YAMLCPP_LIBRARY})
#message(FATAL_ERROR ${catkin_LIBRARIES} ${YAMLCPP_LIBRARY} ${Boost_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(basetest ${YAMLCPP_LIBRARY} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})
#/home/paolo/catkin_ws/src/external_libraries/yaml-cpp-0.5.1/build/libyaml-cpp.a

#add_executable(testFilter 
#	src/Filter/testFilter.cpp
#)
#target_link_libraries(testFilter ${PROJECT_NAME})


catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS  telekyb_defines telekyb_msgs telekyb_srvs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


