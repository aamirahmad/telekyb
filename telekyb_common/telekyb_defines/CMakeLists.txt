# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(telekyb_defines)


find_package(catkin REQUIRED COMPONENTS roscpp)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(
        include
        ${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	src/telekyb_defines/MKDefines.cpp
)

add_dependencies(${PROJECT_NAME}
telekyb_msgs_generate_messages_cpp
telekyb_srvs_generate_messages_cpp
tk_draft_msgs_generate_messages_cpp
)

if(YAMLCPP_FOUND)
	 include_directories(${YAMLCPP_INCLUDE_DIR})
ENDIF()

if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
ENDIF()

add_executable(test_enum src/enum/enum.cpp)

target_link_libraries(test_enum
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)


catkin_package(
    DEPENDS roscpp 
    CATKIN_DEPENDS  
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

