# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/g:qroovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(tk_mkinterface)

cmake_policy(SET CMP0022 OLD)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_serial telekyb_msgs tk_draft_msgs telekyb_defines cmake_modules)
find_package(Boost QUIET REQUIRED COMPONENTS filesystem regex system thread)
find_package(YamlCpp REQUIRED)
find_package(Eigen REQUIRED)

#message(FATAL_ERROR ${PROJECT_SOURCE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
        ${EIGEN_INCLUDE_DIR}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)



# tk_mkinterface Library
add_library(${PROJECT_NAME}
	src/tk_mkinterface/MKData.cpp
	src/tk_mkinterface/MKValue.cpp
	src/tk_mkinterface/MKInterface.cpp
	src/tk_mkinterface/MKInterfaceOptions.cpp
	src/tk_mkinterface/MKInterfaceConnection.cpp
	src/tk_mkinterface/MKInterfaceConnectionOptions.cpp
	src/tk_mkinterface/MKROSInterface.cpp
	src/tk_mkinterface/MKROSInterfaceOptions.cpp
	src/tk_mkinterface/MKSafeMod.cpp
	src/tk_mkinterface/MKCalibrator.cpp
)

add_dependencies(${PROJECT_NAME}
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
)

#MESSAGE(FATAL_ERROR ${catkin_LIBRARIES})

# MKInterface
add_executable(mkinterface
	src/Main.cpp
)
target_link_libraries(mkinterface
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
telekyb_serial
tk_mkinterface
telekyb_defines
)


#message(FATAL_ERROR ${catkin_LIBRARIES})
# add tests
add_executable(mk_test1
	src/mk_tests/mk_test1.cpp
)
target_link_libraries(mk_test1
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
telekyb_serial
tk_mkinterface
telekyb_defines
)

add_executable(mk_test2
	src/mk_tests/mk_test2.cpp
)
target_link_libraries(mk_test2
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
telekyb_serial
tk_mkinterface
telekyb_defines
)

add_executable(mk_test3
	src/mk_tests/mk_test3.cpp
)
target_link_libraries(mk_test3
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
	${PROJECT_NAME}
telekyb_serial
tk_mkinterface
telekyb_defines
)




catkin_package(
    DEPENDS roscpp
    CATKIN_DEPENDS telekyb_base telekyb_serial telekyb_msgs tk_draft_msgs telekyb_defines
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)


