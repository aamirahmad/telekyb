cmake_minimum_required(VERSION 2.4.6)
project(tk_formation)

list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})

find_package(catkin REQUIRED COMPONENTS roscpp telekyb_base telekyb_interface tk_behavior tk_obstacle telekyb_defines telekyb_msgs tk_haptics_base cmake_modules)
find_package(Boost REQUIRED COMPONENTS system thread)



   


list(APPEND CMAKE_MODULE_PATH $ENV{TELEKYB_CMAKE_SCRIPTS_DIR})
find_package(YamlCpp REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}
	${EIGEN_INCLUDE_DIRS}
	${YAMLCPP_INCLUDE_DIR}
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${ODEINTV2_INCLUDE_DIRS}
)


set(BehaviorSRCS
	src/behaviors/AbstractGraphNode.cpp
	src/behaviors/Neighbor.cpp
	src/behaviors/Formation.cpp
	src/behaviors/FormationReconfiguration.cpp
)
add_library(tk_form_behaviors ${BehaviorSRCS})
add_dependencies(tk_form_behaviors
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(tk_form_behaviors
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)




set(FormationCtrlSRC
	src/control/Main.cpp
	src/control/FormationControl.cpp
	src/control/FormationControlElement.cpp
)
add_executable(tk_formationctrl ${FormationCtrlSRC})
add_dependencies(tk_formationctrl
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(tk_formationctrl
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)





set(HapticControllerSRC
	src/haptic_controllers/HapticFormationController.cpp
)
add_library(haptic_controllers ${HapticControllerSRC})
add_dependencies(haptic_controllers
        telekyb_msgs_generate_messages_cpp
        telekyb_srvs_generate_messages_cpp
        tk_draft_msgs_generate_messages_cpp
)

target_link_libraries(haptic_controllers
        ${Boost_LIBRARIES}
        ${YAMLCPP_LIBRARY}
        ${catkin_LIBRARIES}
)




catkin_package(
    DEPENDS roscpp Boost
    CATKIN_DEPENDS telekyb_base telekyb_interface tk_behavior tk_obstacle telekyb_defines telekyb_msgs tk_haptics_base
    INCLUDE_DIRS src
    LIBRARIES tk_form_behaviors haptic_controllers
)