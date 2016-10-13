#!/bin/sh

# setup environment variables

#
#       telekyb 3 workspace variables
#
# the directory of the setup scipt
# it must be in root directory of the project
export TELEKYB_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/.."
#-- export TELEKYB3_CMAKE_SCRIPTS_DIR=$TELEKYB_ROOT_DIR/cmake_scripts
export TELEKYB_CMAKE_SCRIPTS_DIR=$TELEKYB_ROOT_DIR/cmake_scripts
export EIGEN3_INCLUDE_DIR=/usr/include/eigen3
# export ViconDataStreamSDKCPP_ROOT=/

#-- export ODEINT_V2_INCLUDE_DIR=$TELEKYB_ROOT_DIR/external_libraries/odeintv2/headmyshoulder-odeint-v2-5eebbb5/include

#
#	find setup scripts in devel folder 
#
# echo 'source devel/setup.bash files'
#-- for file in $(find $TELEKYB_ROOT_DIR -regextype sed -regex ".*/devel/setup.bash");
#-- do
#-- 	echo 'source setup file' $file;
#-- 	source $file;
#-- done
#-- 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$TELEKYB_ROOT_DIR
