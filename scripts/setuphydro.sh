#!/bin/bash

default=y
read -p "Do you want me to install some prerequisites from the ros repository? (sudo apt-get install python-wstool) [$default] " INSTALLPREREQ
INSTALLPREREQ=${INSTALLPREREQ:-$default}

if [ "$INSTALLPREREQ" == "y" ]; then
  echo "sudo apt-get install python-wstool"
  sudo apt-get install python-wstool
fi

default=y
read -p "Do you want me to check out the humus-telekyb/hydro repository ? [$default] " CHECKOUTREP
CHECKOUTREP=${CHECKOUTREP:-$default}

ROS_PACKAGE_PATH_TK=""
TELEKYB_CMAKE_SCRIPTS_DIR=""

if [ "$CHECKOUTREP" == "y" ]; then
  default=$HOME/workspace/telekyb_ws/src
  read -p "Enter base path where to check out the repository [$default]? " REPPATH
  REPPATH=${REPPATH:-$default}
  if [ ! -d "$REPPATH" ]; then
    echo "$REPPATH does not exist"
    exit 1
  fi
  cd $REPPATH && svn checkout https://svn.tuebingen.mpg.de/humus-telekyb/hydro/trunk && cd -
  ROS_PACKAGE_PATH_TK=$REPPATH/humus-telekyb/hydro/trunk/packages
  TELEKYB_CMAKE_SCRIPTS_DIR=$REPPATH/humus-telekyb/hydro/trunk/cmake_scripts
else
  default=$HOME/humus-telekyb/hydro/trunk/packages
  read -p "Enter the path to the telekyb packages [$default]: " ROS_PACKAGE_PATH_TK
  ROS_PACKAGE_PATH_TK=${ROS_PACKAGE_PATH_TK:-$default}
  
  default=`dirname $ROS_PACKAGE_PATH_TK`/cmake_scripts
  read -p "Enter the path to the telekyb cmake scripts [$default]: " TELEKYB_CMAKE_SCRIPTS_DIR
  TELEKYB_CMAKE_SCRIPTS_DIR=${TELEKYB_CMAKE_SCRIPTS_DIR:-$default}

  if [ ! -d "$ROS_PACKAGE_PATH_TK" ]; then
    echo "$ROS_PACKAGE_PATH_TK does not exist"
    exit 1
  fi
  if [ ! -d "$TELEKYB_CMAKE_SCRIPTS_DIR" ]; then
    echo "$TELEKYB_CMAKE_SCRIPTS_DIR does not exist"
    exit 1
  fi

fi

ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH_TK:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH
export TELEKYB_CMAKE_SCRIPTS_DIR

echo "ROS_PACKAGE_PATH is $ROS_PACKAGE_PATH"
echo "TELEKYB_CMAKE_SCRIPTS_DIR is $TELEKYB_CMAKE_SCRIPTS_DIR"

default=y
read -p "Do you want me to include the path definition in your ~/.bashrc file ? [$default] " UPDATEBASHRC
UPDATEBASHRC=${UPDATEBASHRC:-$default}

if [ "$UPDATEBASHRC" == "y" ]; then
  echo "" >> ~/.bashrc
  echo "# The following has been automatically added by setuphydro.sh" >> ~/.bashrc
  echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH" >> ~/.bashrc
  echo "export TELEKYB_CMAKE_SCRIPTS_DIR=$TELEKYB_CMAKE_SCRIPTS_DIR" >> ~/.bashrc
fi

default=y
read -p "Do you want me to create a catkin workspace for you? [$default] " CREATECATKINWS
CREATECATKINWS=${CREATECATKINWS:-$default}

if [ "$CREATECATKINWS" == "y" ]; then
  default=$HOME/telekyb_catkin_ws
  read -p "Enter the path to the telekyb catkin workspace [$default]: " CATKINWSPATH
  CATKINWSPATH=${CATKINWSPATH:-$default}
  if [ -d "$CATKINWSPATH" ]; then
    echo "$CATKINWSPATH already exists, to protect data I will exit now"
    exit 1
  fi
  mkdir -p $CATKINWSPATH/src && cd $CATKINWSPATH/src && catkin_init_workspace
  echo "I will now overlay the essential telekyb packages"
  cd $CATKINWSPATH/src && wstool init && cd -
  for package in telekyb_algo telekyb_common telekyb_haptics telekyb_main telekyb_utils; do
    echo $package
    cd $CATKINWSPATH/src && wstool set $package --svn https://svn.tuebingen.mpg.de/humus-telekyb/hydro/trunk/packages/$package && cd -
  done
  cd $CATKINWSPATH/src && wstool update && cd -
  cd $CATKINWSPATH && catkin_make
fi

echo "#########################################################################"
echo "#########################################################################"
echo "## PLEASE CLOSE THIS SHELL FOR THE CHANGES IN ~/.bashrc TO TAKE EFFECT ##"
if [ "$CREATECATKINWS" == "y" ]; then
  echo "##        Your catkin workspace can be found in $CATKINWSPATH          ##"
fi
echo "#########################################################################"
echo "#########################################################################"
