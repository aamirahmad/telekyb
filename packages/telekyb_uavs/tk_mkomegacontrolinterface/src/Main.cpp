/*
 * Main.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */


// Main Function for mk_interface

#include <telekyb_base/TeleKyb.hpp>

#include <tk_mkomegacontrolinterface/MKOmegaControlInterface.hpp>
#include <tk_mkomegacontrolinterface/MKOmegaControlROSInterface.hpp>

#include <ros/ros.h>

using namespace telekyb;

int main(int argc, char **argv) {

  // Receiving is threaded out by itself.
  telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","2");
  TeleKyb::init(argc,argv,"MKInterface", ros::init_options::AnonymousName);

  MKOmegaControlInterface* mkIF = new MKOmegaControlInterface();

  
  if (mkIF->hasConnection()) {
    sleep(1);
//     mkIF->getConnection()->beep((uint16_t){880});
    ROS_INFO("===========READY TO GO===============");
    
//     sleep(1);
//     mkIF->getConnection()->turnMotorOn();

    
//     if (motorstate){
//     mkIF->getConnection()->turnMotorOn();
//     std::cout<<"turn me on"<<std::endl;
//     }
    
    
//     sleep(1);
//     mkIF->getConnection()->turnMotorOff();
    
    ros::spin();
    // perhaps we switch to waitfor...
// 	  ros::waitForShutdown();
  } else {
    ROS_ERROR("Could not find UAV with matching tUavID and tUavFirmware");
  }

  delete mkIF;

  TeleKyb::shutdown();
  return 0;
}

