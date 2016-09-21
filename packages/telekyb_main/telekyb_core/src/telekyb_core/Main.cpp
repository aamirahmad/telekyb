/*
 * Main.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCore.hpp>

#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>


void callback(telekyb_core::telekyb_coreConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}


// main
int main(int argc, char **argv) {

// This has to run with a few more threads
	telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","4");
	telekyb::TeleKyb::init(argc,argv,"TeleKybCore", ros::init_options::AnonymousName);
	

	dynamic_reconfigure::Server<telekyb_core::telekyb_coreConfig> server;
	dynamic_reconfigure::Server<telekyb_core::telekyb_coreConfig>::CallbackType f;
	
	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);	

	telekyb::TeleKybCore* core = new telekyb::TeleKybCore();

	// wait for shutdown.
	ros::waitForShutdown();

	// delete
	delete core;

	telekyb::TeleKyb::shutdown();
}
