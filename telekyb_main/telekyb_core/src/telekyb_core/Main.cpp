/*
 * Main.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCore.hpp>
#include <telekyb_core/UAVcontrolParametersConfig.h>
#include <dynamic_reconfigure/server.h>

#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

// main

double positionControl_tDerivGain;

void reconf_callback(telekyb_core::UAVcontrolParametersConfig &config, uint32_t level)
{
   
}

int main(int argc, char **argv) {

// This has to run with a few more threads
	telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","4");
	telekyb::TeleKyb::init(argc,argv,"TeleKybCore", ros::init_options::AnonymousName);

	telekyb::TeleKybCore* core = new telekyb::TeleKybCore();
	
	dynamic_reconfigure::Server<telekyb_core::UAVcontrolParametersConfig> server;
	dynamic_reconfigure::Server<telekyb_core::UAVcontrolParametersConfig>::CallbackType f;
	f = boost::bind(&reconf_callback, _1, _2);
	server.setCallback(f);	

	// wait for shutdown.
	ros::waitForShutdown();

	// delete
	delete core;

	telekyb::TeleKyb::shutdown();
}
