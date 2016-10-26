/*
 * Main.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCore.hpp>
#include <telekyb_core/UAVcontrolParametersConfig.h>

#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

// main

float positionControl_tDerivGain;

void reconf_callback(TeleKybCore::UAVcontrolParametersConfig &config, uint32_t level)
{
    positionControl_tDerivGain = config.positionControl_tDerivGain;
}

int main(int argc, char **argv) {

// This has to run with a few more threads
	telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","4");
	telekyb::TeleKyb::init(argc,argv,"TeleKybCore", ros::init_options::AnonymousName);

	telekyb::TeleKybCore* core = new telekyb::TeleKybCore();
	
	dynamic_reconfigure::Server<TeleKybCore::UAVcontrolParametersConfig> server;
	dynamic_reconfigure::Server<TeleKybCore::UAVcontrolParametersConfig>::CallbackType f;
	f = boost::bind(&reconf_callback, _1, _2);
	server.setCallback(f);	

	// wait for shutdown.
	ros::waitForShutdown();

	// delete
	delete core;

	telekyb::TeleKyb::shutdown();
}
