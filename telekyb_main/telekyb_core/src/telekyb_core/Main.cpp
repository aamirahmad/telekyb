/*
 * Main.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <telekyb_core/TeleKybCore.hpp>

#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

// main
int main(int argc, char **argv) {

// This has to run with a few more threads
	telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","4");
	telekyb::TeleKyb::init(argc,argv,"TeleKybCore", ros::init_options::AnonymousName);

	telekyb::TeleKybCore* core = new telekyb::TeleKybCore();

	// wait for shutdown.
	ros::waitForShutdown();

	// delete
	delete core;

	telekyb::TeleKyb::shutdown();
}
