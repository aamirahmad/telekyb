/*
 * Main.cpp
 *
 *  Created on: Jan 5, 2012
 *      Author: mriedel
 */


#include <telekyb_base/TeleKyb.hpp>

#include "HandJoystick.hpp"

#include <ros/ros.h>

using namespace telekyb;

int main(int argc, char **argv) {
	TeleKyb::init(argc, argv, "tk_handjoystick");

	HandJoystick* j = new HandJoystick;

	ros::waitForShutdown();

	delete j;

	TeleKyb::shutdown();
}


