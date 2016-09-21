/**
 * Main.cpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Sep 29, 2014
 *      Author: Johannes Lächele
 *  
 */


#include <telekyb_base/TeleKyb.hpp>
#include <ros/ros.h>

#include "TeleopExperiment.hpp"

int main(int argc, char **argv) {

	telekyb::TeleKyb::init(argc,argv, "tk_exp_joystick");

	TeleopExperiment* e = new TeleopExperiment();

	// spin here
	ros::waitForShutdown();

	delete e;

	telekyb::TeleKyb::shutdown();
}

