/*
 * BehaviorTester.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <tk_behavior/BehaviorController.hpp>
#include <telekyb_base/TeleKyb.hpp>

#include <ros/ros.h>

int main(int argc, char **argv) {
	telekyb::TeleKyb::init(argc,argv, "BehaviorTester");

	telekyb::BehaviorController::Instance();

	ros::waitForShutdown();

	telekyb::TeleKyb::shutdown();
}


