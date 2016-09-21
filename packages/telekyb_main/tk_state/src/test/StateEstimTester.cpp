/*
 * BehaviorTester.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_base/TeleKyb.hpp>

int main(int argc, char **argv) {
	telekyb::TeleKyb::init(argc,argv, "StateEstimTester");

	telekyb::StateEstimatorController::Instance();

	ros::waitForShutdown();

	telekyb::TeleKyb::shutdown();
}


