/*
 * Main.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: Johannes Lächele
 *  
 */

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Options.hpp>

#include "CyberMotionxPCBridge.hpp"

using namespace telekyb;

int main(int argc, char **argv) {

	// Receiving is threaded out by itself.
	RawOptionsContainer::addOption("tRosNrSpinnerThreads","2");
	TeleKyb::init(argc,argv,"CyberMotion");

	CyberMotionxPCBridge KUKA;
	KUKA.init();

	ros::waitForShutdown();

	TeleKyb::shutdown();
	return 0;
}
