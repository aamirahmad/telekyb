/*
 * Main.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: mriedel
 */


// Main Function for mk_interface

#include <telekyb_base/TeleKyb.hpp>

#include <tk_mkinterface/MKInterface.hpp>

#include <ros/ros.h>

using namespace telekyb;


int main(int argc, char **argv) {

	// Receiving is threaded out by itself.
	telekyb::RawOptionsContainer::addOption("tRosNrSpinnerThreads","2");
	TeleKyb::init(argc,argv,"MKInterface", ros::init_options::AnonymousName);

	MKInterface* mkIF = new MKInterface();

	if (mkIF->hasConnection()) {
//		MKInterfaceConnection* c = mkIF->getConnection();
//		const MKData& data = c->getMKDataRef();
//		sleep(5);
//
//		MKValue* motorState = data.MotState;
//		c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Init));
//
//		sleep(5);
//		c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::On));
//
//		ros::waitForShutdown();
//
//		c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Off));


		// spin here // in addition to the 1 AsyncSpinner.
		//ros::spin();
		// perhaps we switch to waitfor...
		ros::waitForShutdown();
	} else {
		ROS_ERROR("Could not find UAV with matching tUavID and tUavFirmware");
	}

	delete mkIF;

	TeleKyb::shutdown();
	return 0;
}

