/*
 * mk_test1.cpp
 *
 *  Created on: Nov 25, 2011
 *      Author: mriedel
 */

#include <tk_mkinterface_outdoor/MKInterfaceConnection.hpp>

#include <tk_mkinterface_outdoor/MKData.hpp>

#include <telekyb_base/TeleKyb.hpp>

#include <ros/ros.h>

using namespace telekyb;



int main(int argc, char **argv) {


	TeleKyb::init(argc,argv,"mk_test3");

#ifdef __APPLE__
	MKInterfaceConnection* c = new MKInterfaceConnection("/dev/tty.usbserial-A600eG5l");
#else
	MKInterfaceConnection* c = new MKInterfaceConnection("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A600eG5l-if00-port0");
#endif
	if (c->isOpen()) {
		c->updateValue(MKDataDefines::FIRMWARE_REVISION);

		MKValue* robotID = c->getMKDataRef().getValueByID(MKDataDefines::ROBOT_ID);
		ROS_INFO("PRE RobotID: %d", robotID->getValue());
		ROS_INFO_STREAM("PRE Time: " << robotID->getStamp().dateTimeToString());

		c->updateValue(MKDataDefines::ROBOT_ID);

		ROS_INFO("Post RobotID: %d", robotID->getValue());
		ROS_INFO_STREAM("POST Time: " << robotID->getStamp().dateTimeToString());


		MKValue* mirrorActive = c->getMKDataRef().getValueByID(MKDataDefines::MIRROR_DATA_ACTIVE);


		c->updateValue(mirrorActive->getID());
		ROS_INFO("MirrorActive: %d", mirrorActive->getValue());

		// set Init!
		if ( c->setValue(mirrorActive->getMKSingleValuePacketWithValue(1)) ) {
			ROS_INFO("Set Successful!");
		}

		sleep(5);
		MKActiveIDs pattern = c->getMKDataRef().getPattern(MKDataPattern::RawImuAttEst);
		c->setActiveDataIDs(pattern);


//		MotorState state(motorState->getValue());
//		ROS_INFO("Current Motorstate: %s, %d", state.str(), state.value());
//
		sleep(5);
		MKValue* motorState = c->getMKDataRef().getValueByID(MKDataDefines::MOTOR_STATE);
		c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Init));

		sleep(5);
		c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Off));



	}

	ros::waitForShutdown();

	delete c;

	TeleKyb::shutdown();

	return EXIT_SUCCESS;
}




