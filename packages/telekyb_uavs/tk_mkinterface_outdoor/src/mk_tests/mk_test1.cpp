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


	TeleKyb::init(argc,argv,"mk_test1");

#ifdef __APPLE__
	MKInterfaceConnection* c = new MKInterfaceConnection("/dev/tty.usbserial-A600eG5l");
#else
	MKInterfaceConnection* c = new MKInterfaceConnection("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A600eG5l-if00-port0");
#endif
	if (c->isOpen()) {


		c->updateValue(MKDataDefines::FIRMWARE_REVISION);

		MKValue* mkRobotID = c->getMKDataRef().getValueByID(MKDataDefines::ROBOT_ID);
		ROS_INFO("PRE RobotID: %d", mkRobotID->getValue());
		ROS_INFO_STREAM("PRE Time: " << mkRobotID->getStamp().dateTimeToString());

		c->updateValue(MKDataDefines::ROBOT_ID);

		ROS_INFO("Post RobotID: %d", mkRobotID->getValue());
		ROS_INFO_STREAM("POST Time: " << mkRobotID->getStamp().dateTimeToString());


		MKValue* motorState = c->getMKDataRef().getValueByID(MKDataDefines::MOTOR_STATE);
		c->updateValue(MKDataDefines::MOTOR_STATE);


		ROS_INFO("MotorState: %d", motorState->getValue());

		MotorState state(motorState->getValue());
		ROS_INFO("Current Motorstate: %s, %d", state.str(), state.value());

		// set Init!
		//motorState->setValue(MotorState::Init);
		//MKSingleValuePacket motorState(MKDataDefines::MOTOR_STATE, MotorState::Init);
		if ( c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Init)) ) {
			ROS_INFO("Set Successful!");
		}

		// update again
		c->updateValue(MKDataDefines::MOTOR_STATE);

		state = (MotorState)motorState->getValue();
		ROS_INFO("Current Motorstate: %s, %d", state.str(), state.value());

		sleep(5);

		if ( c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::On)) ) {
			ROS_INFO("Set Successful!");
		}

		sleep(10);

		// set back
		if ( c->setValue(motorState->getMKSingleValuePacketWithValue(MotorState::Off)) ) {
			ROS_INFO("Set Successful!");
		}

		// update again
		c->updateValue(MKDataDefines::MOTOR_STATE);

		state = (MotorState)motorState->getValue();
		ROS_INFO("Current Motorstate: %s, %d", state.str(), state.value());

	}

	delete c;

	TeleKyb::shutdown();

	return EXIT_SUCCESS;
}




