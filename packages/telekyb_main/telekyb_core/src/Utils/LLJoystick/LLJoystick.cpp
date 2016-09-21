/*
 * LLJoystick.cpp
 *
 *  Created on: Oct 29, 2011
 *      Author: mriedel
 */

#include "LLJoystick.hpp"

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/ROS.hpp>


// I hope this get's removed soon
#define PITCH_ROLL_CMD_DEG_SCALE 4.0

#define ZERO_STICK_VALUE 66.0
#define MIN_THRUST_CMD 28 // 8 + 20 (delta_trust in SaturateMotors())
#define MAX_THRUST_CMD 210 // 230 - 20 (delta_trust in SaturateMotors())


// main
int main(int argc, char **argv) {

	telekyb::TeleKyb::init(argc, argv, "LLCommands");

	telekyb::LLJoystick j;
	j.run();

	telekyb::TeleKyb::shutdown();

	return 0;
}


namespace TELEKYB_NAMESPACE {


LLJoystick::LLJoystick()
	: rollCmd(0), pitchCmd(0), yawCmd(0), thrustCmd(0)
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	joystick = nodeHandle.subscribe(options.tJoystickTopic->getValue()
				, 1, &LLJoystick::joystickCB, this);
	llCommand = nodeHandle.advertise<telekyb_msgs::TKTTCommands>(options.tLLCommandsTopic->getValue(),1);

}

LLJoystick::~LLJoystick()
{

}


void LLJoystick::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// we use the first three axes.
	if (msg->axes.size() < 4) {
		ROS_ERROR_STREAM("Discarding Joystick Input from " << options.tJoystickTopic->getValue()
				<< ". At least 4 Axes are needed. Received: " << msg->axes.size());
		return;
	}


//	decThrust = fmax( (Decimal)MIN_THRUST_CMD , fmin((Decimal)MAX_THRUST_CMD ,106.0 + 300.0*pos.z() ) );
//	decRoll  = PITCH_ROLL_CMD_DEG_SCALE*100.0*pos.x();
//	decPitch = PITCH_ROLL_CMD_DEG_SCALE*100.0*pos.y();
//	decYawRate   = 100.0*oriVel.z();

	thrustCmd = std::max( MIN_THRUST_CMD , std::min(MAX_THRUST_CMD , (int)(ZERO_STICK_VALUE + 10.0*msg->axes[3]) ) );
	rollCmd  = PITCH_ROLL_CMD_DEG_SCALE*25.0*msg->axes[0];
	pitchCmd = PITCH_ROLL_CMD_DEG_SCALE*25.0*msg->axes[1];
	yawCmd   = 100.0*msg->axes[2];

}

void LLJoystick::run()
{
	ros::Rate rate(options.tCommandRate->getValue());

	while(ros::ok()) {
		// read joystick translate into command
		telekyb_msgs::TKTTCommands msg;
		msg.header.stamp = ros::Time::now();
		msg.roll_torque = rollCmd;
		msg.pitch_torque = pitchCmd;
		msg.yaw_torque = yawCmd;
		msg.thrust = thrustCmd;

		llCommand.publish(msg);

		rate.sleep();
	}
}

}
