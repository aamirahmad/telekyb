/*
 * LLJoystick.hpp
 *
 *  Created on: Oct 29, 2011
 *      Author: mriedel
 */

#ifndef LLJOYSTICK_HPP_
#define LLJOYSTICK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <telekyb_msgs/TKTTCommands.h>

#include "LLJoystickOptions.hpp"

namespace TELEKYB_NAMESPACE {


class LLJoystick {
private:
	LLJoystickOptions options;

	ros::NodeHandle nodeHandle;
	ros::Subscriber joystick;
	ros::Publisher llCommand;

	int rollCmd;
	int pitchCmd;
	int yawCmd;
	int thrustCmd;

public:
	LLJoystick();
	virtual ~LLJoystick();

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);

	void run();
};

}

#endif /* LLJOYSTICK_HPP_ */
