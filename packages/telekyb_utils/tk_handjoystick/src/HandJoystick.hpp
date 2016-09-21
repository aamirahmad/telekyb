/*
 * HandJoystick.hpp
 *
 *  Created on: Jan 5, 2012
 *      Author: mriedel
 */

#ifndef HANDJOYSTICK_HPP_
#define HANDJOYSTICK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include "HandJoystickOptions.hpp"

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TransformStamped.h>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

class HandJoystick {
protected:
	HandJoystickOptions options;

	ros::Subscriber joySub;
	ros::Subscriber transformSub;

	// Publish Joy
	ros::Publisher pubJoy;
	// Publish StampedTransform
	ros::Publisher pubTransform;

	bool neutralPosSet;
	bool neutralPosSetRequest;
	Position3D neutralPosition;

public:
	HandJoystick();
	virtual ~HandJoystick();

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);
	void transformCB(const geometry_msgs::TransformStamped::ConstPtr& msg);
};

} /* namespace telekyb */
#endif /* HANDJOYSTICK_HPP_ */
