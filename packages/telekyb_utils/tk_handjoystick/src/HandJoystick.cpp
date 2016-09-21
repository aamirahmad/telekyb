/*
 * HandJoystick.cpp
 *
 *  Created on: Jan 5, 2012
 *      Author: mriedel
 */

#include "HandJoystick.hpp"

#include <telekyb_base/ROS.hpp>

#define JOYMSG_BUTTONS 3 // all 0.
#define JOYMSG_AXES 3 // At least 3

namespace TELEKYB_NAMESPACE {

HandJoystick::HandJoystick()
	: neutralPosSet(false), neutralPosSetRequest(false)
{
	ros::NodeHandle mainNodeHandle(ROSModule::Instance().getMainNodeHandle());
	joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue()
			, 10, &HandJoystick::joystickCB, this);
	transformSub = mainNodeHandle.subscribe(options.tTransformStampedTopic->getValue()
				, 10, &HandJoystick::transformCB, this);

	pubJoy = mainNodeHandle.advertise<sensor_msgs::Joy>(options.tJoyPubName->getValue(), 10 );
	pubTransform = mainNodeHandle.advertise<geometry_msgs::TransformStamped>(options.tTransformPubName->getValue(),10);
}

HandJoystick::~HandJoystick()
{

}

void HandJoystick::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	if (msg->buttons[9]) {
		neutralPosSetRequest = true;
		ROS_INFO("Setting Neutral Position");
	}
}

void HandJoystick::transformCB(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	// BEWARE! I put in a const NED Transform here!!! Without matrix

	if (neutralPosSetRequest) {
		neutralPosition(0) = msg->transform.translation.x;
		neutralPosition(1) = msg->transform.translation.y;
		neutralPosition(2) = msg->transform.translation.z;

		neutralPosSetRequest = false;
		neutralPosSet = true;
	}

	//ROS_INFO("Transform CB");

	if (neutralPosSet) {
		sensor_msgs::Joy joyMsg;
		joyMsg.axes.resize(JOYMSG_AXES, 0.0);
		joyMsg.buttons.resize(JOYMSG_BUTTONS, 0);

		joyMsg.axes[0] = options.tAxisValueScale->getValue() * (msg->transform.translation.x - neutralPosition(0));
		joyMsg.axes[1] = -1.0 * options.tAxisValueScale->getValue() * (msg->transform.translation.y - neutralPosition(1));
		joyMsg.axes[2] = -1.0 * options.tAxisValueScale->getValue() * (msg->transform.translation.z - neutralPosition(2));

		for (unsigned int i = 0; i < joyMsg.axes.size(); ++i) {
			if (fabs(joyMsg.axes[i]) > options.tMaxAxisValue->getValue()) {
				joyMsg.axes[i] = copysign(options.tMaxAxisValue->getValue(), joyMsg.axes[i]);
			}
		}

		// Transform
		geometry_msgs::TransformStamped transformMsg(*msg);
		transformMsg.transform.translation.x = joyMsg.axes[0];
		transformMsg.transform.translation.y = joyMsg.axes[1];
		transformMsg.transform.translation.z = joyMsg.axes[2];

		transformMsg.transform.rotation.y *= -1.0;
		transformMsg.transform.rotation.z *= -1.0;

		pubJoy.publish(joyMsg);

		pubTransform.publish(transformMsg);

	}
}

} /* namespace telekyb_traj */
