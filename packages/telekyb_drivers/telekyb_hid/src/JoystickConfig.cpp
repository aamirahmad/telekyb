/*
 * JoystickConfig.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#include <telekyb_hid/JoystickConfig.hpp>

#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

JoystickConfig::JoystickConfig()
	: axisCutOff(0.0)
{

}

JoystickConfig::~JoystickConfig()
{

}

void JoystickConfig::setVectors(const std::vector<unsigned char>& byteArray_,
		const std::vector<JoyAxis>& axes_, const std::vector<JoyButton>& buttons_)
{
	byteArray = byteArray_;
	axes = axes_;
	buttons = buttons_;
}

void JoystickConfig::setModifiers(const std::vector<int>& buttonRemapping_,
		const std::vector<int>& axisRemapping_, const std::vector<double>& axisMultiplier_)
{
	buttonRemapping = buttonRemapping_;

	unsigned int old_button_size = buttonRemapping.size();
	if (old_button_size > buttons.size()) {
		ROS_ERROR("Buttonsremapping Array cannot be bigger then the number of buttons!");
	}

	buttonRemapping.resize(buttons.size());
	for (unsigned int i = 0; i < buttons.size(); i++) {
		// Extend
		if (i >= old_button_size) {
			buttonRemapping[i] = i;
		}


		if (buttonRemapping[i] < 0 || buttonRemapping[i] > (signed)buttons.size()-1) {
			ROS_ERROR("Wrong button mapping for button %d, value must be between 0 and %d "
					,i, (signed)buttons.size()-1);
			ROS_ERROR("Setting identity for button %d", i);
			buttonRemapping[i] = i;
		}
	}


	axisRemapping = axisRemapping_;
	unsigned int old_axesRe_size = axisRemapping.size();
	if (old_axesRe_size > axes.size()) {
		ROS_ERROR("Axesremapping Array cannot be bigger then the number of Axes!");
	}

	axisMultiplier = axisMultiplier_;
	unsigned int old_axesMu_size = axisMultiplier.size();
	if (old_axesMu_size > axes.size()) {
		ROS_WARN("Axesmultiplier Array cannot be bigger then the number of Axes!");
	}

	axisRemapping.resize(axes.size());
	axisMultiplier.resize(axes.size());
	for (unsigned int i = 0; i < axes.size(); ++i) {
		// Extend
		if (i >= old_axesRe_size) {
			axisRemapping[i] = i;
		}

		if (i >= old_axesMu_size) {
			axisMultiplier[i] = 1.0;
		}

		if (axisRemapping[i] < 0 || axisRemapping[i] > (signed)axes.size()-1 ) {
			ROS_ERROR("Wrong axes mapping for axis %d, value must be between 0 and %d "
					,i, (signed)axes.size()-1);
			ROS_ERROR("Setting identity for axis %d", i);
			axisRemapping[i] = i;
		}

	}


}

void JoystickConfig::setAxisCutOff(float axisCutOff_)
{
	axisCutOff = axisCutOff_;
}

void JoystickConfig::parseInput(unsigned char* input, sensor_msgs::Joy& joyMsg) const
{
	joyMsg.axes.resize(axes.size());
	for (unsigned int i = 0; i < axes.size(); ++i) {
		float value = (float)input[axes.at(i).bytePos] / (axes.at(i).max - axes.at(i).min);
		value = (value * 2.0) - copysign(1.0, axes.at(i).max - axes.at(i).min);

		if (fabsf(value) < axisCutOff) {
			value = 0.0;
		}
		joyMsg.axes[axisRemapping[i]] = value * axisMultiplier[axisRemapping[i]];

	}

	joyMsg.buttons.resize(buttons.size());
	for (unsigned int i = 0; i < buttons.size(); ++i) {
		bool pressed = ((input[buttons.at(i).bytePos] ^ byteArray[buttons.at(i).bytePos]) & buttons.at(i).mask)
						== buttons.at(i).value;
		joyMsg.buttons[buttonRemapping[i]] = (pressed ? 1 : 0);
//		if (pressed) {
//			ROS_INFO("Button %d pressed!", i);
//		}
	}
}

} /* namespace telekyb */
