/*
 * JoystickConfig.h
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#ifndef JOYSTICKCONFIG_HPP_
#define JOYSTICKCONFIG_HPP_

#include <telekyb_hid/HIDDevice.hpp>

#include <sensor_msgs/Joy.h>

#include <vector>

namespace TELEKYB_NAMESPACE
{

struct JoyAxis {
	int bytePos;
	int min;
	int max;
	std::string name;
};

struct JoyButton {
	int bytePos;
	unsigned char value;
	unsigned char mask;
	std::string name;
};


class JoystickConfig {
protected:
	std::vector<unsigned char> byteArray;
	std::vector<JoyAxis> axes;
	std::vector<JoyButton> buttons;
	float axisCutOff;

	std::vector<int> buttonRemapping;
	std::vector<int> axisRemapping;
	std::vector<double> axisMultiplier;

public:
	JoystickConfig();
	virtual ~JoystickConfig();

	void setVectors(const std::vector<unsigned char>& byteArray_,
			const std::vector<JoyAxis>& axes_, const std::vector<JoyButton>& buttons_);
	void setModifiers(const std::vector<int>& buttonRemapping_,
			const std::vector<int>& axisRemapping_, const std::vector<double>& axisMultiplier_);

	void parseInput(unsigned char* input, sensor_msgs::Joy& joyMsg) const;
	void setAxisCutOff(float axisCutOff_);

};


} /* namespace telekyb */


#endif /* JOYSTICKCONFIG_H_ */
