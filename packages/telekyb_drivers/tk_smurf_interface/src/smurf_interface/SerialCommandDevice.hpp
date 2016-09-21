/*
 * SerialCommandDevice.hpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel
 */

#ifndef SERIALCOMMANDDEVICE_HPP_
#define SERIALCOMMANDDEVICE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_msgs/TKMotorCommands.h>
#include <telekyb_serial/SerialDevice.h>

#include <telekyb_base/Time.hpp>
#include <telekyb_base/ROS.hpp>

#include "SerialCommandDeviceOptions.hpp"

namespace TELEKYB_NAMESPACE {

class SerialCommandDevice : public SerialDevice {
private:
	SerialCommandDeviceOptions &options;

	ros::NodeHandle nodeHandle;
	ros::Subscriber commandSub;
	telekyb::Timer secureTimer;

	void commandCallback(const telekyb_msgs::TKMotorCommands::ConstPtr& commandMsg);
	inline
	unsigned char setpoint( double force);

public:
	SerialCommandDevice();
	virtual ~SerialCommandDevice();

};

}

#endif /* SERIALCOMMANDDEVICE_HPP_ */
