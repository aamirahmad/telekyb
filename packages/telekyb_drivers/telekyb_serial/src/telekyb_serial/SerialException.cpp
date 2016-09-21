/*
 * SerialException.cpp
 *
 *  Created on: Oct 13, 2011
 *      Author: mriedel
 */

#include <telekyb_serial/SerialException.h>

#include <ros/ros.h>

namespace TELEKYB_NAMESPACE
{

SerialException::SerialException(const std::string& msg_, SerialExceptionCode code_,ros::console::Level level_)
	: msg(msg_), level(level_) , processed(false), code(code_) {

}

SerialException::~SerialException() throw() {
	if (!processed) {
		process();
	}
}

void SerialException::process() {

	ROS_LOG_STREAM(level, ROSCONSOLE_DEFAULT_NAME, msg);

	if (level == ros::console::levels::Fatal) {
		//ROS_BREAK();
		ros::shutdown();
	}

	processed = true;
}

}
