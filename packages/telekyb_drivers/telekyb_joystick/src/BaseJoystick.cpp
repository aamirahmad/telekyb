/*
 * BaseJoystick.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#include <telekyb_joystick/BaseJoystick.hpp>

#include <fcntl.h>
#include <ros/console.h>

namespace TELEKYB_NAMESPACE
{

BaseJoystick::BaseJoystick(const std::string& devPath_, bool autoOpen_)
	: joystick_fd(-1), devPath(devPath_)
{
	if (autoOpen_) {
		openJoystick();
	}

}

BaseJoystick::~BaseJoystick()
{
	// does not make Sense. Element not copyable.
	//closeJoystick();
}


bool BaseJoystick::openJoystick()
{
	if ((joystick_fd = open(devPath.c_str(), O_RDONLY)) != -1) {
		ROS_INFO("Opened joystick: %s", devPath.c_str());
		return true;
	} else {
		ROS_ERROR("Unable to open joystick: %s", devPath.c_str());
		return false;
	}
}
bool BaseJoystick::isOpen()
{
	return (joystick_fd != -1);
}
void BaseJoystick::closeJoystick()
{
	if (isOpen()) {
		close(joystick_fd);
	}
}

}
