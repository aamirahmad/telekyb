/*
 * BaseJoystick.hpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#ifndef BASEJOYSTICK_HPP_
#define BASEJOYSTICK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <string>

namespace TELEKYB_NAMESPACE
{

class BaseJoystick {
protected:
	int joystick_fd;
	std::string devPath;


public:
	BaseJoystick(const std::string& devPath_, bool autoOpen = true);
	virtual ~BaseJoystick();

	bool openJoystick();
	bool isOpen();
	void closeJoystick();
};

}
#endif /* BASEJOYSTICK_HPP_ */
