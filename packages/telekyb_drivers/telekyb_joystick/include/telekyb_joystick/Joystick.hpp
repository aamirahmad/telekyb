/*
 * Joystick.hpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_joystick/BaseJoystick.hpp>
#include <telekyb_joystick/JoystickOptions.hpp>


namespace TELEKYB_NAMESPACE
{

class Joystick : public BaseJoystick {
protected:
	// Options
	JoystickOptions* options;
	// ROS
	ros::NodeHandle nodeHandle;
	ros::Publisher pubJoy;

	// Vector3Stamped
	ros::Publisher pubVector3;


public:
	Joystick();
	Joystick(const std::string& devPath_, bool autoOpen_ = true);
	virtual ~Joystick();

	// run loop;
	void run();

//	void startThread();
//	void joinThread();

//	static std::vector<Joystick> getJoysticks(const std::string& paths);
};

}
#endif /* JOYSTICK_HPP_ */
