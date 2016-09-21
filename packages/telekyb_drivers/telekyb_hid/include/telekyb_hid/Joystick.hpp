/*
 * Joystick.hpp
 *
 *  Created on: Oct 24, 2011
 *      Author: mriedel
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <telekyb_hid/HIDDevice.hpp>
#include <telekyb_hid/JoystickConfig.hpp>
#include <telekyb_hid/JoystickOptions.hpp>

#include <boost/thread.hpp>

//ros
#include <ros/publisher.h>

#include <vector>

namespace TELEKYB_NAMESPACE
{

class Joystick : public HIDDevice {
protected:
	JoystickOptions* options; // singleton holder
	JoystickConfig config;

	std::string name;

	ros::NodeHandle nodeHandle;
	ros::Publisher pub; // output
	std::string frameID;

	// Thread
	boost::thread* thread;

	// main run loop
	void run(); // endless loop

public:
	Joystick(unsigned short int vendorID_ , unsigned short int productID_, const std::string& name_);
	virtual ~Joystick();

	void setVectors(const std::vector<unsigned char>& byteArray_,
			const std::vector<JoyAxis>& axes_, const std::vector<JoyButton>& buttons_);

	std::string getName() const;
	void setName(const std::string& name_);

	void startThread();
	void joinThread();

	static void getJoysticks(const std::string& filename, std::vector<Joystick*>& joysticks);
};

} /* namespace telekyb */
#endif /* JOYSTICK_HPP_ */
