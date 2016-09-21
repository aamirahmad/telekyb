/*
 * Joystick.cpp
 *
 *  Created on: Oct 25, 2011
 *      Author: mriedel
 */

#include <telekyb_joystick/Joystick.hpp>

#include <linux/joystick.h>
#include <sensor_msgs/Joy.h>
#include <telekyb_base/ROS.hpp>

#include <geometry_msgs/Vector3Stamped.h>

// boost
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

// plausibility checks
#define MAX_NUMBER_AXES 20
#define MAX_NUMBER_BUTTONS 50

namespace TELEKYB_NAMESPACE {


Joystick::Joystick()
	: BaseJoystick("", false)
{

}

Joystick::Joystick(const std::string& devPath_, bool autoOpen_)
	: BaseJoystick(devPath_, autoOpen_)
{

	options = JoystickOptions::InstancePtr();
	nodeHandle = ROSModule::Instance().getNodeNameNodeHandle();
	pubJoy = nodeHandle.advertise<sensor_msgs::Joy>(options->tPubName->getValue(), 1 );
	if (options->tPublishVector3->getValue()) {
		pubVector3 = nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options->tPubName->getValue() + "_vector3", 1 );
	}

}

Joystick::~Joystick()
{

}

void Joystick::run()
{
//    std::cout << "Joystick::run() begin" << std::endl;
	if (!isOpen()) {
		ROS_ERROR("Joystick %s not open. Cannot enter runloop. ", devPath.c_str());
		return;
	}

	// remapping
	std::vector<int> buttonRemapping = options->tButtonRemapping->getValue();
	// check validity
	for (unsigned int i = 0; i < buttonRemapping.size(); ++i) {
		if (buttonRemapping[i] < 0 || buttonRemapping[i] > MAX_NUMBER_BUTTONS ) {
			ROS_ERROR("Wrong button mapping for button %d, value must be between 0 and %d "
					,i, MAX_NUMBER_BUTTONS);
			ROS_ERROR("Setting identity for button %d", i);
			buttonRemapping[i] = i;
		}
	}
	std::vector<int> axisRemapping = options->tAxesRemapping->getValue();
	// check validity
	for (unsigned int i = 0; i < axisRemapping.size(); ++i) {
		if (axisRemapping[i] < 0 || axisRemapping[i] > MAX_NUMBER_AXES ) {
			ROS_ERROR("Wrong axes mapping for axis %d, value must be between 0 and %d "
					,i, MAX_NUMBER_AXES);
			ROS_ERROR("Setting identity for axis %d", i);
			axisRemapping[i] = i;
		}
	}
	std::vector<double> axisMultiplier = options->tAxisMultiplier->getValue();

	double autorepeat_rate = options->tAutoRepeatRate->getValue();
	double autorepeat_interval = 1.0 / autorepeat_rate;
	double coalesce_interval = options->tCoalesceInterval->getValue();
	double scale = -1. / (1. - options->tDeadZone->getValue()) / 32767.;
	double unscaled_deadzone = 32767. * options->tDeadZone->getValue();


	js_event event;
	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	bool publish_now = false;
	bool publish_soon = false;
	bool tv_set = false;
	bool publication_pending = false;

	fd_set set;
	sensor_msgs::Joy joyMsg;

	while (ros::ok()) {

//        std::cout << "Joystick::run() open loop" << std::endl;


		publish_now = false;
		publish_soon = false;

		FD_ZERO(&set);
		FD_SET(joystick_fd, &set);

		int select_out = select(joystick_fd + 1, &set, NULL, NULL, &tv);
		if (select_out == -1) {
			tv.tv_sec = 0;
			tv.tv_usec = 0;
			//ROS_INFO("Select returned negative. %i", ros::isShuttingDown());
			continue;
			//				break; // Joystick is probably closed. Not sure if this case is useful.
		}

		if (FD_ISSET(joystick_fd, &set)) {
			if (read(joystick_fd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
				break; // Joystick is probably closed. Definitely occurs.

			//ROS_INFO("Read data...");
			joyMsg.header.stamp = ros::Time().now();
			switch (event.type) {
			case JS_EVENT_BUTTON:
			case JS_EVENT_BUTTON | JS_EVENT_INIT:
				// remapping?
				if (event.number >= buttonRemapping.size()) {
					// fill
					int old_size = buttonRemapping.size();
					buttonRemapping.resize(event.number + 1);
					for (unsigned int i = old_size; i < buttonRemapping.size(); i++) {
						// add identity.
						buttonRemapping[i] = i;
					}
				}

				if (buttonRemapping[event.number] >= joyMsg.buttons.size()) {
					int old_size = joyMsg.buttons.size();
					joyMsg.buttons.resize(buttonRemapping[event.number] + 1);
					for (unsigned int i = old_size; i < joyMsg.buttons.size(); i++) {
						joyMsg.buttons[i] = 0.0;
					}
				}

				// remapping
				joyMsg.buttons[buttonRemapping[event.number]] = (event.value ? 1 : 0);




				// For initial events, wait a bit before sending to try to catch
				// all the initial events.
                if (!(event.type & JS_EVENT_INIT)){
					publish_now = true;
//                    std::cout << "Joystick::run() publish now=true" << std::endl;
                }
				else
					publish_soon = true;
				break;
			case JS_EVENT_AXIS:
			case JS_EVENT_AXIS | JS_EVENT_INIT:
				// size extension?
				if (event.number >= axisRemapping.size()) {
					// fill
					int old_size = axisRemapping.size();
					axisRemapping.resize(event.number + 1);
					for (unsigned int i = old_size; i < axisRemapping.size(); i++) {
						// add identity.
						axisRemapping[i] = i;
					}
				}
				// multiplier extension?
				if (axisRemapping[event.number] >= axisMultiplier.size()) {
					int old_size = axisMultiplier.size();
					axisMultiplier.resize(axisRemapping[event.number] + 1);
					for (unsigned int i = old_size; i < axisMultiplier.size(); i++)
						axisMultiplier[i] = 1.0;
				}

				if (axisRemapping[event.number] >= joyMsg.axes.size()) {
					int old_size = joyMsg.axes.size();
					joyMsg.axes.resize(axisRemapping[event.number] + 1);
					for (unsigned int i = old_size; i < joyMsg.axes.size(); i++)
						joyMsg.axes[i] = 0.0;
				}
				if (!(event.type & JS_EVENT_INIT)) // Init event.value is wrong.
				{
					double val = event.value;
					// Allows deadzone to be "smooth"
					if (val > unscaled_deadzone)
						val -= unscaled_deadzone;
					else if (val < -unscaled_deadzone)
						val += unscaled_deadzone;
					else
						val = 0;
					joyMsg.axes[axisRemapping[event.number]] = val * scale * axisMultiplier[axisRemapping[event.number]];
				}
				// Will wait a bit before sending to try to combine events.
				publish_soon = true;
				break;
			default:
				ROS_WARN("joy_node: Unknown event type. Please file a ticket. time=%u, value=%d, type=%Xh, number=%d", event.time, event.value, event.type, event.number);
				break;
			}
        } else if (tv_set) {// Assume that the timer has expired.
//            std::cout << "Joystick::run() publish now=true" << std::endl;

			publish_now = true;
        }

		if (publish_now) {
			// Assume that all the JS_EVENT_INIT messages have arrived already.
			// This should be the case as the kernel sends them along as soon as
			// the device opens.
			//ROS_INFO("Publish...");
			if (options->tPublishVector3->getValue() && joyMsg.axes.size() >= 3) {
				geometry_msgs::Vector3Stamped vec3Msg;
				vec3Msg.header.stamp = ros::Time::now();
				vec3Msg.vector.x = joyMsg.axes[0];
				vec3Msg.vector.y = joyMsg.axes[1];
				vec3Msg.vector.z = joyMsg.axes[2];
				pubVector3.publish(vec3Msg);
			}

//            std::cout << "Joystick::run() publish now" << std::endl;

			pubJoy.publish(joyMsg);
			publish_now = false;
			tv_set = false;
			publication_pending = false;
			publish_soon = false;
		}

		// If an axis event occurred, start a timer to combine with other
		// events.
		if (!publication_pending && publish_soon) {
			tv.tv_sec = trunc(coalesce_interval);
			tv.tv_usec = (coalesce_interval - tv.tv_sec) * 1e6;
			publication_pending = true;
			tv_set = true;
			//ROS_INFO("Pub pending...");
		}

		// If nothing is going on, start a timer to do autorepeat.
		if (!tv_set && autorepeat_rate > 0) {
			tv.tv_sec = trunc(autorepeat_interval);
			tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
			tv_set = true;
			//ROS_INFO("Autorepeat pending... %i %i", tv.tv_sec, tv.tv_usec);
		}

		if (!tv_set) {
			tv.tv_sec = 1;
			tv.tv_usec = 0;
		}

		//diagnostic_.update();
	} // End of joystick open loop.
//    std::cout << "Joystick::run() end" << std::endl;

}



//std::vector<Joystick> Joystick::getJoysticks(const std::string& paths)
//{
//	std::vector<std::string> indiviualPaths;
//	boost::split(indiviualPaths, paths, boost::is_any_of(" \t"));
//
//	std::vector<Joystick> res;
//	res.resize(indiviualPaths.size());
//	for (unsigned int i = 0; i < indiviualPaths.size(); ++i) {
//		// create Joystick
//		res[i] = Joystick(indiviualPaths[i]);
//	}
//
//	return res;
//}

}
