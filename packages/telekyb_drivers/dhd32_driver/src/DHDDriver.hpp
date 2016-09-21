/*
 * DHDDriver.h
 *
 *  Created on: Sep 27, 2011
 *      Author: mriedel
 */

#ifndef DHDDRIVER_H_
#define DHDDRIVER_H_

#include <map>

#include <ros/ros.h>

#include "DHDDevice.hpp"

#include <tk_haptics_base/HapticDeviceController.hpp>
#include <pluginlib/class_loader.h>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE
{

//class DHDDriverOptions : public OptionContainer
//{
//public:
////	Option< std::vector<int> >* robotIDs;
////	Option<std::string>* tJoystickTopic;
////	Option<bool>* tUseMKInterface;
//
//	Option< std::string >* tJoystickTopic;
//	Option< double >* tJoystickFrequency;
//	DHDDriverOptions();
//};

class DHDDriver {
protected:
	bool initError;

	// DeviceContainer
	std::set<DHDDevice*> devices;

	// Loader
	pluginlib::ClassLoader<HapticDeviceController> controllerLoader;


	void dhdInit();

public:
	DHDDriver();
	virtual ~DHDDriver();

	void start();
};

} // Namespace

#endif /* DHDDRIVER_H_ */
