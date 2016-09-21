/*
 * DHDDevice.h
 *
 *  Created on: Sep 29, 2011
 *      Author: mriedel
 */

#ifndef DHDDEVICE_HPP_
#define DHDDEVICE_HPP_

#include <ros/ros.h>
#include <iostream>

#include <boost/thread.hpp>

#include <tk_haptics_base/HapticDeviceController.hpp>
#include <pluginlib/class_loader.h>

#include "DHDDeviceOptions.hpp"

namespace TELEKYB_NAMESPACE
{

class DHDDevice {
private:
	// ID of the Device
	int id;
	std::string name;
	std::string identifier;
	std::string longIdentifier;

	// Options
	DHDDeviceOptions *options;

	// Controller
	HapticDeviceController *controller;

	HapticInput input;
	HapticOuput output;

	// Params
	//DHDParams params;
	//Position3D offset;
	//double normFactor;

	// ROS
	ros::NodeHandle nodeHandle;

	// tk_haptics_msgs::TKHapticOutput Publisher
	ros::Publisher statusPub;

	// Thread
	boost::thread thread;

	// initializer before spin loop
	bool init();
	// the threaded function
	void spin();

public:
	DHDDevice(int id_, bool alreadyOpen = false);
	virtual ~DHDDevice();

	std::string getLongIdentifier() const;

	// Thread stuff
	void startThread();
	void joinThread();

	void loadController(pluginlib::ClassLoader<HapticDeviceController>& controllerLoader);
};

} // Namespace

#endif /* DHDDEVICE_HPP_ */
