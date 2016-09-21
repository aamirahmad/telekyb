/*
 * DHDDriver.cpp
 *
 *  Created on: Sep 27, 2011
 *      Author: mriedel
 */

#include "DHDDriver.hpp"

#include <dhdc.h>
#include <telekyb_defines/telekyb_defines.hpp>

#include <boost/foreach.hpp>


namespace TELEKYB_NAMESPACE
{

DHDDriver::DHDDriver()
	: controllerLoader("tk_haptics_base", std::string( TELEKYB_NAMESPACE_STRING ) + "::HapticDeviceController")
{
	dhdInit();
}

DHDDriver::~DHDDriver()
{
	// Destruct Devices.
	BOOST_FOREACH(DHDDevice* d, devices) {
		delete d;
	}
}


void DHDDriver::dhdInit()
{
	int api_major, api_minor, api_release, api_revision;
	dhdGetAPIVersion(&api_major, &api_minor, &api_release, &api_revision);

	if (dhdGetDeviceCount() != 0) {
		for ( int i = 0;  i < dhdGetDeviceCount(); ++i) {
			dhdOpenID(i);
			ROS_INFO_STREAM("Device " << i << ": " << dhdGetSystemName(i));
			// chose first(!) found device
			if (dhdGetSystemType(i) == DHD_DEVICE_NONE) {
				ROS_ERROR("Device with ID %d recognized, but unsupported in DHD %d.%d.%d.%d",i, api_major, api_minor, api_release, api_revision);
				dhdClose(i);
			} else {
				devices.insert(new DHDDevice(i, true));
			}
		}
	}

	// Print Error if no devices in list
	if (devices.empty()) {
		ROS_ERROR("Haptic Device not found!");
	}

}

void DHDDriver::start()
{
	if (devices.empty()) {
		return;
	}

	// Load Controllers
	ROS_INFO("Loading Controllers for devices.");
	BOOST_FOREACH(DHDDevice* d, devices) {
		d->loadController(controllerLoader);
	}

	ROS_INFO("Creating threads for devices.");
	// Start one Thread per Device
	BOOST_FOREACH(DHDDevice* d, devices) {
		d->startThread();
	}

	// Wait for all Threads to finish.
	BOOST_FOREACH(DHDDevice* d, devices) {
		d->joinThread();
	}
}

} // Namespace
