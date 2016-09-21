/*
 * SampleController.cpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#include "SampleController.hpp"

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <ros/console.h>

PLUGINLIB_DECLARE_CLASS(tk_haptics_base, SampleController, telekyb_haptic::SampleController, TELEKYB_NAMESPACE::HapticDeviceController);

namespace telekyb_haptic {

SampleController::SampleController()
{

}

SampleController::~SampleController()
{

}

// Identifier (e.g. for NodeHandle)
void SampleController::setIdentifier(const std::string& identifier)
{
	ROS_INFO("SampleController: Got identifier %s", identifier.c_str());
}

// Get's specific Axes mapping, Set if needed
void SampleController::setAxesMapping(HapticAxesMapping& xAxis, HapticAxesMapping& yAxis, HapticAxesMapping& zAxis)
{
	ROS_INFO("Recv Axes Mapping (X,Y,Z): (%s,%s,%s)", xAxis.str(), yAxis.str(), zAxis.str());
}

// Get the Range of each axes
void SampleController::setAxesRange(const Position3D& minValues, const Position3D& maxValues) {

}

void SampleController::willEnterSpinLoop()
{
	frequencyTimer.reset();
}

// has to be fast and should not slow down the loop
void SampleController::loopCB(const HapticOuput& output, HapticInput& input) {

	if (frequencyTimer.frequency() < 1.0) {
		ROS_INFO("Output every second");

		ROS_INFO("Position: (%f,%f,%f)", output.position(0), output.position(1), output.position(2));
		ROS_INFO("Velocity: (%f,%f,%f)", output.linVelocity(0), output.linVelocity(1), output.linVelocity(2));
		Eigen::Vector3d angles = output.orientation.toRotationMatrix().eulerAngles(0,1,2);
		ROS_INFO("Orientation: (%f,%f,%f)", angles(0), angles(1), angles(2));
		ROS_INFO("Force: (%f,%f,%f)", output.force(0), output.force(1), output.force(2));
		ROS_INFO("Loop Frequency: %d", (int)(output.frequency) );
		ROS_INFO("Primary Button: %d", output.primaryButton);


		frequencyTimer.reset();
	}

	// Center to 0/0/0
	double posGain = 100.0;
	double velGain = 20.0;
	//input.force = -posGain*output.position - velGain*output.linVelocity;
	input.force = Vector3D(0.0,0.0,0.0);
}

void SampleController::didLeaveSpinLoop()
{

}

}
