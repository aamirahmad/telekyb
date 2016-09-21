/*
 * DHDDevice.cpp
 *
 *  Created on: Sep 29, 2011
 *      Author: mriedel
 */

#include "DHDDevice.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>


// 3rd Party
#include <dhdc.h>
#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

// Status Msg
#include <tk_haptics_msgs/TKHapticOutput.h>

// move to parameters!
//#define INITIAL_REFRESH_RATE 0.01
//#define DELIMITER "_"

#define OMEGA_REST_POS_X  0.022
#define OMEGA_REST_POS_Y  0.000
#define OMEGA_REST_POS_Z -0.067

// 0.8cm
#define OMEGA_REST_MIN_DISTANCE 0.007
#define OMEGA_SHUTDOWN_TIMEOUT_S 3.0

namespace TELEKYB_NAMESPACE
{

DHDDevice::DHDDevice(int id_, bool alreadyOpen)
	: id(id_), options(NULL), controller(NULL)
{
	identifier = lexical_cast<std::string>(id);
	options = new DHDDeviceOptions(identifier);

	if (!alreadyOpen) {
		dhdOpenID(id);
	}
	name = dhdGetSystemName(id);
	longIdentifier = "ID: " + lexical_cast<std::string>(id) + " Type: " + name;

	// ROS
	nodeHandle = ros::NodeHandle(ROSModule::Instance().getMainNodeHandle(), identifier);


	dhdEnableExpertMode();

	if (options->tCustomEffectorMass->getValue() >= 0.0) {
		// set custom EffectorMass
		ROS_INFO("Setting Custom Effector Mass of %f", options->tCustomEffectorMass->getValue());
		dhdSetEffectorMass(options->tCustomEffectorMass->getValue(), id);
	}

	// Set Option Values
	if (options->tDisableGravityCompensation->getValue()) {
		ROS_INFO("Disabling Gravity compenstation.");
		// Disable Compenstation
		dhdSetGravityCompensation(DHD_OFF, id);
	} else {
		dhdSetGravityCompensation(DHD_ON, id);
	}

	if (options->tEnableForceAtStart->getValue()) {
		ROS_INFO("Enabling Force at Startup");
		dhdEnableForce(DHD_ON, id);
	}

	// Test
	//dhdReset(id);
//	if (dhdResetWrist(id) == -1) {
//		ROS_ERROR("Unable to initalize Wrist Calibration.");
//	}

	dhdDisableExpertMode();


	//struct DhdParams dhd33Params = {
	//		/*posMin*/ {-0.0475, -0.109, -0.08},
	//		/*posMax*/ { 0.071,   0.108,  0.122},
	//		/*posRotation*/{
	//			{0.0,1.0,0.0},
	//			{-1.0, 0.0,0.0},
	//			{0.0, 0.0,1.0}
	//		},
	//		/*noNalibPos*/
	//		{0.018, 0.000, -0.073}
	//};
}

DHDDevice::~DHDDevice()
{
	// INFO: This doesn't get output anymore, because the node is already shut down. cout would work.
	ROS_INFO_STREAM(longIdentifier << ": Closing Device");
	//dhdEnableForce(DHD_OFF, id);
	dhdClose(id);

	// Delete Controller
	if (controller) {
		delete controller;
	}

	delete options;
}

std::string DHDDevice::getLongIdentifier() const
{
	return longIdentifier;
}

void DHDDevice::loadController(pluginlib::ClassLoader<HapticDeviceController>& controllerLoader)
{
	try {
		controller = controllerLoader.createClassInstance(options->tHapticDeviceController->getValue());
		// success
		//ROS_INFO_STREAM("Successfully loaded Behavior: " << behaviorClassName << ", Instance: " << (long)b);
	} catch (pluginlib::PluginlibException &e) {
		ROS_ERROR_STREAM("HapticDeviceController Plugin "
				<< options->tHapticDeviceController->getValue() << " failed to load. Message: " << e.what());
	}

}

void DHDDevice::startThread()
{
	thread = boost::thread(&DHDDevice::spin, this);
}

void DHDDevice::joinThread()
{
	thread.join();
}

bool DHDDevice::init()
{
	if (!controller) {
		ROS_ERROR("Cannot enter spinloop without a HapticDeviceController");
		return false;
	}
	// Send initializer
	controller->setIdentifier(identifier);

	// Do controller Init
	HapticAxesMapping xAxis, yAxis, zAxis;
	// Axes for DHD Devices
	xAxis = HapticAxesMapping::Forward;
	yAxis = HapticAxesMapping::Right;
	zAxis = HapticAxesMapping::Up;
	controller->setAxesMapping(xAxis, yAxis, zAxis);

	// create Publisher
	statusPub = nodeHandle.advertise<tk_haptics_msgs::TKHapticOutput>(options->tStatusOutputTopic->getValue(), 10);

	return true;
}

void DHDDevice::spin()
{
	if (!init()) {
		return;
	}

	// DHD Parameters out / spin
	double px, py, pz;
	double oa, ob, og;
	double vx, vy, vz;
	//double ifx, ify, ifz;
	//double ita, itb, itg;


	ROS_INFO_STREAM(longIdentifier << ": Entering spin loop");
	controller->willEnterSpinLoop();

	// Loop Variables
	Vector3D appliedForce;
	Eigen::Matrix3d rotMatix;

	// For StatusOutput
	Timer outputTimer;
	tk_haptics_msgs::TKHapticOutput outputMsg;
	outputMsg.header.frame_id = longIdentifier;
	while (ros::ok()) {

		// write down position
		if (dhdGetPositionAndOrientationRad(&px, &py, &pz, &oa, &ob, &og, id) < 0) {
			ROS_ERROR_STREAM(longIdentifier <<": Cannot read position and/or orientation: " << dhdErrorGetLastStr());
			break;
		}

		if (dhdGetLinearVelocity(&vx, &vy, &vz, id) < 0) {
			ROS_ERROR_STREAM(longIdentifier <<": Cannot get linear velocity estimate: " << dhdErrorGetLastStr());
			break;
		}

		// Generate Output

		rotMatix =
				Eigen::AngleAxisd(oa, Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(ob, Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(og, Eigen::Vector3d::UnitZ());

		output.position = Position3D(px,py,pz) - options->tCenterTranslation->getValue();
		output.linVelocity = Velocity3D(vx, vy, vz);
		output.orientation = Quaternion(rotMatix);
		output.force = input.force;
		output.frequency = dhdGetComFreq(id) * 1000;
		output.primaryButton = dhdGetButton(0, id);

		// loop CB
		controller->loopCB(output, input);

		appliedForce = input.force + options->tForceOffset->getValue();

		if (dhdSetForce(appliedForce(0), appliedForce(1), appliedForce(2), id) < DHD_NO_ERROR) {
			ROS_ERROR_STREAM(identifier << ": Cannot set force: " << dhdErrorGetLastStr());
			break;
		}

		// send out status?
		double outputFreq = options->tStatusOutputFreq->getValue();
		if (outputFreq > 0.01 && outputFreq > outputTimer.frequency()) {
			// reset
			outputTimer.reset();

			outputMsg.header.stamp = ros::Time::now();
			outputMsg.force.x = input.force(0);
			outputMsg.force.y = input.force(1);
			outputMsg.force.z = input.force(2);
			outputMsg.frequency = output.frequency;
			outputMsg.linear.x = output.linVelocity(0);
			outputMsg.linear.y = output.linVelocity(1);
			outputMsg.linear.z = output.linVelocity(2);
			outputMsg.pose.position.x = output.position(0);
			outputMsg.pose.position.y = output.position(1);
			outputMsg.pose.position.z = output.position(2);
			outputMsg.pose.orientation.w = output.orientation.w();
			outputMsg.pose.orientation.x = output.orientation.x();
			outputMsg.pose.orientation.y = output.orientation.y();
			outputMsg.pose.orientation.z = output.orientation.z();
			outputMsg.primaryButton = output.primaryButton;
			// publish
			statusPub.publish(outputMsg);
		}
	}
	controller->didLeaveSpinLoop();
	ROS_INFO_STREAM(longIdentifier << ": Leaving spin loop");

	// Gentle Shutdown for legacy OMEGA.
	if (dhdGetSystemType(id) == DHD_DEVICE_OMEGA) {

		Timer timeout;

		Position3D endPos(OMEGA_REST_POS_X, OMEGA_REST_POS_Y, OMEGA_REST_POS_Z);

		Position3D currPos(px,py,pz);
		Velocity3D currVel(vx,vy,vz);
		double posGain = 80.0;
		double velGain = 20.0;
		Position3D direction = endPos - currPos;
		while (direction.norm() > OMEGA_REST_MIN_DISTANCE && timeout.getElapsed().toDSec() < OMEGA_SHUTDOWN_TIMEOUT_S) {


			Vector3D force = posGain*direction - velGain*currVel;
			//std::cout << "OMEGA applying: " << std::endl << force << std::endl;
			//force = Vector3D(0.0, 0.0, 0.0);
			// apply force
			if (dhdSetForce(force(0), force(1), force(2), id) < DHD_NO_ERROR) {
				std::cerr << identifier << ": Cannot set force: " << dhdErrorGetLastStr() << std::endl;
				break;
			}


			// write new position
			if (dhdGetPositionAndOrientationRad(&px, &py, &pz, &oa, &ob, &og, id) < 0) {
				std::cerr << longIdentifier <<": Cannot read position and/or orientation: " << dhdErrorGetLastStr() << std::endl;
				break;
			}

			if (dhdGetLinearVelocity(&vx, &vy, &vz, id) < 0) {
				std::cerr << longIdentifier <<": Cannot get linear velocity estimate: " << dhdErrorGetLastStr() << std::endl;
				break;
			}

			// update.
			currPos << px,py,pz;
			currVel << vx,vy,vz;

			direction = endPos - currPos;
		}
	}

}

} // Namespace

