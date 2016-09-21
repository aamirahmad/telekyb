/*
 * FormationController.cpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#include "HapticFormationController.hpp"

#include <telekyb_base/ROS/ROSModule.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <ros/console.h>

// to get the system
#include <telekyb_interface/TeleKybCore.hpp>

PLUGINLIB_DECLARE_CLASS(tk_formation, HapticFormationController, telekyb_haptic::HapticFormationController, TELEKYB_NAMESPACE::HapticDeviceController);

namespace telekyb_haptic {

HapticFormationControllerOptions::HapticFormationControllerOptions(const std::string& identifier)
	: OptionContainer("HapticFormationController_" + identifier)
{
	tRobotIDs = addOption< std::vector<int> >("tRobotIDs",
			"Specifies the ID of the Robots to connect to", std::vector<int>() , true, true);
	tSendFrequency = addOption< double >("tSendFrequency",
			"Send Frequency", 100.0 , false, true);
	tInputTopicName = addOption< std::string >("tInputTopicName",
			"Appended Topicname for TKState Subscription of UAVs", "Sensor/TKState" , false, true);
	tOutputTopicName = addOption< std::string >("tOutputTopicName",
			"Topic to send Vector3Stamped to", "output" , false, true);
	tVelocityGain = addOption< double >("tVelocityGain",
			"Gain of Velocity", 5.0 , false, true);
	Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Zero();
	rotationMatrix.diagonal() << 1.0,-1.0,-1.0;
	tRotationMatrix = addOption<Eigen::Matrix3d>("tRotationMatrix","ConversionMatrix from TKState to Haptic", rotationMatrix, false, true);


	// Gains for Controller
	tPropGain = addOption< double >("tPropGain",
			"Gain of Velocity", -20.0 , false, false);
	tDeriGain = addOption< double >("tDeriGain",
			"Gain of Velocity", -5.0 , false, false);
	tErrGain = addOption< double >("tErrGain",
			"Gain of Velocity", 100.0 , false, false);
}

HapticFormationController::HapticFormationController()
	: options(NULL)
{

}

HapticFormationController::~HapticFormationController()
{
	// free generic subscribers
	for (unsigned int i = 0; i < stateSubscribers.size(); ++i) {
		delete stateSubscribers[i];
	}

	if (options) { delete options; }
}

// Identifier (e.g. for NodeHandle)
void HapticFormationController::setIdentifier(const std::string& identifier)
{
	ROS_INFO("HapticFormationController: Got identifier %s", identifier.c_str());

	options = new HapticFormationControllerOptions(identifier);

	std::vector<int> robotIDs = options->tRobotIDs->getValue();

	//stateSubscribers.resize(stateSubscribers, NULL);
	for (unsigned int i = 0; i < robotIDs.size(); ++i) {
		ros::NodeHandle nh; // wait also 2s
		if (! telekyb_interface::TeleKybCore::getTeleKybCoreMainNodeHandle(robotIDs[i], nh, 4.0)) {
			continue;
		}

		GenericSubscriber< telekyb_msgs::TKState > *sub =
				new GenericSubscriber< telekyb_msgs::TKState >(nh, options->tInputTopicName->getValue(), 1);
		stateSubscribers.push_back(sub);
	}

	if (stateSubscribers.empty()) {
		ROS_WARN("TKState Subscriber list empty. Unable to determine any Velocity.");
	}

	nodeHandle = ros::NodeHandle(ROSModule::Instance().getMainNodeHandle(), identifier);

	vectorPub = nodeHandle.advertise<geometry_msgs::Vector3Stamped>(options->tOutputTopicName->getValue(),10);

}

// Get's specific Axes mapping, Set if needed
void HapticFormationController::setAxesMapping(HapticAxesMapping& xAxis, HapticAxesMapping& yAxis, HapticAxesMapping& zAxis)
{
	ROS_INFO("Recv Axes Mapping (X,Y,Z): (%s,%s,%s)", xAxis.str(), yAxis.str(), zAxis.str());
}

// Get the Range of each axes
void HapticFormationController::setAxesRange(const Position3D& minValues, const Position3D& maxValues) {

}

void HapticFormationController::willEnterSpinLoop()
{
	frequencyTimer.reset();
}

// has to be fast and should not slow down the loop
void HapticFormationController::loopCB(const HapticOuput& output, HapticInput& input) {

	// Del Velocity
	Velocity3D desVel = output.position * options->tVelocityGain->getValue();
	desVel = options->tRotationMatrix->getValue() * desVel;

	Velocity3D curVel = Velocity3D::Zero();
	for (unsigned int i = 0; i < stateSubscribers.size(); ++i) {
		TKState velMsg(stateSubscribers[i]->getLastMsg());
		curVel += velMsg.linVelocity;
	}

	if (stateSubscribers.size() != 0) {
		curVel /= stateSubscribers.size();
	}


	Velocity3D velError = (curVel - desVel) / options->tVelocityGain->getValue();
	velError = options->tRotationMatrix->getValue() * velError;

	// Calculate force
	input.force = options->tPropGain->getValue() * output.position
			+ options->tDeriGain->getValue() * output.linVelocity
			+ options->tErrGain->getValue() * velError;


	if (frequencyTimer.frequency() < options->tSendFrequency->getValue()) {
		//ROS_INFO("Output every second");

//		ROS_INFO("Position: (%f,%f,%f)", output.position(0), output.position(1), output.position(2));
//		ROS_INFO("Velocity: (%f,%f,%f)", output.linVelocity(0), output.linVelocity(1), output.linVelocity(2));
//		Eigen::Vector3d angles = output.orientation.toRotationMatrix().eulerAngles(0,1,2);
//		ROS_INFO("Orientation: (%f,%f,%f)", angles(0), angles(1), angles(2));
//		ROS_INFO("Force: (%f,%f,%f)", output.force(0), output.force(1), output.force(2));
//		ROS_INFO("Loop Frequency: %d", (int)(output.frequency) );
//		ROS_INFO("Primary Button: %d", output.primaryButton);
		geometry_msgs::Vector3Stamped outputMsg;
		outputMsg.header.stamp = ros::Time::now();
		outputMsg.vector.x = desVel(0);
		outputMsg.vector.y = desVel(1);
		outputMsg.vector.z = desVel(2);

		vectorPub.publish(outputMsg);
		frequencyTimer.reset();
	}

}

void HapticFormationController::didLeaveSpinLoop()
{

}

}
