/*
 * BehaviorInterface.hpp
 *
 *  Created on: Nov 10, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORCONTROLLERINTERFACE_HPP_
#define BEHAVIORCONTROLLERINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <ros/ros.h>

// BehaviorContainer. Contains UserBehaviors that are loaded at runtime
#include <tk_behavior/BehaviorContainer.hpp>

// Services
#include <telekyb_srvs/StringArrayOutput.h>
#include <telekyb_srvs/BehaviorInput.h>
#include <telekyb_srvs/BehaviorOutput.h>
#include <telekyb_srvs/BehaviorInputOutput.h>

#include <std_srvs/Empty.h>

namespace TELEKYB_NAMESPACE {

// Forward Declaration
class BehaviorController;


class BehaviorControllerInterface {
protected:
	// the Controller
	BehaviorController& behaviorController;

	// the Container
	BehaviorContainer behaviorContainer;

	// NodeHandle
	ros::NodeHandle nodeHandle;

	// ServiceServer
	ros::ServiceServer getAvailableBehaviorsClient;
	ros::ServiceServer loadBehavior;
	ros::ServiceServer unloadBehavior;
	ros::ServiceServer switchBehavior;
	ros::ServiceServer getSystemBehavior;
	ros::ServiceServer getActiveBehavior;
	ros::ServiceServer emergencyLand;
	ros::ServiceServer normalBrake;

	// ServiceServerCallBacks
	bool getAvailableBehaviorsCB(
			telekyb_srvs::StringArrayOutput::Request& request,
			telekyb_srvs::StringArrayOutput::Response& response);
	bool loadBehaviorCB(
			telekyb_srvs::BehaviorInputOutput::Request& request,
			telekyb_srvs::BehaviorInputOutput::Response& response);
	bool unloadBehaviorCB(
			telekyb_srvs::BehaviorInput::Request& request,
			telekyb_srvs::BehaviorInput::Response& response);
	bool switchBehaviorCB(
			telekyb_srvs::BehaviorInput::Request& request,
			telekyb_srvs::BehaviorInput::Response& response);
	bool getSystemBehaviorCB(
			telekyb_srvs::BehaviorInputOutput::Request& request,
			telekyb_srvs::BehaviorInputOutput::Response& response);
	bool getActiveBehaviorCB(
			telekyb_srvs::BehaviorOutput::Request& request,
			telekyb_srvs::BehaviorOutput::Response& response);
	bool emergencyLandCB(
			std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);
	bool normalBrakeCB(
			std_srvs::Empty::Request& request,
			std_srvs::Empty::Response& response);


	// Publish activeBehavior Changes
	ros::Publisher activeBehaviorPub;

	// Trajectory
	ros::Publisher trajectoryPub;

public:
	BehaviorControllerInterface(BehaviorController& behaviorController_);
	virtual ~BehaviorControllerInterface();

	// published the curret active Behavior
	void publishActiveBehavior();

	void publishTKTrajectory(const TKTrajectory& trajectory);

};

}

#endif /* BEHAVIORCONTROLLERINTERFACE_HPP_ */
