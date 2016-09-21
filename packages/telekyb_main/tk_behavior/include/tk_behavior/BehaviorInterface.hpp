/*
 * BehaviorInterface.hpp
 *
 *  Created on: Nov 22, 2011
 *      Author: mriedel
 */

#ifndef BEHAVIORINTERFACE_HPP_
#define BEHAVIORINTERFACE_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <ros/ros.h>

// Services
#include <telekyb_srvs/BehaviorInput.h>
#include <telekyb_srvs/BehaviorOutput.h>

#include <telekyb_srvs/BoolInput.h>

namespace TELEKYB_NAMESPACE {

// forward declaration
class Behavior;

class BehaviorInterface {
protected:
	Behavior& behavior;

	// NodeHandle
	ros::NodeHandle nodeHandle;

	// ServiceServers
	ros::ServiceServer setNextBehavior;
	ros::ServiceServer getNextBehavior;
	ros::ServiceServer setParameterInitialized;

	// ServiceServers Callbacks
	bool setNextBehaviorCB(
			telekyb_srvs::BehaviorInput::Request& request,
			telekyb_srvs::BehaviorInput::Response& response);

	bool getNextBehaviorCB(
			telekyb_srvs::BehaviorOutput::Request& request,
			telekyb_srvs::BehaviorOutput::Response& response);

	bool setParameterInitializedCB(
			telekyb_srvs::BoolInput::Request& request,
			telekyb_srvs::BoolInput::Response& response);


public:
	BehaviorInterface(Behavior& behavior_);
	virtual ~BehaviorInterface();
};

}

#endif /* BEHAVIORINTERFACE_HPP_ */
