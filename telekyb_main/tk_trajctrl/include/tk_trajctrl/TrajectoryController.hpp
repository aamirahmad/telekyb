/*
 * TrajectoryController.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYCONTROLLER_HPP_
#define TRAJECTORYCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <tk_trajctrl/TrajectoryControllerOptions.hpp>

// TrajInput
#include <telekyb_base/Messages.hpp>

// Telekyb Msgs
#include <telekyb_msgs/TKState.h>

// Ros
#include <ros/ros.h>

// Boost
#include <boost/thread/mutex.hpp>

// Class Loading
#include <pluginlib/class_loader.h>

// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>

namespace TELEKYB_NAMESPACE {

/*
 * OptionListener<bool> -> if MassEstimation is completely disabled!
 */

class TrajectoryController {
private:
	// Singleton Stuff
	static TrajectoryController* instance;

	TrajectoryController();
	virtual ~TrajectoryController();

	TrajectoryController(const TrajectoryController &);
	TrajectoryController& operator=(const TrajectoryController &);

protected:
	TrajectoryControllerOptions options;

	// ClassLoader
	pluginlib::ClassLoader<TrajectoryTracker> ttLoader;

	// active State
	boost::shared_ptr<telekyb::TrajectoryTracker> activeTrajectoryTracker;

	// Nodehandle
	ros::NodeHandle nodeHandle;
	// Subscribe to TKState
	ros::Subscriber tTcStateSub;

	// initialize. This does further setup AFTER the object has been created.
	// This is needed, since Objects that reference the Behavior Controller can only be created after it returns from the constuctor
	// (Singleton Issue).
	//** This is like a Constructor. It's called by the Singleton creator DIRECTLY AFTER the actual constructor. **/
	void initialize();



public:
	void trajInputStep(const TKTrajectory& nextInput);
	//void toggleMassEstimation(bool toggleME);

	void tkStateCB(const telekyb_msgs::TKState::ConstPtr& msg);


	// Singleton Stuff
	static TrajectoryController& Instance();
	static TrajectoryController* InstancePtr();
	static bool HasInstance();
	static void ShutDownInstance();
};

}

#endif /* TRAJECTORYCONTOLLER_HPP_ */
