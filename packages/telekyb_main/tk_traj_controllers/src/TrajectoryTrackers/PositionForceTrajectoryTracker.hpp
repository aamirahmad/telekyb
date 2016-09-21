/*
 * PositionForceTrajectoryTracker.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef POSITIONFORCETRAJECTORYTRACKER_HPP_
#define POSITIONFORCETRAJECTORYTRACKER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>


// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>

// Position Controller
#include <tk_ctrlalgo/PositionForceControl.hpp>
// Yaw Controller
#include <tk_ctrlalgo/YawControl.hpp>
// Dynamic Mass Estimator
// Class Loading
#include <pluginlib/class_loader.h>
// Interface Definition
#include <tk_param_estimator/MassEstimator.hpp>

// Ros
#include <ros/ros.h>


// Boost
#include <boost/thread/mutex.hpp>

// using namespace TELEKYB_NAMESPACE;

namespace trajectory_trackers_plugin {


// Option Definition
class PositionForceTrajectoryTrackerOptions : public 	TELEKYB_NAMESPACE::OptionContainer {
public:
	//Option<bool>* tCompletelyDisableME;

	TELEKYB_NAMESPACE::Option<std::string>* tCommandsTopic;
	TELEKYB_NAMESPACE::Option<std::string>* tPluginLookupName;

	PositionForceTrajectoryTrackerOptions();
};

/*
 * OptionListener<bool> -> if MassEstimation is completely disabled!
 */

class PositionForceTrajectoryTracker : public TELEKYB_NAMESPACE::TrajectoryTracker {
protected:
	
// 	TELEKYB_NAMESPACE::YawControl yawControllo;
	TELEKYB_NAMESPACE::YawControl* yawControl;
	
// 	TELEKYB_NAMESPACE::PositionControl positionControllo;
	TELEKYB_NAMESPACE::PositionForceControl* positionControl;
	//MassEstimation massEstimation;

	// currentMass -> either constant or estimated by MassEstimator
	double currentMass;

	boost::mutex currentInputMutex;
	TELEKYB_NAMESPACE::TKTrajectory currentInput;

	// Nodehandle
	ros::NodeHandle nodeHandle;
	ros::NodeHandle commandNodeHandle;

	// Publish TKLLCommands
	ros::Publisher tTcCommandsPub;

	// Mass Estimation Option
	TELEKYB_NAMESPACE::Option<bool>* tDoMassEstimation;

	// ClassLoader
	pluginlib::ClassLoader<tk_param_estimator::MassEstimator> meLoader;

	// Loaded Mass Estimator
	boost::shared_ptr<tk_param_estimator::MassEstimator> massEstimator;

	PositionForceTrajectoryTrackerOptions options;

public:
	PositionForceTrajectoryTracker();
	virtual ~PositionForceTrajectoryTracker();

	// Standard Interface functions
	void initialize();
	void destroy();

	std::string getName() const;

	// Callback Functions
	void trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory);
	void stateCB(const TELEKYB_NAMESPACE::TKState& state);


};

}

#endif /* STANDARDTRAJECTORYTRACKER_HPP_ */
