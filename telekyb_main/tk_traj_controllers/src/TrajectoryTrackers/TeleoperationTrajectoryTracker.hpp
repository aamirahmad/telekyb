/**
 * TeleoperationTrajectoryTracker.hpp
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or
 * copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 *  Created on: Sep 29, 2014
 *      Author: Johannes Lächele
 *  
 */
#ifndef TELEOPERATIONTRAJECTORYTRACKER_HPP_
#define TELEOPERATIONTRAJECTORYTRACKER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

// Interface Definition
#include <tk_trajctrl/TrajectoryTracker.hpp>

// Position Controller
#include <tk_ctrlalgo/PositionControl.hpp>
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

namespace trajectory_trackers_plugin {


// Option Definition
class TeleoperationTrajectoryTrackerOptions : public TELEKYB_NAMESPACE::OptionContainer {
public:
	//Option<bool>* tCompletelyDisableME;

	TELEKYB_NAMESPACE::Option<std::string>* tCommandsTopic;
	TELEKYB_NAMESPACE::Option<std::string>* tPluginLookupName;

	TeleoperationTrajectoryTrackerOptions();
};

class TeleoperationTrajectoryTracker : public TELEKYB_NAMESPACE::TrajectoryTracker {
protected:

	TELEKYB_NAMESPACE::YawControl* yawControl;

	TELEKYB_NAMESPACE::PositionControl* positionControl;
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

	TeleoperationTrajectoryTrackerOptions options;

public:
	TeleoperationTrajectoryTracker();
	virtual ~TeleoperationTrajectoryTracker();

	// Standard Interface functions
	void initialize();
	void destroy();

	std::string getName() const;

	// Callback Functions
	void trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory);
	void stateCB(const TELEKYB_NAMESPACE::TKState& state);

};

} // namespace trajectory_trackers_plugin
#endif /* TELEOPERATIONTRAJECTORYTRACKER_HPP_ */
