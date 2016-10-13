/*
 * EurocTrajectoryTracker.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef EUROCTRAJECTORYTRACKER_HPP_
#define EUROCTRAJECTORYTRACKER_HPP_

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
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>


// Boost
#include <boost/thread/mutex.hpp>

// using namespace TELEKYB_NAMESPACE;

namespace trajectory_trackers_plugin {


// Option Definition
class EurocTrajectoryTrackerOptions : public 	TELEKYB_NAMESPACE::OptionContainer {
public:
	//Option<bool>* tCompletelyDisableME;

	TELEKYB_NAMESPACE::Option<std::string>* tCommandsTopic;
	TELEKYB_NAMESPACE::Option<std::string>* tPluginLookupName;
	TELEKYB_NAMESPACE::Option<int>* tTaskNumber;
	
	TELEKYB_NAMESPACE::Option<std::string>* tExternalForceTopic;
	TELEKYB_NAMESPACE::Option<std::string>* tCommandedThurstTopic;


	EurocTrajectoryTrackerOptions();
};

/*
 * OptionListener<bool> -> if MassEstimation is completely disabled!
 */

class EurocTrajectoryTracker : public TELEKYB_NAMESPACE::TrajectoryTracker {
protected:
	
	TELEKYB_NAMESPACE::YawControl* yawControl;
	
	TELEKYB_NAMESPACE::PositionControl* positionControl;

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
	
	ros::Subscriber extforcesub;
	ros::Subscriber commandedthrustsub;

	// Loaded Mass Estimator
	boost::shared_ptr<tk_param_estimator::MassEstimator> massEstimator;

	EurocTrajectoryTrackerOptions options;

	double additional_pitch;
	double additional_roll;
	
	double commanded_thrust;

public:
	EurocTrajectoryTracker();
	virtual ~EurocTrajectoryTracker();

	// Euroc Interface functions
	void initialize();
	void destroy();

	std::string getName() const;

	// Callback Functions
	void trajectoryCB(const TELEKYB_NAMESPACE::TKTrajectory& trajectory);
	void stateCB(const TELEKYB_NAMESPACE::TKState& state);
	
// 	void clockcb(const rosgraph_msgs::Clock::ConstPtr& msg);
	void extforcecb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void thrustcb(const std_msgs::Float64::ConstPtr& msg);

};

}

#endif /* EUROCTRAJECTORYTRACKER_HPP_ */
