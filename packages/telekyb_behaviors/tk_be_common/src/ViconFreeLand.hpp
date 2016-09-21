/*
 * ViconFreeLand.hpp
 *
 *  Created on: Nov 10, 2011
 *      Author: mriedel
 */

#ifndef VICONFREELAND_HPP_
#define VICONFREELAND_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_behavior/Behavior.hpp>
#include "telekyb_msgs/Behavior.h"
#include "std_msgs/Float64.h"

// plugin stuff
//#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Vector3.h>
#include <telekyb_msgs/TKState.h>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Filter/OneEuroFilter.hpp>
#include <telekyb_base/Estimators/VarianceEstimator.hpp>
#include <std_msgs/Bool.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_behavior {

class ViconFreeLand : public Behavior {
protected:
	Option<double>* tLandVelocity;
	Option<std::string>* tBehaviorTopic;
	Option<std::string>* tAccComTopic;
	Option<std::string>* tDvoVelTopic;
	Option<std::string>* tVerticalVelTopic;
	Option<double>* tGainValue;
	Option<double>* tWaitingTime;
	Option<double>* tFilterAlpha;
	Option<double>* tDetectionSensitivity;
	
	double comZ;
	double dvoZVel;
	double estimatedAcc;
	double estimatedVel;
	double k;
	double commandedZVel;
	double elapsedTime;
	bool detected;
	bool timerInitialized;
	std::string currentBehavior;
		
	Timer integTimer;
	Timer detectionTimer;
	
	ros::NodeHandle nodeHandle;

	ros::Subscriber behaviorSub;
	ros::Subscriber accSub;
	ros::Subscriber velSub;
	ros::Subscriber dvoSub;
	
	HighPassFilter *filter;
	VarianceEstimator *varianceEstimator;
	
	void behaviorCallback(const telekyb_msgs::Behavior::ConstPtr& msg);
	
	void accComCallback(const geometry_msgs::Vector3::ConstPtr& msg);
	
	void dvoVelCallback(const telekyb_msgs::TKState::ConstPtr& msg);
	
	void detectionCallback(const std_msgs::Bool::ConstPtr& msg);
	
	void verticalVelCallback(const geometry_msgs::Vector3::ConstPtr& msg);

	
public:
	ViconFreeLand();

	virtual void initialize();
	virtual void destroy();

	// Called directly after Change Event is registered.
	virtual bool willBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called after actual Switch. Note: During execution trajectoryStepCreation is used
	virtual void didBecomeActive(const TKState& currentState, const Behavior& previousBehavior);
	// Called directly after Change Event is registered: During execution trajectoryStepTermination is used
	virtual void willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);
	// Called after actual Switch. Runs in seperate Thread.
	virtual void didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior);

	// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
	virtual void trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
	virtual void trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
	virtual void trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput);

	// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
	virtual bool isValid(const TKState& currentState) const;
};

}

#endif /* VICONFREELAND_HPP_ */
