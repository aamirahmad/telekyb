/*
 * ViconFreeLand.cpp
 *
 *  Created on: Oct 31, 2013
 *      Author: mbasile
 */

#include <telekyb_base/ROS.hpp>
#include "ViconFreeLand.hpp"





// Declare
PLUGINLIB_DECLARE_CLASS(tk_be_common, ViconFreeLand, telekyb_behavior::ViconFreeLand, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

ViconFreeLand::ViconFreeLand()
	: Behavior("tk_be_common/ViconFreeLand", BehaviorType::Land)
{

}


void ViconFreeLand::initialize()
{
	tAccComTopic = addOption<std::string>("tAccComTopic",
			"topic published by PositionControl",
			"comAcc", false, false);
	
	tVerticalVelTopic = addOption<std::string>("tVerticalVelTopic",
			"topic to get the commanded velocity", "undef", false, false);
	
	tDvoVelTopic = addOption<std::string>("tDvoVelTopic",
			"topic containing the DVO velocity estimate",
			"localState", false, false);
	
	tGainValue = addOption("tGainValue",
			"gain for the estimation of the accelaration", 12.0, false ,false);
	
	tFilterAlpha = addOption("tFilterAlpha",
			"alpha value used for the filter", 0.05, false ,false);
	
	tDetectionSensitivity = addOption("tDetectionSensitivity",
			"value used to estabilish the sensitivity to contacts", 3.0, false ,false);
	
	tWaitingTime = addOption("tWaitingTime",
			"seconds to wait before switching off the motors when no peaks are detected but the robot has landed", 1.5, false ,false);
	
	tLandVelocity = addOption<double>("tLandVelocity",
			"Defines the Velocity of the Land Behavior",
			0.5, false, false);
	
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	
}

void ViconFreeLand::destroy()
{

}

void ViconFreeLand::verticalVelCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	commandedZVel = msg->z;
}

void ViconFreeLand::accComCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
	comZ = msg->z;
}

void ViconFreeLand::dvoVelCallback(const telekyb_msgs::TKState::ConstPtr& msg)
{
	dvoZVel = msg->twist.linear.z;
}

void ViconFreeLand::detectionCallback(const std_msgs::Bool_< std::allocator< void > >::ConstPtr& msg)
{
	detected = msg->data;
}


bool ViconFreeLand::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{		
	
	velSub = nodeHandle.subscribe(tVerticalVelTopic->getValue(), 1, &ViconFreeLand::verticalVelCallback, this);
	accSub = nodeHandle.subscribe(tAccComTopic->getValue(), 1, &ViconFreeLand::accComCallback, this);
	dvoSub = nodeHandle.subscribe(tDvoVelTopic->getValue(), 1, &ViconFreeLand::dvoVelCallback, this);

	filter = new HighPassFilter(tFilterAlpha->getValue(), 0);
	varianceEstimator = new VarianceEstimator();

	detected = false;
	dvoZVel = 0;
	comZ = 0;
	estimatedAcc = 0.0;
	estimatedVel = 0.0;
	k = tGainValue->getValue();
	timerInitialized = false;
	return true;
}

void ViconFreeLand::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void ViconFreeLand::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void ViconFreeLand::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void ViconFreeLand::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void ViconFreeLand::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{	
	double timeStep = integTimer.getElapsed().toDSec();
	integTimer.reset();
	estimatedAcc = k*(dvoZVel - estimatedVel);
	estimatedVel += estimatedAcc*timeStep;
	
	double appEstAcc = estimatedAcc;
	double standarDeviation;
	
	appEstAcc =  filter->filter(appEstAcc);
	
	standarDeviation = sqrt(varianceEstimator->step(estimatedAcc));
	
	bool velInconsistency = (commandedZVel > 0) && (fabs(dvoZVel) < 0.1);	
	
	if (velInconsistency) {
		if (!timerInitialized) {
			detectionTimer.reset();
			elapsedTime = detectionTimer.getElapsed().toDSec();
			timerInitialized = true;
		} else {
			elapsedTime = detectionTimer.getElapsed().toDSec();
		}
	} else timerInitialized = false;
	
	std::cout << "elapsedTime: " << elapsedTime << std::endl;
	
	detected = (estimatedAcc < - tDetectionSensitivity->getValue()*standarDeviation) || (velInconsistency && elapsedTime > tWaitingTime->getValue());

	generatedTrajInput.setVelocity(Velocity3D(0.0, 0.0, tLandVelocity->getValue()));
	generatedTrajInput.setYawRate(0.0);
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void ViconFreeLand::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool ViconFreeLand::isValid(const TKState& currentState) const
{
	return !detected;
}


}
