/*
 * Ground.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: mriedel
 */

#include <behaviors/Ground.hpp>

#include <telekyb_defines/physic_defines.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Ground, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Ground::Ground()
	: Behavior("tk_behavior/Ground", BehaviorType::Ground)
{

}

void Ground::initialize()
{
	controller = TrajectoryController::InstancePtr();
	tDoMassEstimation = OptionContainer::getGlobalOptionByName<bool>("TrajectoryController","tDoMassEstimation");
	if (!tDoMassEstimation) {
		ROS_ERROR("Unable to get Option TrajectoryController/tDoMassEstimation. Quitting...");
		ros::shutdown();
	} else {
		// no Parameters
		parameterInitialized = true;
	}
}

void Ground::destroy()
{

}


// GROUND MUST ALWAYS RETURN TRUE!!!
bool Ground::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// DEBUG.
	//ROS_INFO("Ground: Storing yawAngle: %f ", yawAngle);

	// Deactivate MassEstimation
	// TODO: Change this to Option Access!
	tDoMassEstimation->setValue(false);
	//controller->toggleMassEstimation(false);

	return true;
}

void Ground::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void Ground::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// Activate MassEstimation
	//controller->toggleMassEstimation(true);
	tDoMassEstimation->setValue(true);
}

void Ground::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Ground::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Ground::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setAcceleration( Acceleration3D(0.0, 0.0, GRAVITY) );
	generatedTrajInput.setYawRate(0.0);
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Ground::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Ground::isValid(const TKState& currentState) const
{
	// never turns invalid
	return true;
}



}
