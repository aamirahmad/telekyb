/*
 * HoverBehavior.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <behaviors/Hover.hpp>

#define MAX_INITIAL_VELOCITY 0.05


// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Hover, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Hover::Hover()
	: Behavior("tk_behavior/Hover",  BehaviorType::Air)
{

}

void Hover::initialize()
{
	// the Standard Brake Minimal Value should be significantly below this value!
	tHoverMaxInitialVelocity = addOption<double>("tHoverMaxInitialVelocity",
			"Defines the Maximal Initial Velocity to be able to switch into Hover",
			0.2, false, true);
	tHoverVelocityMode = addOption<bool>("tHoverVelocityMode",
			"Hover in velocity mode",
			false, false, false);
	//options = new HoverOptions(this);

	// can work with default parameters
	parameterInitialized = true;
}

void Hover::destroy()
{

}


bool Hover::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// Error if too fast ? Put as parameter?
	if (currentState.linVelocity.norm() > tHoverMaxInitialVelocity->getValue()) {
		ROS_ERROR_STREAM("Too fast to switch into Hover! Current Velocity Norm: " << currentState.linVelocity.norm());
		return false;
	}

	hoverPosition = currentState.position;
	yawAngle = currentState.getEulerRPY()(2);

	// from Martin:
	//ROS_INFO_STREAM("Setting Position: " << std::endl << currentState.position << " Angle: " << currentState.getEulerRPY()(2));
	//ROS_INFO("Hover: Storing yawAngle: %f ", yawAngle);

	// Thomas: changed to one line output
	ROS_INFO_STREAM("Setting Position: (" << currentState.position(0) << ", " << currentState.position(1) << ", " << currentState.position(2) << "), " << " Angle: " << currentState.getEulerRPY()(2));

	return true;
}

void Hover::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
//	ROS_ERROR("Sleeping after Switch to Hover.");
//	usleep(2*1000*1000); // 10 sec
}

void Hover::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void Hover::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Hover::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Hover::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	if (tHoverVelocityMode->getValue()) {
			generatedTrajInput.setVelocity(Velocity3D::Zero());
			generatedTrajInput.setYawRate(0.0);
	} else {
			generatedTrajInput.setPosition(hoverPosition);
			generatedTrajInput.setYawAngle(yawAngle);
	}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Hover::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Hover::isValid(const TKState& currentState) const
{
	// never turns invalid
	return true;
}



}
