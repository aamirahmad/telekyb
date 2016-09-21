/*
 * FixedPointHoverBehavior.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include "FixedPointHover.hpp"

#define MAX_INITIAL_VELOCITY 0.05


// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::FixedPointHover, TELEKYB_NAMESPACE::Behavior)

namespace telekyb_behavior {

FixedPointHover::FixedPointHover()
    : Behavior("tk_be_common/FixedPointHover",  BehaviorType::Air)
{

}

void FixedPointHover::initialize()
{
	// the Standard Brake Minimal Value should be significantly below this value!
	tFixedPointHoverMaxInitialVelocity = addOption<double>("tFixedPointHoverMaxInitialVelocity",
			"Defines the Maximal Initial Velocity to be able to switch into FixedPointHover",
			0.2, false, true);
	tFixedPointHoverVelocityMode = addOption<bool>("tFixedPointHoverVelocityMode",
			"FixedPointHover in velocity mode",
			false, false, false);
	tFixedPointHoverHoveringPosition = addOption<Position3D>("tFixedPointHoverHoveringPosition",
			"Hoverin a predertermined position",
			Position3D(0.0, 0.0, -1.0), false, false);
    //options = new FixedPointHoverOptions(this);
    tFixedPointHoverHoveringOrientation = addOption<double>("tFixedPointHoverHoveringOrientation",
            "Hoverin at a predertermined orientation",
            0.0, false, false);


	// can work with default parameters
	parameterInitialized = true;
}

void FixedPointHover::destroy()
{

}


bool FixedPointHover::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// Error if too fast ? Put as parameter?
	if (currentState.linVelocity.norm() > tFixedPointHoverMaxInitialVelocity->getValue()) {
        ROS_ERROR_STREAM_THROTTLE(1,"Too fast to switch into FixedPointHover! Current Velocity Norm: " << currentState.linVelocity.norm());
		return false;
	}

	
// 	hoverPosition = tFixedPointHoverHoveringPosition->getValue();
// 	yawAngle = currentState.getEulerRPY()(2);
	
	hoverPosition = tFixedPointHoverHoveringPosition->getValue();
    yawAngle = tFixedPointHoverHoveringOrientation->getValue();

	// from Martin:
	//ROS_INFO_STREAM("Setting Position: " << std::endl << currentState.position << " Angle: " << currentState.getEulerRPY()(2));
	//ROS_INFO("FixedPointHover: Storing yawAngle: %f ", yawAngle);

	// Thomas: changed to one line output
	ROS_INFO_STREAM("Setting Position: (" << hoverPosition(0) << ", " << hoverPosition(1) << ", " << hoverPosition(2) << "), " << " Angle: " << currentState.getEulerRPY()(2));

	return true;
}

void FixedPointHover::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
//	ROS_ERROR("Sleeping after Switch to FixedPointHover.");
//	usleep(2*1000*1000); // 10 sec
}

void FixedPointHover::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
    // not used
}

void FixedPointHover::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void FixedPointHover::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or FixedPointHover if undef).
void FixedPointHover::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
    hoverPosition = tFixedPointHoverHoveringPosition->getValue();
    
    generatedTrajInput.setPosition(hoverPosition);
    generatedTrajInput.setYawAngle(yawAngle);
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void FixedPointHover::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	// Same as active
	trajectoryStepActive(currentState,generatedTrajInput);
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool FixedPointHover::isValid(const TKState& currentState) const
{
	// never turns invalid
	return true;
}



}
