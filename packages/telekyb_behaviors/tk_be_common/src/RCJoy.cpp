/*
 * RCJoy.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: pstegagno
 */

#include "RCJoy.hpp"

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::RCJoy, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

RCJoy::RCJoy()
	: Behavior("tk_be_common/RCJoy", BehaviorType::Air)
{

}

void RCJoy::RCjoyCB(const sensor_msgs::Joy::ConstPtr& msg)
{
//	ROS_INFO("Received RCJoy CB");

	if (msg->axes.size() < 4) {
		ROS_ERROR("We need at least 4 Axes!");
		return;
	}

	if (msg->buttons.size() < 2) {
		ROS_ERROR("We need at least 2 Buttons!");
		return;
	}

	//dead man button is pressed, or we do not care about the dead man button
	if ( msg->buttons[0] || !tRCJoyUseDeadManSwitch->getValue()) {
		lastVelocityInput = Velocity3D(msg->axes[0],msg->axes[1],msg->axes[2]);
		if (msg->axes.size() != 4) {
			lastYawRateInput = (msg->axes[4] - msg->axes[3])/2.0 * tRCJoyYawRateScale->getValue();
		} else {
			// 4 axes case
			lastYawRateInput = msg->axes[3] * tRCJoyYawRateScale->getValue();
		}
	} else {
		// apply 0
		lastVelocityInput = Velocity3D::Zero();
		lastYawRateInput = 0.0;
	}


	// invalidate
	if ( msg->buttons[1] ) {
		valid = false;
	}
	
	
}

void RCJoy::initialize()
{
	tRCJoyTopic = addOption<std::string>("tRCJoyTopic",
			"RCJoyTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);

// 	tRCJoyUsePositionMode = addOption("tRCJoyUsePositionMode",
// 			"Integrates Position from Velocity input.", true, false, false);
	tRCJoyYawRateScale = addOption("tRCJoyYawRateScale",
			"Commanded Yaw Rate is scaled to this value", 1.0, false ,false);

// 	tRCJoyUseRelativeMode = addOption("tRCJoyUseRelativeMode",
// 			"Enable this to have all inputs interpreted w.r.t. the local frame.", false, false, true);
	tRCJoyUseDeadManSwitch = addOption("tRCJoyUseDeadManSwitch",
			"Disable this to fly without pressing a special button simultaneously.", true, false, true);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	// no Parameters
	//parameterInitialized = true;
}

void RCJoy::destroy()
{

}

bool RCJoy::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	joySub = nodeHandle.subscribe(tRCJoyTopic->getValue()
			, 10, &RCJoy::RCjoyCB, this);

	lastVelocityInput = Velocity3D::Zero();
	lastYawRateInput = 0.0;

	if (tRCJoyUsePositionMode->getValue()) {
		posModeCurPosition = currentState.position;
		posModeCurYawAngle = currentState.getEulerRPY()(2);
		posModeLastInputTime = Time();
	}

	valid = true;

	return true;
}

void RCJoy::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void RCJoy::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void RCJoy::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	joySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void RCJoy::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void RCJoy::trajectoryStepActive(TKTrajectory& generatedTrajInput)
{
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);
		
		generatedTrajInput.setRollAngle(lastVelocityInput(0));
		generatedTrajInput.setPitchAngle(lastVelocityInput(1));
		generatedTrajInput.setThrust(lastVelocityInput(2));
		generatedTrajInput.setYawRate(lastYawRateInput);
// 		// set Velocity Mode
// 		if (tRCJoyUsePositionMode->getValue()) {
// 			// integrate Position
// 			double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
// 	//		ROS_INFO("timeDiffSec: %f", timeDiffSec);
// 			posModeLastInputTime = Time();
// 			if (tRCJoyUseRelativeMode->getValue()) {
// 				//all is w.r.t. local frame
// 				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
// 				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
// 				posModeCurPosition += (newVel * timeDiffSec);
// 			} else {
// 				//all is w.r.t. world frame
// 				posModeCurPosition += (lastVelocityInput * timeDiffSec);
// 			}
// 
// 			posModeCurYawAngle += (lastYawRateInput * timeDiffSec);
// 
// 	//		ROS_INFO_STREAM("Position: " << std::endl << posModeCurPosition);
// 	//		ROS_INFO_STREAM("YawAngle: " << posModeCurYawAngle.dCast());
// 
// 			if (tRCJoyUseRelativeMode->getValue()) {
// 				//all is w.r.t. local frame
// 				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
// 				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
// 				generatedTrajInput.setPosition(posModeCurPosition,newVel);
// 				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
// 			} else {
// 				//all is w.r.t. world frame
// 				generatedTrajInput.setPosition(posModeCurPosition, lastVelocityInput);
// 				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
// 			}
// 
// 		} else {
// 			if (tRCJoyUseRelativeMode->getValue()) {
// 				//all is w.r.t. local frame
// 				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
// 				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
// 				generatedTrajInput.setVelocity(newVel);
// 				generatedTrajInput.setYawRate(lastYawRateInput);
// 			} else{
// 				//all is w.r.t. world frame
// 				generatedTrajInput.setVelocity(lastVelocityInput);
// 				generatedTrajInput.setYawRate(lastYawRateInput);
// 			}
// 		}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void RCJoy::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool RCJoy::isValid(const TKState& currentState) const
{
	// never turns invalid
	return valid;
}


}
