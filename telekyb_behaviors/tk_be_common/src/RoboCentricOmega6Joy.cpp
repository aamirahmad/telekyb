/*
 * RoboCentricOmega6Joy.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: modelga
 */

#include "RoboCentricOmega6Joy.hpp"
// #include <../../opt/ros/fuerte/include/ros/time.h>

#include <telekyb_base/ROS.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::RoboCentricOmega6Joy, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

RoboCentricOmega6Joy::RoboCentricOmega6Joy()
	: Behavior("tk_be_common/RoboCentricOmega6Joy", BehaviorType::Air)
{

}

void RoboCentricOmega6Joy::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
//	ROS_INFO("Received RoboCentricOmega6Joy CB");

	if (msg->axes.size() < 4) {
		ROS_ERROR("We need at least 4 Axes!");
		return;
	}

	if (msg->buttons.size() < 2) {
		ROS_ERROR("We need at least 2 Buttons!");
		return;
	}
	
	
	deadManPressed = (bool)msg->buttons[0];
	currentTime = ros::Time::now().toSec();
	
	//dead man button is pressed, or we do not care about the dead man button
// 	if ( deadManPressed || !tRoboCentricOmega6JoyUseDeadManSwitch->getValue()) {
// 		lastVelocityInput = Velocity3D(msg->axes[0],msg->axes[1],msg->axes[2]);
// 		if (msg->axes.size() != 4) {
// 			lastYawRateInput = (msg->axes[4] - msg->axes[3])/2.0 * tRoboCentricOmega6JoyYawRateScale->getValue() /*+ tSinAmplitude->getValue()*sin(  tSinPulse->getValue()*(currentTime - lastTime))*/;
// 		} else {
// 			// 4 axes case
// 			lastYawRateInput = msg->axes[3] * tRoboCentricOmega6JoyYawRateScale->getValue() ;
// 		}
// 	}

	// invalidate
	if ( msg->buttons[1] ) {
		valid = false;
	}
}



void RoboCentricOmega6Joy::userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	currentTime = ros::Time::now().toSec();
	// no dead man button
	if (!deadManPressed) {
		// tese two values have been sperimentally estimated 
		//double estimatedAmplitudeCorrection = 0.0;//0.6328;
		//double estimatedPhaseCorrection = 0.13; 
		
		//float velocityPhaseCorrection = estimatedAmplitudeCorrection*tSinAmplitude->getValue()/tSinPulse->getValue()*cos(tSinPulse->getValue()* (currentTime - lastTime - estimatedPhaseCorrection));
		//std_msgs::Float64 velPhaseCorrection;
		//velPhaseCorrection.data = velocityPhaseCorrection;
		//yawSinComponent.data = tSinAmplitude->getValue()*sin(  tSinPulse->getValue()*(currentTime - lastTime));
		
		lastVelocityInput = Velocity3D(-msg->vector.x*cos(M_PI/4), -msg->vector.x*sin(M_PI/4), msg->vector.z);
		lastYawRateInput = - msg->vector.y;
		
		commandedYawRate.data = -msg->vector.y;
		//velPhaseCorrectionPublisher.publish(velPhaseCorrection);
		commandedYawRatePublisher.publish(commandedYawRate);
		//yawSinComponentPublisher.publish(yawSinComponent);
	}
	else {
		lastVelocityInput = Velocity3D(0.0, 0.0, 0.0);
		lastYawRateInput = 0.0;
	}
}

void RoboCentricOmega6Joy::initialize()
{
	tRoboCentricOmega6JoyTopic = addOption<std::string>("tOmega6JoyTopic",
			"RoboCentricOmega6JoyTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);
	
	tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic",
			"Topic to be published when the sinusoid is added to the yaw motion",
			"/yawSinComponent", false, false);
	tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic",
			"topic of the commanded yaw rate by the operator",
			"/commandedYawRate", false, false);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);

	tRoboCentricOmega6JoyUsePositionMode = addOption("tRoboCentricOmega6JoyUsePositionMode",
			"Integrates Position from Velocity input.", true, false, false);
	tRoboCentricOmega6JoyYawRateScale = addOption("tRoboCentricOmega6JoyYawRateScale",
			"Commanded Yaw Rate is scaled to this value", 1.0, false ,false);
	tSinPulse = addOption("tSinPulse",
			"Pulse for the yaw motion sinusoid", 1.0, false ,false);
	tSinAmplitude = addOption("tSinAmplitude",
			"Amplitude for the yaw motion sinusoid", 0.5, false ,false);
	tRoboCentricOmega6JoyUseRelativeMode = addOption("tRoboCentricOmega6JoyUseRelativeMode",
			"Enable this to have all inputs interpreted w.r.t. the local frame.", false, false, true);
	tRoboCentricOmega6JoyUseDeadManSwitch = addOption("tRoboCentricOmega6JoyUseDeadManSwitch",
			"Disable this to fly without pressing a special button simultaneously.", true, false, true);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	
	lastTime = ros::Time::now().toSec();;

	// no Parameters
	//parameterInitialized = true;
}

void RoboCentricOmega6Joy::destroy()
{

}

bool RoboCentricOmega6Joy::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	joySub = nodeHandle.subscribe(tRoboCentricOmega6JoyTopic->getValue()
			, 10, &RoboCentricOmega6Joy::joystickCB, this);
	
	
// 	std::cout << "  !!!!!!!!!!!!!!  " << tYawSinComponentTopic->getValue() << std::endl;
// 	std::cout << "  !!!!!!!!!!!!!!  " << tCommandedYawRateTopic->getValue() << std::endl;
	
	//yawSinComponentPublisher = nodeHandle.advertise<std_msgs::Float64>(tYawSinComponentTopic->getValue(), 1);
	commandedYawRatePublisher = nodeHandle.advertise<std_msgs::Float64>(tCommandedYawRateTopic->getValue(), 1);
	//velPhaseCorrectionPublisher = nodeHandle.advertise<std_msgs::Float64>("/velPhaseCorrection", 1);
	
	
		
	std::cout << "topic: " << tVelocityInputTopic->getValue() << std::endl;

	
	userInputSub = nodeHandle.subscribe(tVelocityInputTopic->getValue(), 1, &RoboCentricOmega6Joy::userVelocityCB, this);

	lastVelocityInput = Velocity3D::Zero();
	lastYawRateInput = 0.0;

	if (tRoboCentricOmega6JoyUsePositionMode->getValue()) {
		posModeCurPosition = currentState.position;
		posModeCurYawAngle = currentState.getEulerRPY()(2);
		posModeLastInputTime = Time();
	}

	
	deadManPressed = false;
	valid = true;

	return true;
}

void RoboCentricOmega6Joy::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void RoboCentricOmega6Joy::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void RoboCentricOmega6Joy::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	joySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void RoboCentricOmega6Joy::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void RoboCentricOmega6Joy::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{

	// set Velocity Mode
	//Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
	//Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
	//generatedTrajInput.setVelocity(newVel);
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void RoboCentricOmega6Joy::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool RoboCentricOmega6Joy::isValid(const TKState& currentState) const
{
	// never turns invalid
	return valid;
}


}
