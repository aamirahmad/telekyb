/*
 * Omega6Joy.cpp
 *
 *  Created on: Nov 12, 2011
 *      Author: mriedel
 */

#include "Omega6Joy.hpp"
#include <ros/time.h>

#include <telekyb_base/ROS.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Omega6Joy, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Omega6Joy::Omega6Joy()
	: Behavior("tk_be_common/Omega6Joy", BehaviorType::Air)
{

}

void Omega6Joy::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
//	ROS_INFO("Received Omega6Joy CB");

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
	if ( deadManPressed || !tOmega6JoyUseDeadManSwitch->getValue()) {
		lastVelocityInput = Velocity3D(msg->axes[0],msg->axes[1],msg->axes[2]);
		if (msg->axes.size() != 4) {
			lastYawRateInput = (msg->axes[4] - msg->axes[3])/2.0 * tOmega6JoyYawRateScale->getValue() /*+ tSinAmplitude->getValue()*sin(  tSinPulse->getValue()*(currentTime - lastTime))*/;
		} else {
			// 4 axes case
			lastYawRateInput = msg->axes[3] * tOmega6JoyYawRateScale->getValue() ;
		}
	}

	// invalidate
	if ( msg->buttons[1] ) {
		valid = false;
	}
}



void Omega6Joy::userVelocityCB(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
	currentTime = ros::Time::now().toSec();
	// no dead man button
	if (!deadManPressed) {
		// tese two values have been sperimentally estimated 
		double estimatedAmplitudeCorrection = 0.6328;
		double estimatedPhaseCorrection = 0.13; 
		
		float velocityPhaseCorrection = estimatedAmplitudeCorrection*tSinAmplitude->getValue()/tSinPulse->getValue()*cos(tSinPulse->getValue()* (currentTime - lastTime - estimatedPhaseCorrection));
		std_msgs::Float64 velPhaseCorrection;
		velPhaseCorrection.data = velocityPhaseCorrection;
		yawSinComponent.data = tSinAmplitude->getValue()*sin(  tSinPulse->getValue()*(currentTime - lastTime));
		
		lastVelocityInput = Velocity3D(-msg->vector.x*cos(M_PI/4 + velocityPhaseCorrection), -msg->vector.x*sin(M_PI/4  + velocityPhaseCorrection), msg->vector.z);
		lastYawRateInput = - msg->vector.y + yawSinComponent.data;
		
		commandedYawRate.data = -msg->vector.y;
		velPhaseCorrectionPublisher.publish(velPhaseCorrection);
		commandedYawRatePublisher.publish(commandedYawRate);
		yawSinComponentPublisher.publish(yawSinComponent);
	} 
}

void Omega6Joy::initialize()
{
	tOmega6JoyTopic = addOption<std::string>("tOmega6JoyTopic",
			"Omega6JoyTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", false, false);
	
	tYawSinComponentTopic = addOption<std::string>("tYawSinComponentTopic",
			"Topic to be published when the sinusoid is added to the yaw motion",
			"/yawSinComponent", false, false);
	tCommandedYawRateTopic = addOption<std::string>("tCommandedYawRateTopic",
			"topic of the commanded yaw rate by the operator",
			"/commandedYawRate", false, false);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic", "Topic Name of Input of User Velocity",
			"undef", false, false);

	tOmega6JoyUsePositionMode = addOption("tOmega6JoyUsePositionMode",
			"Integrates Position from Velocity input.", true, false, false);
	tOmega6JoyYawRateScale = addOption("tOmega6JoyYawRateScale",
			"Commanded Yaw Rate is scaled to this value", 1.0, false ,false);
	tSinPulse = addOption("tSinPulse",
			"Pulse for the yaw motion sinusoid", 1.0, false ,false);
	tSinAmplitude = addOption("tSinAmplitude",
			"Amplitude for the yaw motion sinusoid", 0.5, false ,false);
	tOmega6JoyUseRelativeMode = addOption("tOmega6JoyUseRelativeMode",
			"Enable this to have all inputs interpreted w.r.t. the local frame.", false, false, true);
	tOmega6JoyUseDeadManSwitch = addOption("tOmega6JoyUseDeadManSwitch",
			"Disable this to fly without pressing a special button simultaneously.", true, false, true);

	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	
	lastTime = ros::Time::now().toSec();;

	// no Parameters
	//parameterInitialized = true;
}

void Omega6Joy::destroy()
{

}

bool Omega6Joy::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	joySub = nodeHandle.subscribe(tOmega6JoyTopic->getValue()
			, 10, &Omega6Joy::joystickCB, this);
	
	
	std::cout << "  !!!!!!!!!!!!!!  " << tYawSinComponentTopic->getValue() << std::endl;
	std::cout << "  !!!!!!!!!!!!!!  " << tCommandedYawRateTopic->getValue() << std::endl;
	
	yawSinComponentPublisher = nodeHandle.advertise<std_msgs::Float64>(tYawSinComponentTopic->getValue(), 1);
	commandedYawRatePublisher = nodeHandle.advertise<std_msgs::Float64>(tCommandedYawRateTopic->getValue(), 1);
	velPhaseCorrectionPublisher = nodeHandle.advertise<std_msgs::Float64>("/velPhaseCorrection", 1);
	
	userInputSub = nodeHandle.subscribe(tVelocityInputTopic->getValue(), 1, &Omega6Joy::userVelocityCB, this);

	lastVelocityInput = Velocity3D::Zero();
	lastYawRateInput = 0.0;

	if (tOmega6JoyUsePositionMode->getValue()) {
		posModeCurPosition = currentState.position;
		posModeCurYawAngle = currentState.getEulerRPY()(2);
		posModeLastInputTime = Time();
	}

	
	deadManPressed = false;
	valid = true;

	return true;
}

void Omega6Joy::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	// not used
}

void Omega6Joy::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void Omega6Joy::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	joySub.shutdown();
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void Omega6Joy::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Omega6Joy::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);

		// set Velocity Mode
		if (tOmega6JoyUsePositionMode->getValue()) {
// 			std::cout << "P" << std::endl;
			// integrate Position
			double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
	//		ROS_INFO("timeDiffSec: %f", timeDiffSec);
			posModeLastInputTime = Time();
			if (tOmega6JoyUseRelativeMode->getValue()) {
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				posModeCurPosition += (newVel * timeDiffSec);
			} else {
				//all is w.r.t. world frame
				posModeCurPosition += (lastVelocityInput * timeDiffSec);
			}

			posModeCurYawAngle += (lastYawRateInput * timeDiffSec);

	//		ROS_INFO_STREAM("Position: " << std::endl << posModeCurPosition);
	//		ROS_INFO_STREAM("YawAngle: " << posModeCurYawAngle.dCast());

			if (tOmega6JoyUseRelativeMode->getValue()) {
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				generatedTrajInput.setPosition(posModeCurPosition,newVel);
				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
			} else {
				//all is w.r.t. world frame
				generatedTrajInput.setPosition(posModeCurPosition, lastVelocityInput);
				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);
				
			}

		} else {
// 			std::cout << "V" ;
			if (tOmega6JoyUseRelativeMode->getValue()) {
// 				std::cout << "1" ;
				//all is w.r.t. local frame
				Eigen::Vector2d partial = currentState.orientation.toRotationMatrix().block<2,2>(0,0) * lastVelocityInput.block<2,1>(0,0);
				Eigen::Vector3d newVel(partial(0), partial(1), lastVelocityInput(2));
				generatedTrajInput.setVelocity(newVel);
				generatedTrajInput.setYawRate(lastYawRateInput);
			} else{
// 				std::cout << "2" ;
				//all is w.r.t. world frame
				generatedTrajInput.setVelocity(lastVelocityInput);
				generatedTrajInput.setYawRate(lastYawRateInput);// ### RE-ENABLE THIS
// 				generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput); // ### ELIMINATE THIS!!!!!!!
			}
// 			std::cout << std::endl;
		}
}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void Omega6Joy::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Omega6Joy::isValid(const TKState& currentState) const
{
	// never turns invalid
	return valid;
}


}
