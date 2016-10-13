/*
 * Teleoperation.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: siddian
 */

#include "Teleoperation.hpp"

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::Teleoperation, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

Teleoperation::Teleoperation()
: Behavior("tk_be_common/Teleoperation", BehaviorType::Air),
  nodeHandle( TELEKYB_NAMESPACE::ROSModule::Instance().getMainNodeHandle() ),
  tJoystickTopic(NULL),
  tRollInputScaling(NULL)
{
	valid = false;
	lastYawRateInput = 0;
}

Teleoperation::~Teleoperation() {
}

void Teleoperation::initialize() {
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"JoystickTopic that published sensor_msgs::Joy",
			"/TeleKyb/tJoy/joy", true, false);
	tRollInputScaling = addOption<double>("tRollInputScaling",
			"defines how the values from the sticks are scaled before using them as input.",
			0.2, true, false);
}

void Teleoperation::destroy() {

}

void Teleoperation::joystickCB(const sensor_msgs::Joy::ConstPtr& msg) {

	//	ROS_INFO("Received Joystick CB");

	if (msg->axes.size() < 4) {
		ROS_ERROR("We need at least 4 Axes!");
		return;
	}

	if (msg->buttons.size() < 2) {
		ROS_ERROR("We need at least 2 Buttons!");
		return;
	}

	// invalidate
	if ( msg->buttons[1] ) {
		valid = false;
	}
}
void Teleoperation::WittensteinCB(const sensor_msgs::Joy::ConstPtr& msg) {
//	std::cout << "wittenstein axis: " << msg->axes[0] * tRollInputScaling->getValue() << std::endl;

//	lastVelocityInput = Velocity3D(msg->axes[0],msg->axes[1],msg->axes[2]);
//	lastVelocityInput = Velocity3D(msg->axes[0],0,0);
	lastVelocityInput = Velocity3D(0,0,0);
	//we will use the Acc input for defining the desired roll angle to be commanded later by the
	//TeleoperationTrajectoryTracker
	lastAccInput = Eigen::Vector3d(msg->axes[0] * tRollInputScaling->getValue(),0,0);
	// in order to identify whether we want the acc to be used as roll, define the elements of jerk > 0
	lastJerkInput = Eigen::Vector3d(1,0,0);

//	if (msg->axes.size() != 4) {
//		lastYawRateInput = (msg->axes[4] - msg->axes[3])/2.0;// * tJoystickYawRateScale->getValue();
//	} else {
//		// 4 axes case
//		lastYawRateInput = msg->axes[3];// * tJoystickYawRateScale->getValue();
//	}
	lastYawRateInput = 0;

}

bool Teleoperation::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior) {
	joySub = nodeHandle.subscribe(tJoystickTopic->getValue(), 10, &Teleoperation::joystickCB, this);
	WSSub = nodeHandle.subscribe("/TeleKyb/tJoy/Wittensteins", 1, &Teleoperation::WittensteinCB, this);

	lastVelocityInput = Velocity3D::Zero();
	lastAccInput = Velocity3D::Zero();
	lastYawRateInput = 0.0;
	posModeCurPosition = currentState.position;
	posModeCurYawAngle = currentState.getEulerRPY()(2);
	posModeLastInputTime = Time();

	valid = true;

	return true;
}

void Teleoperation::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior) {
	// not used
}

void Teleoperation::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior) {
	// not used
}

void Teleoperation::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior) {
	joySub.shutdown();
	WSSub.shutdown();
}

// called every time a new TKState is available AND it is the NEW Behavior of an active Switch
void Teleoperation::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput) {
	generatedTrajInput.setVelocity(lastVelocityInput);
	generatedTrajInput.setYawRate(lastYawRateInput);
}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void Teleoperation::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput) {
	//	ROS_INFO_STREAM("InputVel: " << std::endl << lastVelocityInput);
	//	ROS_INFO_STREAM("InputYawRate: " << lastYawRateInput);

//	std::cout << "P" << std::endl;
	// integrate Position
	double timeDiffSec = (Time() - posModeLastInputTime).toDSec();
	//		ROS_INFO("timeDiffSec: %f", timeDiffSec);
	posModeLastInputTime = Time();

	//all is w.r.t. world frame
	posModeCurPosition += (lastVelocityInput * timeDiffSec);

//	posModeCurYawAngle += (lastYawRateInput * timeDiffSec);
	posModeCurYawAngle = M_PI_2;

	//		ROS_INFO_STREAM("Position: " << std::endl << posModeCurPosition);
//	ROS_INFO_STREAM("YawAngle: " << posModeCurYawAngle.dCastPi(0));

	//all is w.r.t. world frame
	generatedTrajInput.setPosition(posModeCurPosition, lastVelocityInput, lastAccInput, lastJerkInput);
	generatedTrajInput.setYawAngle(posModeCurYawAngle.dCastPi(0), lastYawRateInput);

}

// called every time a new TKState is available AND it is the OLD Behavior of an active Switch
void Teleoperation::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput) {
	generatedTrajInput.setVelocity( Velocity3D::Zero() );
	generatedTrajInput.setYawRate( 0.0 );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool Teleoperation::isValid(const TKState& currentState) const {
	return true;
}

}
