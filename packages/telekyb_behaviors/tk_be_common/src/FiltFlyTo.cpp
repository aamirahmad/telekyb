/*
 * FiltFlyTo.cpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#include "FiltFlyTo.hpp"

#include <boost/numeric/odeint.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_behavior::FiltFlyTo, TELEKYB_NAMESPACE::Behavior);

namespace telekyb_behavior {

FiltFlyTo::FiltFlyTo()
	: Behavior("tk_be_common/FiltFlyTo", BehaviorType::Undef)
{

}

void FiltFlyTo::initialize()
{
	tFlyToDestination = addOption<Position3D>("tFlyToDestination",
			"Specifies the Destination of the FiltFlyTo Behavior",
			Position3D(0.0,0.0,-1.0), false, false);
	tFlyToYawDestination = addOption<double>("tFlyToYawDestination",
			"Specifies the Yaw Destination of the FiltFlyTo Behavior",
			0, false, false);

	tGroundHeight = addOption<double>("tGroundHeight",
				"Specifies the z value at which the behavior turns invalid",
				-.2, false, false);

	tSmoothingGains = addOption<Eigen::Vector4d>("tSmoothingGains", "Smoothing gains for linear trajectory", Eigen::Vector4d(4.060000275093721,6.181100841145216,4.182206857259342,1.061106291208401), false, true);
	tSmoothingGainsYaw = addOption<Eigen::Vector2d>("tSmoothingGainsYaw", "Smoothing gains for the yaw angle", Eigen::Vector2d(2.010000000000018,1.010000000000018), false, true);

	tPublishSmoothedTrajectory = addOption<bool>("tPublishSmoothedTrajectory", "Republish smoothed trajectory", false, false, true);
	tSmoothedTrajectoryTopic = addOption<std::string>("tSmoothedTrajectoryTopic", "Smoothed trajectory topic name", "SmoothedTrajectory", false, true);
}

void FiltFlyTo::destroy()
{

}

bool FiltFlyTo::willBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	firstStep = true;
	return true;
}

void FiltFlyTo::didBecomeActive(const TKState& currentState, const Behavior& previousBehavior)
{
	ROS_INFO("Flying to: [%f, %f, %f | %f ]", tFlyToDestination->getValue()(0)
			, tFlyToDestination->getValue()(1)
			, tFlyToDestination->getValue()(2)
			, tFlyToYawDestination->getValue());
	// not used
}

void FiltFlyTo::willBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

void FiltFlyTo::didBecomeInActive(const TKState& currentState, const Behavior& nextBehavior)
{
	// not used
}

// called everytime a new TKState is available AND it is the NEW Behavior of an active Switch
void FiltFlyTo::trajectoryStepCreation(const TKState& currentState, TKTrajectory& generatedTrajInput)
{

}

// called everytime a new TKState is available. Should return false if invalid (swtich to next behavior, or Hover if undef).
void FiltFlyTo::trajectoryStepActive(const TKState& currentState, TKTrajectory& generatedTrajInput)
{

	double timeStep = timer.getElapsed().toDSec();
	timer.reset();

	Eigen::Vector3d snap = Eigen::Vector3d::Zero();
	double yawAcceleration = 0.0;

	if (firstStep || timeStep > 0.1){
		ROS_INFO("Resetting trajectory smoother internal state");
		filterState = QuadrupleIntegratorState(currentState);
		firstStep = false;
	} else {

		Eigen::Vector3d saturationSnap = Eigen::Vector3d::Zero();

		snap =
			- tSmoothingGains->getValue()(0) * (filterState.jerk)
			- tSmoothingGains->getValue()(1) * (filterState.acceleration)
			- tSmoothingGains->getValue()(2) * (filterState.velocity)
			- tSmoothingGains->getValue()(3) * (filterState.position - tFlyToDestination->getValue())
			- saturationSnap;

		const double yawAngleError = fmod(filterState.yawAngle -tFlyToYawDestination->getValue() + M_PI, 2*M_PI) - M_PI;

//		const double yawAngleError = atan2(sin(filterState.yawAngle -tFlyToYawDestination->getValue()), cos(filterState.yawAngle -tFlyToYawDestination->getValue()));

//		std::cout << "yawAngleError: " << yawAngleError << ", " <<  << std::endl;

		yawAcceleration =
				- tSmoothingGainsYaw->getValue()(0) * (filterState.yawRate)
				- tSmoothingGainsYaw->getValue()(1) * (yawAngleError);

		filter.setInput(snap, yawAcceleration);

		typedef boost::numeric::odeint::runge_kutta4< QuadrupleIntegratorState, double, QuadrupleIntegratorState, double, boost::numeric::odeint::vector_space_algebra> stepper;
		boost::numeric::odeint::integrate_const( stepper() , filter, filterState , 0.0, timeStep, timeStep);

	}

	generatedTrajInput.setPosition(filterState.position,
			filterState.velocity,
			filterState.acceleration,
			filterState.jerk,
			snap);
	generatedTrajInput.setYawAngle(filterState.yawAngle, filterState.yawRate, yawAcceleration);

}

// called everytime a new TKState is available AND it is the OLD Behavior of an active Switch
void FiltFlyTo::trajectoryStepTermination(const TKState& currentState, TKTrajectory& generatedTrajInput)
{
	generatedTrajInput.setPosition( filterState.position );
	generatedTrajInput.setYawAngle( filterState.yawAngle );
}

// Return true if the active Behavior is (still) valid. Initiate Switch otherwise
bool FiltFlyTo::isValid(const TKState& currentState) const
{
//	Vector3D direction = tFlyToDestination->getValue() - currentState.position;
//	return direction.norm() > tFlyToInnerDestinationRadius->getValue();
	if (currentState.position(2) < tGroundHeight->getValue()){
		return true;
	} else {
		return false;
	}


}


} /* namespace telekyb_behavior */

