/*
 * VMTrajectorySmoothing.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: rspica
 */

#include <trajmodules/VMTrajectorySmoothing.hpp>

#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

#include <boost/numeric/odeint.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::VMTrajectorySmoothing, TELEKYB_NAMESPACE::TrajectoryModule);

namespace telekyb_traj {

VMTrajectorySmoothingOptions::VMTrajectorySmoothingOptions()
	: OptionContainer("VMTrajectorySmoothing")
{
	tSaturationBegin = addOption("tSaturationBegin", "Value at which linear acceleration saturation must be started", 0.6, false, true);
	tSaturationFull = addOption("tSaturationFull", "Value at which linear acceleration saturation is full", 0.75, false, true);
	tSaturationLimit = addOption("tSaturationLimit", "Maximum allowed linear acceleration", 100.0, false, true);
	tSaturationPotentialGain = addOption("tSaturationPotentialGain", "Gain for linear acceleration saturation potential", 5.0, false, true);
	tSaturateAcceleration = addOption<bool>("tSaturateAcceleration", "Saturate linear acceleration", false, false, true);
	tSmoothingGains = addOption<Eigen::Vector3d>("tSmoothingGains", "Smoothing gains for linear trajectory", Eigen::Vector3d(2.200000000002267e+02,1.310000000002191e+04,1.100000000001940e+05), false, true);
	tSmoothingGainsYaw = addOption<double>("tSmoothingGainsYaw", "Smoothing gains for the yaw angle", 10.0, false, true);

	tPublishSmoothedTrajectory = addOption<bool>("tPublishSmoothedTrajectory", "Republish smoothed trajectory", false, false, true);
	tSmoothedTrajectoryTopic = addOption<std::string>("tSmoothedTrajectoryTopic", "Smoothed trajectory topic name", "SmoothedTrajectory", false, true);
}

VMTrajectorySmoothing::VMTrajectorySmoothing()
	: TrajectoryModule("tk_trajprocessor/VMTrajectorySmoothing", TrajModulePosType::Velocity, 110), // after obs avoid
	  firstStep(true),
	  nodeHandle(ROSModule::Instance().getMainNodeHandle()){
}

void VMTrajectorySmoothing::initialize()
{

}

void VMTrajectorySmoothing::destroy()
{

}

// set back to intial conditions
void VMTrajectorySmoothing::willTurnActive()
{
	if (options.tPublishSmoothedTrajectory->getValue()){
		smoothTrajPub = nodeHandle.advertise<telekyb_msgs::TKTrajectory>(options.tSmoothedTrajectoryTopic->getValue(), 10);
	}
	ROS_INFO("VMTrajectorySmoothing did turn active!");
	firstStep = true;
}

// called after turning inactive
void VMTrajectorySmoothing::didTurnInactive()
{
	ROS_INFO("VMTrajectorySmoothing did turn inactive!");
}

bool VMTrajectorySmoothing::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
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
		if (options.tSaturateAcceleration->getValue()){
			const double accelerationNorm = trajInput.acceleration.norm();

			if (accelerationNorm > options.tSaturationBegin->getValue()){
				ROS_INFO("saturation applied");
				double potential;
				if (accelerationNorm < options.tSaturationFull->getValue()){
					potential = options.tSaturationPotentialGain->getValue()*tan(M_PI_2*(accelerationNorm-options.tSaturationBegin->getValue())/(options.tSaturationFull->getValue()-options.tSaturationFull->getValue()));
					potential = std::min(potential, options.tSaturationLimit->getValue());
				} else {
					potential = options.tSaturationLimit->getValue();
				}
				saturationSnap = (potential/accelerationNorm)*trajInput.acceleration;
			}
		}

		snap =
			- options.tSmoothingGains->getValue()(0) * (filterState.jerk)
			- options.tSmoothingGains->getValue()(1) * (filterState.acceleration)
			- options.tSmoothingGains->getValue()(2) * (filterState.velocity-trajInput.velocity)
			- saturationSnap;

		yawAcceleration =
				- options.tSmoothingGainsYaw->getValue() * (filterState.yawRate-trajInput.yawRate);


		filter.setInput(snap, yawAcceleration);

		typedef boost::numeric::odeint::runge_kutta4< QuadrupleIntegratorState, double, QuadrupleIntegratorState, double, boost::numeric::odeint::vector_space_algebra> stepper;
		boost::numeric::odeint::integrate_const( stepper() , filter, filterState , 0.0, timeStep, timeStep);
	}

	trajInput.setVelocity(filterState.velocity,
			filterState.acceleration,
			filterState.jerk,
			snap);
	trajInput.setYawAngle(filterState.yawAngle, filterState.yawRate, yawAcceleration);

	if (options.tPublishSmoothedTrajectory->getValue()){
		telekyb_msgs::TKTrajectory smoothTrajMsg;
		trajInput.toTKTrajMsg(smoothTrajMsg);
		smoothTrajMsg.header.stamp = ros::Time::now();
		smoothTrajPub.publish(smoothTrajMsg);
	}

	return true;

}

} /* namespace telekyb_traj */
