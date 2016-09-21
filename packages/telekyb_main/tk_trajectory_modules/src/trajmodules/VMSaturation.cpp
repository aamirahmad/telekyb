/*
 * VMSaturation.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#include <trajmodules/VMSaturation.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_traj::VMSaturation, TELEKYB_NAMESPACE::TrajectoryModule);

namespace telekyb_traj {

VMSaturationOptions::VMSaturationOptions()
	: OptionContainer("VMSaturation")
{
	tMaxVel = addOption<double>( "tMaxVel", "Maximum velocity in velocity mode", 100.0, false, true);
	tMaxVelRate = addOption<double>( "tMaxVelRate", "Maximum velocity rate in velocity mode", 10.0, false, true);
}

VMSaturation::VMSaturation()
	: TrajectoryModule("tk_trajprocessor/VMSaturation", TrajModulePosType::Velocity, -100)
{

}

void VMSaturation::initialize()
{

}

void VMSaturation::destroy()
{

}

// set back to intial conditions
void VMSaturation::willTurnActive()
{
	lastVelocity = Velocity3D::Zero();
	lastVelocityTimer.reset();
}

// called after turning inactive
void VMSaturation::didTurnInactive()
{
}

bool VMSaturation::trajectoryStep(const TKState& currentState, TKTrajectory& trajInput)
{
	//ROS_INFO("Called VMSaturation");

	// Velocity Saturation
	std::cout << "velocity norm is:" << trajInput.velocity.norm() << std::endl;
	if(trajInput.velocity.norm() > options.tMaxVel->getValue()) {
		//ROS_WARN("Saturating commanded Velocity to %f", options.tMaxVel->getValue());
		trajInput.velocity.normalize();
		trajInput.velocity *= fabs(options.tMaxVel->getValue());
		trajInput.acceleration = Acceleration3D(0.0,0.0,0.0);
	}

	// Velocity Rate Saturation
	Velocity3D velDiff = trajInput.velocity - lastVelocity;
	double elapsedSec = lastVelocityTimer.getElapsed().toDSec();
	lastVelocityTimer.reset();

	//ROS_INFO("VelRate: %f", velDiff.norm() / elapsedSec);

	if(velDiff.norm() / elapsedSec < options.tMaxVelRate->getValue()) {
		lastVelocity = trajInput.velocity;
	}else{
		//ROS_INFO("Saturating Velocity Rate.");
		lastVelocity += velDiff.normalized() * options.tMaxVelRate->getValue() * elapsedSec;
	}

	trajInput.velocity = lastVelocity;

	return true;
}

} /* namespace telekyb_traj */
