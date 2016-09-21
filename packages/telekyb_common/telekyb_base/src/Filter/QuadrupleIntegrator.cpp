/*
 * QuadrupleIntegrator.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: rspica
 */



#include <telekyb_base/Filter/QuadrupleIntegrator.hpp>



namespace TELEKYB_NAMESPACE {

QuadrupleIntegratorState::QuadrupleIntegratorState(const TKTrajectory& inTraj):
	position(inTraj.position),
	velocity(inTraj.velocity),
	acceleration(inTraj.acceleration),
	jerk(inTraj.jerk),
	yawAngle(inTraj.yawAngle),
	yawRate(inTraj.yawRate){
}

QuadrupleIntegratorState::QuadrupleIntegratorState(const TKState& inState):
	position(inState.position),
	velocity(inState.linVelocity),
	acceleration(Eigen::Vector3d::Zero()),
	jerk(Eigen::Vector3d::Zero()),
	yawAngle(inState.getEulerRPY()(2)),
	yawRate(inState.angVelocity(2)){
}

void QuadrupleIntegrator::setInput(const Eigen::Vector3d& inSnap, const double inYawAcc){
	_inputSnap = inSnap;
	_inputYawAcc = inYawAcc;
}

void QuadrupleIntegrator::operator()(QuadrupleIntegratorState& state, QuadrupleIntegratorState& dState , double t ){
	// Quadruple integrator
	dState.position = state.velocity;
	dState.velocity = state.acceleration;
	dState.acceleration = state.jerk;
	dState.jerk = _inputSnap;
	dState.yawAngle = state.yawRate;
	dState.yawRate = _inputYawAcc;
}


}
