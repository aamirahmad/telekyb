/*
 * QuadrupleIntegrator.hpp
 *
 *  Created on: Nov 26, 2012
 *      Author: rspica
 */

#ifndef QUADRUPLEINTEGRATOR_HPP_
#define QUADRUPLEINTEGRATOR_HPP_


#include <telekyb_base/Messages/TKTrajectory.hpp>
#include <telekyb_base/Messages/TKState.hpp>

namespace TELEKYB_NAMESPACE {

class QuadrupleIntegratorState{
public:
	Eigen::Vector3d position;
	Eigen::Vector3d	velocity;
	Eigen::Vector3d	acceleration;
	Eigen::Vector3d	jerk;
	double yawAngle;
	double yawRate;

	QuadrupleIntegratorState(){};
	QuadrupleIntegratorState(const TKTrajectory& inTraj);
	QuadrupleIntegratorState(const TKState& inState);

	QuadrupleIntegratorState& operator+=(const QuadrupleIntegratorState& addend) {
		position += addend.position;
		velocity += addend.velocity;
		acceleration += addend.acceleration;
		jerk += addend.jerk;
		yawAngle += addend.yawAngle;
		yawRate += addend.yawRate;
		return *this;
	}

	QuadrupleIntegratorState operator+(const QuadrupleIntegratorState& addend) const {
		QuadrupleIntegratorState temp;
		temp.position = position + addend.position;
		temp.velocity = velocity + addend.velocity;
		temp.acceleration = acceleration + addend.acceleration;
		temp.jerk = jerk + addend.jerk;
		temp.yawAngle = yawAngle + addend.yawAngle;
		temp.yawRate = yawRate + addend.yawRate;
		return (temp);
	}

	template <typename _scalarT>
	QuadrupleIntegratorState& operator*=(const _scalarT factor) {
		position *= factor;
		velocity *= factor;
		acceleration *= factor;
		jerk *= factor;
		yawAngle *= factor;
		yawRate *= factor;
		return *this;
	}

	template <typename _scalarT>
	QuadrupleIntegratorState operator*(const _scalarT factor) const {
		QuadrupleIntegratorState temp;
		temp.position = position*factor;
		temp.velocity = velocity*factor;
		temp.acceleration = acceleration*factor;
		temp.jerk = jerk*factor;
		temp.yawAngle = yawAngle*factor;
		temp.yawRate = yawRate*factor;
		return (temp);
	}
};

template <typename _scalarT>   //this is for left multiplication by a scalar (uses right multiplier)
QuadrupleIntegratorState operator*(const _scalarT factor, const QuadrupleIntegratorState& state) { return state*factor; };





class QuadrupleIntegrator {
protected:
	Eigen::Vector3d _inputSnap;
	double _inputYawAcc;

public:

	QuadrupleIntegrator(){};

	void setInput(const Eigen::Vector3d& inSnap, const double inYawAcc);
    void operator()(QuadrupleIntegratorState& state, QuadrupleIntegratorState& dState , double t );
};

}

#endif /*QUADRUPLEINTEGRATOR_HPP_*/
