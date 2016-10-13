#ifndef COMPLEMENTARYDATATYPES_HPP_
#define COMPLEMENTARYDATATYPES_HPP_

#include <Eigen/Geometry>

class State {
public:
	Eigen::Vector3d position;
	Eigen::Vector3d lin_velocity;
	Eigen::Quaterniond orientation;
	Eigen::Vector3d bias;

	State operator+(const State& addend) const {
		State temp;
		temp.position = position + addend.position;
		temp.lin_velocity = lin_velocity + addend.lin_velocity;
		temp.orientation.w() = orientation.w() + addend.orientation.w();
		temp.orientation.vec() = orientation.vec() + addend.orientation.vec();
		temp.bias = bias + addend.bias;
		return (temp);
	}

	State& operator+=(const State& addend) {
		position += addend.position;
		lin_velocity += addend.lin_velocity;
		orientation.w() += addend.orientation.w();
		orientation.vec() += addend.orientation.vec();
		bias += addend.bias;
		return *this;
	}

	template <typename _scalarT>
	State operator*(const _scalarT factor) const {
		State temp;
		temp.position = factor*position;
		temp.lin_velocity = factor*lin_velocity;
		temp.orientation.w() = factor*orientation.w();
		temp.orientation.vec() = factor*orientation.vec();
		temp.bias = factor*bias;
		return (temp);
	}

	template <typename _scalarT>
	State& operator*=(const _scalarT factor) {
		State temp;
		position *= factor;
		lin_velocity *= factor;
		orientation.w() *= factor;
		orientation.vec() *= factor;
		bias *= factor;
		return *this;
	}

};

template <typename _scalarT>   //this is for left multiplication by a scalar (uses right multiplier)
State operator*(const _scalarT factor, const State& state) { return state*factor; };


class Measure {
public:
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;
};

class Input {
public:
	Eigen::Vector3d lin_acc;
	Eigen::Vector3d ang_vel;
};

class DynamicSystem {
protected:
	Eigen::Vector3d positionGain;
	Eigen::Vector3d velocityGain;
	Eigen::Vector3d orientationGain;
	Eigen::Vector3d biasGain;

	Eigen::Vector3d gravity;

	Eigen::Matrix3d matrixA;
	Eigen::Vector3d vectorB;
	double normalizationGain;

	Input input;
	Measure measure;
public:

	DynamicSystem(){}

	DynamicSystem(const Eigen::Vector3d& inPositionGain, const Eigen::Vector3d& inVelocityGain, const Eigen::Vector3d& inOrientationGain, const Eigen::Vector3d& inBiasGain,
			const Eigen::Vector3d& inGravity, double inNormalizationGain, const Eigen::Matrix3d& inMatrixA, const Eigen::Vector3d& inVectorB):
		positionGain(inPositionGain),
		velocityGain(inVelocityGain),
		orientationGain(inOrientationGain),
		biasGain(inBiasGain),
		gravity(inGravity),
		matrixA(inMatrixA),
		vectorB(inVectorB),
		normalizationGain(inNormalizationGain) {
	}

	void setInput(const Input& inInput){
		input = inInput;
	}

	const Input& getInput(){
		return input;
	}

	void setMeasure(const Measure& inMeasure){
			measure = inMeasure;
	}

	const Measure& getMeasure(){
		return measure;
	}

	/* Differential equation of the estimator */
    void operator()(State& inState, State& dState , double t )
    {
    	Eigen::Vector3d positionResidual = measure.position - inState.position;
    	Eigen::Quaterniond orientationResidual = inState.orientation.conjugate()*measure.orientation;

    	Eigen::Matrix3d rot = inState.orientation.normalized().toRotationMatrix();
    	dState.position =  inState.lin_velocity + positionGain.asDiagonal()*positionResidual;
    	dState.lin_velocity = rot*(matrixA*input.lin_acc + vectorB + inState.bias) + gravity
    			+ velocityGain.asDiagonal()*positionResidual;

    	Eigen::Quaterniond omegaQuaternion;
    	omegaQuaternion.w() = normalizationGain*(1-inState.orientation.squaredNorm());
    	omegaQuaternion.vec() = .5*input.ang_vel + orientationGain.asDiagonal()*(orientationResidual.w()*orientationResidual.vec());

    	dState.orientation = inState.orientation*omegaQuaternion;

    	dState.bias = biasGain.asDiagonal()*rot.transpose()*positionResidual;

    }
};

#endif /* COMPLEMENTARYDATATYPES_HPP_ */
