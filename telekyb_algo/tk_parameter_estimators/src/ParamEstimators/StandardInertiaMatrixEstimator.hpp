/*
 * StandardInertiaMatrixEstimator.hpp
 *
 *  Created on: Jul 28, 2012
 *      Author: rspica
 */

#ifndef STANDARDINERTIAMATRIXESTIMATION_HPP_
#define STANDARDINERTIAMATRIXESTIMATION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Filter/IIRFilter.hpp>

// Import Interface
#include <tk_param_estimator/InertiaMatrixEstimator.hpp>

using namespace TELEKYB_NAMESPACE;
using namespace tk_param_estimator;

namespace parameter_estimators_plugin {

class StandardInertiaMatrixEstimOptions : public OptionContainer {
public:

	Option<Eigen::Vector3d>* tInitialInertiaMatrix;

	Option<Eigen::Vector3d>* tAFiltCoeff;/*a*/
	Option<Eigen::Vector3d>* tKappaZeroGain;
	Option<Eigen::Vector3d>* tLambdaZeroGain;
	Option<double>* tSampleTime;

	Option<Eigen::Vector3d>* tMaxInertia;
	Option<Eigen::Vector3d>* tMinInertia;

	Option<bool>* tPublishInertia;
	Option<std::string>* tInertiaTopic;

	StandardInertiaMatrixEstimOptions();
};

class StandardInertiaMatrixEstimator : public InertiaMatrixEstimator {
private:
	StandardInertiaMatrixEstimOptions options;

	Eigen::Vector3d estInvInertia;
	Eigen::Vector3d estGain;

	Eigen::Vector3d integInitialInvInertia;
	Eigen::Vector3d integInitialGain;

	// Filters
	IIRFilter* torqueFilter[3];
	IIRFilter* angVelFilter[3];

	// Integrators
	IIRFilter* estInvInertiaIntegrator[3];
	IIRFilter* estGainIntegrator[3];

	ros::NodeHandle nodeHandle;
	ros::Publisher inertiaPub;

public:
	StandardInertiaMatrixEstimator();
	virtual ~StandardInertiaMatrixEstimator();


	void initialize();
	void destroy();
	std::string getName() const;

	void run(const InertiaMatrixEstimInput& input,InertiaMatrixEstimOutput& output);

	Eigen::Matrix3d getInitialInertiaMatrix() const;
};

}

#endif /* STANDARDINERTIAMATRIXESTIMATION_HPP_ */
