/*
 * StandardMassEstimator.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef STANDARDMASSESTIMATION_HPP_
#define STANDARDMASSESTIMATION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Filter/IIRFilter.hpp>

// Import Interface
#include <tk_param_estimator/MassEstimator.hpp>

using namespace TELEKYB_NAMESPACE;
using namespace tk_param_estimator;

namespace parameter_estimators_plugin {

class StandardMassEstimOptions : public OptionContainer {
public:
	Option<double>* tInitialMass;

	Option<double>* tAFiltCoeff;/*a*/
	Option<double>* tLambdaZeroGain;
	Option<double>* tKappaZeroGain;
	Option<double>* tSampleTime;

	Option<double>* tMaxMass;
	Option<double>* tMinMass;

	Option<double>* tGravity;

	Option<bool>* tPublishMass;
	Option<std::string>* tMassTopic;

	StandardMassEstimOptions();
};

class StandardMassEstimator : public MassEstimator {
private:
	StandardMassEstimOptions options;

	ros::Publisher massPub;

	double estInvMass;
	double estGain;

	double integInitialInvMass;
	double integInitialGain;

	// Filters
	IIRFilter* thrustFilter;
	IIRFilter* vertVelFilter;
	IIRFilter* gravityFilter;

	// Integrators
	IIRFilter* estInvMassIntegrator;
	IIRFilter* estGainIntegrator;

	ros::NodeHandle nodeHandle;

public:
	StandardMassEstimator();
	virtual ~StandardMassEstimator();


	void initialize();
	void destroy();
	std::string getName() const;

	void run(const MassEstimInput& input,MassEstimOutput& output);

	double getInitialMass() const;
};

}

#endif /* STANDARDMASSESTIMATION_HPP_ */
