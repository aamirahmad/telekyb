/*
 * NoMassEstimator.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef NOMASSESTIMATION_HPP_
#define NOMASSESTIMATION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Filter/IIRFilter.hpp>

// Import Interface
#include <tk_param_estimator/MassEstimator.hpp>

using namespace TELEKYB_NAMESPACE;
using namespace tk_param_estimator;

namespace parameter_estimators_plugin {

class NoMassEstimOptions : public OptionContainer {
public:
	Option<double>* tInitialMass;

	Option<bool>* tPublishMass;
	Option<std::string>* tMassTopic;

	NoMassEstimOptions();
};

class NoMassEstimator : public MassEstimator {
private:
	NoMassEstimOptions options;

	ros::Publisher massPub;


	ros::NodeHandle nodeHandle;

public:
	NoMassEstimator();
	virtual ~NoMassEstimator();


	void initialize();
	void destroy();
	std::string getName() const;

	void run(const MassEstimInput& input,MassEstimOutput& output);

	double getInitialMass() const;
};

}

#endif /* STANDARDMASSESTIMATION_HPP_ */
