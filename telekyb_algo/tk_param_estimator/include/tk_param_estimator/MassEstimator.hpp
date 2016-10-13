/*
 * MassEstimator.hpp
 *
 *  Created on: Oct 28, 2011
 *      Author: mriedel
 */

#ifndef MASSESTIMATOR_HPP_
#define MASSESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_msgs/TKState.h>

#include <tk_param_estimator/MassEstimDefines.hpp>

#include <string>

namespace tk_param_estimator
{

// Interface definition for StateEstimators

class MassEstimator
{
protected:
//	MassEstimator();


public:
	virtual void initialize() = 0;

	virtual void destroy() = 0;

	virtual std::string getName() const = 0;

	// actual method
	virtual void run(const MassEstimInput& input,MassEstimOutput& output) = 0;

	// ask for current Mass // also for initial;
	virtual double getInitialMass() const = 0;

	// Destructor
	virtual ~MassEstimator() {};
};


} // namepsace


#endif /* MASSESTIMATOR_HPP_ */
