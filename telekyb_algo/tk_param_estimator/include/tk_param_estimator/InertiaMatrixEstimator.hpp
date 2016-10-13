/*
 * InertiaMatrixEstimator.hpp
 *
 *  Created on: Jul 28, 2012
 *      Author: rspica
 */

#ifndef INERTIAMATRIXESTIMATOR_HPP_
#define INERTIAMATRIXESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_msgs/TKState.h>

#include <tk_param_estimator/InertiaMatrixEstimDefines.hpp>

#include <string>

namespace tk_param_estimator
{

// Interface definition for StateEstimators

class InertiaMatrixEstimator
{
protected:
//	InertiaMatrixEstimator();


public:
	virtual void initialize() = 0;

	virtual void destroy() = 0;

	virtual std::string getName() const = 0;

	// actual method
	virtual void run(const InertiaMatrixEstimInput& input,InertiaMatrixEstimOutput& output) = 0;

	// ask for current InertiaMatrix // also for initial;
	virtual Eigen::Matrix3d getInitialInertiaMatrix() const = 0;

	// Destructor
	virtual ~InertiaMatrixEstimator() {};
};


} // namepsace


#endif /* INERTIAMATRIXESTIMATOR_HPP_ */
