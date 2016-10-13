/*
 * StateEstimator.hpp
 *
 *  Created on: Oct 28, 2011
 *      Author: mriedel
 */

#ifndef STATEESTIMATOR_HPP_
#define STATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_msgs/TKState.h>

#include <string>

namespace TELEKYB_NAMESPACE
{

// Forward Declaration
class StateEstimatorController;

// Interface definition for StateEstimators

class StateEstimator
{
protected:
	StateEstimator();
	StateEstimatorController& stateEstimatorController;


	bool initialized;


public:
	// pure within StateEstimator
	bool isInitialized() const;
// 	bool isActive() const;

	virtual void initialize() = 0;
	virtual void willBecomeActive() = 0;
	virtual void willBecomeInActive() = 0;
	virtual void destroy() = 0;

	virtual std::string getName() const = 0;


	// Destructor
	virtual ~StateEstimator();
};


} // namepsace


#endif /* STATEESTIMATOR_HPP_ */
