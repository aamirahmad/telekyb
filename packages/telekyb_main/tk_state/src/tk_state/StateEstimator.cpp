/*
 * StateEstimator.cpp
 *
 *  Created on: Nov 11, 2011
 *      Author: mriedel
 */


#include <tk_state/StateEstimator.hpp>

#include <tk_state/StateEstimatorController.hpp>

namespace TELEKYB_NAMESPACE
{

StateEstimator::StateEstimator()
	: stateEstimatorController( StateEstimatorController::Instance() )
{

}

StateEstimator::~StateEstimator()
{

}

bool StateEstimator::isInitialized() const
{
	return initialized;
}

// bool StateEstimator::isActive() const
// {
// 	return (stateEstimatorController.getActiveStateEstimator() == this);
// }


}

