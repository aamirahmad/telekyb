/*
 * KalmanStateEstimator.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */

#ifndef SENSORHANDLER_HPP_
#define SENSORHANDLER_HPP_


#include <telekyb_defines/telekyb_defines.hpp>
#include <StateEstimators/KalmanDataTypes.hpp>
#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class SensorHandler{

public:
	//Core functions
	virtual void update(StateBufferElement& state) = 0;
	virtual void initialize() = 0;
	virtual void destroy() = 0;
	virtual ~SensorHandler(){};

protected:
	ros::NodeHandle nodeHandle;
	ros::Subscriber sensorSub;
};


} /* namespace telekyb_state */
#endif /* SENSORHANDLER_HPP_ */
