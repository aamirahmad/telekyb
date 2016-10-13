/*
 * KalmanStateEstimator.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */

#ifndef MEASUREHANDLER_HPP_
#define MEASUREHANDLER_HPP_


#include <telekyb_defines/telekyb_defines.hpp>
#include <StateEstimators/KalmanDataTypes.hpp>
#include <telekyb_base/Options.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class MeasureHandlerOption : public OptionContainer {
public:
	//Option<std::string>* viconTopic;

	Option<Eigen::Matrix<double,6,6> >* vicCov;

	Option<Eigen::Vector3d>* vicPosition;
	Option<Eigen::Quaterniond>* vicOrientation;

	MeasureHandlerOption();
};

class MeasureHandler{
protected:
	MeasureHandlerOption options;

	//ros::Subscriber vicSub;
public:
	//Core functions
	void update(StateBufferElement& state, const MeasureBufferElement& z);
};


} /* namespace telekyb_state */
#endif /* MEASUREHANDLER_HPP_ */
