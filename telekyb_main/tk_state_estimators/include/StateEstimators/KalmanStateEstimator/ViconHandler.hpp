/*
 * ViconHandler.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */

#ifndef VICONHANDLER_HPP_
#define VICONHANDLER_HPP_


#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <StateEstimators/KalmanStateEstimator/SensorHandler.hpp>

#include <geometry_msgs/TransformStamped.h>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class ViconHandlerOption : public OptionContainer {
public:
	//Option<std::string>* viconTopic;

	Option<Eigen::Matrix<double,6,6>>* vicCov;

	Option<Eigen::Vector3d>* vicPosition;
	Option<Eigen::Quaterniond>* vicOrientation;

	Option<std::string>* vicTopic;
	ViconHandlerOption();
};

class ViconHandler: public SensorHandler{

protected:
	ViconHandlerOption options;

	void measureCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
	Eigen::Vector3d vicPosition;
	Eigen::Quaterniond vicOrientation;

public:
//	//Core functions
	void update(StateBufferElement& state, const geometry_msgs::TransformStamped& z);
	void initialize();
	void destroy();

	MeasureBufferElement lastMeasure;

};


} /* namespace telekyb_state */
#endif /* VICONHANDLER_HPP_ */
