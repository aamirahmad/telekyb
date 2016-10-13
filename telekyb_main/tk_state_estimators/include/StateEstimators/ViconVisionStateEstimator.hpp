/*
 * ViconVisionStateEstimator.h
 *
 *  Created on: Oct 8, 2013
 *      Author: vgrabe
 */

#ifndef VICONVISIONSTATEESTIMATOR_H_
#define VICONVISIONSTATEESTIMATOR_H_
#include <ros/subscriber.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include <pluginlib/class_list_macros.h>

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Spaces/R3.hpp>
#include <telekyb_base/Filter/IIRFilter.hpp>
#include <tk_state/StateEstimator.hpp>
#include <telekyb_msgs/TKState.h>


namespace telekyb_state {

class ViconVisionStateEstimatorOptions : public TELEKYB_NAMESPACE::OptionContainer
{
public:
	TELEKYB_NAMESPACE::Option<std::string> *tViconSeTopicName, *tVisionSeTopicName, *tJoyTopicName;
	TELEKYB_NAMESPACE::Option<Eigen::Matrix3d>* tViconToNEDMatrix;

	// Velocity Filters
	TELEKYB_NAMESPACE::Option<double>* tViconVelFilterFreq;
	TELEKYB_NAMESPACE::Option<double>* tViconSmoothVelFilterFreq;
	TELEKYB_NAMESPACE::Option<double>* tViconAngFilterFreq;

	TELEKYB_NAMESPACE::Option<double>* tViconSampleTime;

	TELEKYB_NAMESPACE::Option<bool>* tViconPublishSmoothVel;

	ViconVisionStateEstimatorOptions();
};

class ViconVisionStateEstimator: public telekyb::StateEstimator
{
public:
	ViconVisionStateEstimator();
	virtual ~ViconVisionStateEstimator();

	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual inline std::string getName() const;

	void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void visionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
	void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg);

protected:
	ViconVisionStateEstimatorOptions options;

	ros::NodeHandle nodeHandle;
	ros::Subscriber viconSub, visionSub, joySub;

	ros::Publisher smoothVelPub, debugPublisher;

	void initVelocityFilters();

	TELEKYB_NAMESPACE::IIRFilter* velFilter[3];
	TELEKYB_NAMESPACE::IIRFilter* smoothVelFilter[3];
	TELEKYB_NAMESPACE::IIRFilter* angFilter[4];

	bool useVision;
	int numberOfDerivations;

	void velQuatToBodyOmega(const Eigen::Quaterniond& quat, const Eigen::Quaterniond& quatRates, Eigen::Vector3d& bodyOmega);
	void submitState(const TELEKYB_NAMESPACE::TKState& state, bool isVision);

};

} /* namespace TELEKYB_NAMESPACE */
#endif /* VICONVISIONSTATEESTIMATOR_H_ */
