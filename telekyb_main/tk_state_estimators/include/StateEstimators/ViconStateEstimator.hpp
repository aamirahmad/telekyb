/*
 * ViconStateEstimator.hpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */

#ifndef VICONSTATEESTIMATOR_HPP_
#define VICONSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
// ros
#include <ros/subscriber.h>

#include <telekyb_base/Spaces/R3.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>

using namespace TELEKYB_NAMESPACE;

namespace state_estimators_plugin {

class ViconStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tViconSeTopicName;
	Option<Eigen::Matrix3d>* tViconToNEDMatrix;

	// Velocity Filters
	Option<bool>* tViconRoboCentric;
	Option<double>* tViconVelFilterFreq;
	Option<double>* tViconSmoothVelFilterFreq;
	Option<double>* tViconAngFilterFreq;

	Option<double>* tViconSampleTime;

	Option<bool>* tViconPublishSmoothVel;

	ViconStateEstimatorOptions();
};

class ViconStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
	ViconStateEstimatorOptions options;

	ros::NodeHandle nodeHandle;
	ros::Subscriber viconSub;

	ros::Publisher smoothVelPub;

	void initVelocityFilters();

	IIRFilter* velFilter[3];
	IIRFilter* smoothVelFilter[3];
	IIRFilter* angFilter[4];

	void velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega);
	
	// for robocentric rotation


public:
	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

	void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
//	ViconStateEstimator();
//	virtual ~ViconStateEstimator();
};

} /* namespace telekyb_state */
#endif /* VICONSTATEESTIMATOR_HPP_ */
