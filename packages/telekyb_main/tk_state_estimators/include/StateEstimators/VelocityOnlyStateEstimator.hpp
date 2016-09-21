/*
 * VelocityOnlyStateEstimator.hpp
 *
 *  Created on: Aug 8, 2013
 *      Author: pstegagno
 */

#ifndef VELOCITYONLYSTATEESTIMATOR_HPP_
#define VELOCITYONLYSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
// ros
#include <ros/subscriber.h>

#include <telekyb_base/Spaces/R3.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>
#include <tk_draft_msgs/TKSmallImu.h>
#include <std_msgs/Bool.h>


using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class VelocityOnlyStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tViconTopicName;
	Option<std::string>* tImuTopicName;
	Option<std::string>* tDvoTopicName;

	
	Option<Eigen::Matrix3d>* tVelocityOnlyToNEDMatrix;

	// Velocity Filters
	Option<double>* tVelocityOnlyVelFilterFreq;
	Option<double>* tVelocityOnlySmoothVelFilterFreq;
	Option<double>* tVelocityOnlyAngFilterFreq;

	Option<double>* tVelocityOnlySampleTime;

	Option<bool>* tVelocityOnlyPublishSmoothVel;

	VelocityOnlyStateEstimatorOptions();
};

class VelocityOnlyStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
	VelocityOnlyStateEstimatorOptions options;
	
	ros::NodeHandle nodeHandle;
	
	
	ros::Subscriber imuSub;
	ros::Subscriber dvoSub;
	ros::Publisher kalmanVelPub;
	tk_draft_msgs::TKSmallImu msg_state;
	
	
	
	Eigen::Vector3d pstate;
	Eigen::Vector2d astate;
	
	Eigen::Matrix<double,5,5> stateCov;
	
	Time lastImuTime;
	Eigen::Vector3d lastAngVel;
	Eigen::Vector3d lastLinAcc;
	
	Eigen::Vector3d driftAngVel;
	Eigen::Vector3d driftLinAcc;
	
	bool firstExecution;
	
	double imucounter;
	
	
	
	
	ros::Time lastdvotime;
	bool firstDvo;
	ros::Subscriber viconSub;
	
	
	ros::Publisher smoothVelPub;

	void initVelocityFilters();

	IIRFilter* velFilter[3];
	IIRFilter* smoothVelFilter[3];
	IIRFilter* angFilter[4];

	void velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega);

public:
	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

	void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void imuCallback(const tk_draft_msgs::TKSmallImu::ConstPtr& msg);
	void dvoCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};

} /* namespace telekyb_state */
#endif /* VELOCITYONLYSTATEESTIMATOR_HPP_ */


