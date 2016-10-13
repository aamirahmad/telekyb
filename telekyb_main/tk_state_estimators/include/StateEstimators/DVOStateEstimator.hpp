/*
 * DVOStateEstimator.hpp
 *
 *  Created on: Aug 8, 2013
 *      Author: pstegagno
 */

#ifndef DVOSTATEESTIMATOR_HPP_
#define DVOSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
// ros
#include <ros/subscriber.h>

#include <telekyb_base/Spaces/R3.hpp>
#include <telekyb_base/Filter/OneEuroFilter.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>
#include <tk_draft_msgs/TKSmallImu.h>
#include <telekyb_msgs/TKState.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <queue>


using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class DVOStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tViconTopicName;
	Option<std::string>* tImuTopicName;
	Option<std::string>* tDvoTopicName;
	Option<std::string>* tDvoPoseTopicName;

	
	Option<Eigen::Matrix3d>* tDVOToNEDMatrix;

	// Velocity Filters
	Option<double>* tDVOVelFilterFreq;
	Option<double>* tDVOSmoothVelFilterFreq;
	Option<double>* tDVOAngFilterFreq;

	Option<double>* tDVOSampleTime;

	Option<bool>* tDVOPublishSmoothVel;

	DVOStateEstimatorOptions();
};

class DVOStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
	DVOStateEstimatorOptions options;
	
	ros::NodeHandle nodeHandle;
	
	
	std::ofstream dataFile;
	
	
	Eigen::Vector3d pstate;
	Eigen::Vector2d astate;
	
	Eigen::Matrix<double,5,5> stateCov;
	
	Time lastImuTime;
	Eigen::Vector3d lastAngVel;
	Eigen::Vector3d lastLinAcc;
	
	
	ros::Publisher kalmanVelPub;
    telekyb_msgs::TKState msg_state;
	// imu variables
	std::queue<ros::Time> lastimutimequeue;
	std::queue<Eigen::Vector3d> imuAngVelqueue;
	std::queue<double> estRollqueue;
	std::queue<double> estPitchqueue;
	ros::Time lastimutime;
	Eigen::Vector3d imuAngVel;
	double estRoll;
	double estPitch;
	bool firstImu;
	Eigen::Vector3d imuAcc;
	ros::Subscriber imuSub;
	bool _syncronizationNeeded;
	int _wait;
	
	// rot matrix quad/camera
	Eigen::Matrix<double,3,3> Rcq;
	// translation quad/camera
	Eigen::Vector3d pcq;
	Eigen::Matrix<double,3,3> RNEDNWU;
	Eigen::Matrix<double,3,3> Wqq;

	
// 	double rollRateLowPass;
// 	double pitchRateLowPass;
// 	double yawRateLowPass;

	
	
// 	OneEuroFilter *omxFilt;
// 	OneEuroFilter *omyFilt;
// 	OneEuroFilter *omzFilt;
	
	// dvo displacement variables
	ros::Time lastdvotime;
	bool firstDvo;
	ros::Subscriber dvoSub;
	
	// dvo pose variables
	Eigen::Vector3d vcb;
	Eigen::Vector3d vcc;
	Eigen::Vector3d vqq;
	Eigen::Vector3d vqh;
	ros::Time lastdvoPosetime;
	Eigen::Vector3d lastdvoPos;
	Eigen::Matrix3d lastdvoPoseOri;
	bool firstDvoPose;
	ros::Subscriber dvoPoseSub;
	
	OneEuroFilter *xFilt;
	OneEuroFilter *yFilt;
	OneEuroFilter *zFilt;
	
	OneEuroFilter *oxFilt;
	OneEuroFilter *oyFilt;
	OneEuroFilter *ozFilt;
	OneEuroFilter *owFilt;
	
	
	// vicon variables
	ros::Subscriber viconSub;
	ros::Publisher smoothVelPub;
	
	
	
	
	void initVelocityFilters();

	IIRFilter* velFilter[3];
	IIRFilter* smoothVelFilter[3];
	IIRFilter* angFilter[4];
	
	
	OneEuroFilter *smoothVelFilterDVOX;
	OneEuroFilter *smoothVelFilterDVOY;
	OneEuroFilter *smoothVelFilterDVOZ;


	
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
	void dvoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	
};

} /* namespace telekyb_state */
#endif /* VELOCITYONLYSTATEESTIMATOR_HPP_ */


