/*
 * ViconImuStateEstimator.hpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel & rspica
 */

#ifndef VICONIMUSTATEESTIMATOR_HPP_
#define VICONIMUSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
// ros
#include <ros/subscriber.h>

#include <telekyb_base/Spaces/R3.hpp>
#include <telekyb_base/Filter/IIRFilter.hpp>
#include <telekyb_base/Time.hpp>

#include <sensor_msgs/Imu.h>

#include <boost/thread.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class ViconImuStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tViconSeTopicName;
	Option<std::string>* tImuSeTopicName;
	Option<Eigen::Matrix3d>* tViconToNEDMatrix;
	Option<Eigen::Quaterniond>* tViconToNEDQuaternion;

	Option<double>* timeStep;

	// Velocity Filters
	Option<double>* tViconVelFilterFreq;
	Option<double>* tViconSmoothVelFilterFreq;
	Option<double>* tViconAngFilterFreq;
	Option<double>* tViconSampleTime;
	Option<bool>* tViconPublishSmoothVel;

	Option<bool>* tIntegrateLinVel;
	Option<bool>* tIntegrateAngVel;

	ViconImuStateEstimatorOptions();

	Option<std::string>* tSsxSeTopicName;

};

class ViconImuStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
	ViconImuStateEstimatorOptions options;

	ros::NodeHandle nodeHandle;
	ros::Subscriber viconSub;
	ros::Subscriber imuSub;
	ros::Publisher smoothVelPub;

//	ros::Subscriber ssxSub;
//	ros::Publisher debugPub;

	void initVelocityFilters();
	void publishState();

	IIRFilter* velFilter[3];
	IIRFilter* smoothVelFilter[3];

	Velocity3D angVelocityNED;
	Quaternion quatNED;
	Position3D posNED;
	Velocity3D velNED;
	Velocity3D smoothVelNED;
	telekyb::Time intTime;
	telekyb::Timer intTimer; 

	// Thread
	boost::thread thread;
	// Mutexes
	boost::mutex imuMutex;
	boost::mutex viconMutex;
	boost::mutex timeMutex;
	telekyb::Timer rtTimer;

	void spin();
	bool spinning;
	void viconCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void ssxCallback(const telekyb_msgs::TKState::ConstPtr& stateMsg);

public:
	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

};

} /* namespace telekyb_state */
#endif /* VICONIMUSTATEESTIMATOR_HPP_ */
