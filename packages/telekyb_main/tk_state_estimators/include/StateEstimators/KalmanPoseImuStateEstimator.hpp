/*
 * KalmanPoseImuStateEstimator.hpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */

#ifndef KALMANPOSEIMUSTATEESTIMATOR_HPP_
#define KALMANPOSEIMUSTATEESTIMATOR_HPP_

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

#include <sensor_msgs/Imu.h>
//pose sensors:
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//eigen and roll, pitch, yaw from msg:
#include "tf_conversions/tf_eigen.h"

#define SIZE 101

#include <fstream>





using namespace TELEKYB_NAMESPACE;

namespace state_estimators_plugin {

class KalmanPoseImuStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tKalmanPoseImuSeTopicName;
	Option<Eigen::Matrix3d>* tKalmanPoseImuToNEDMatrix;

	// Velocity Filters
	Option<double>* tKalmanPoseImuVelFilterFreq;
	Option<double>* tKalmanPoseImuSmoothVelFilterFreq;
	Option<double>* tKalmanPoseImuAngFilterFreq;

	Option<double>* tKalmanPoseImuSampleTime;

	Option<bool>* tKalmanPoseImuPublishSmoothVel;

	KalmanPoseImuStateEstimatorOptions();
};

class KalmanPoseImuStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
  
  bool busy;
  
  
 	std::ofstream file1, file2, file3;

	ros::Publisher* posePub;
	geometry_msgs::PoseWithCovarianceStamped filteredPose;

	bool firstImu;
	bool firstPose;

	Eigen::MatrixXd kalmanGain;

	Eigen::MatrixXd matCImu;
	Eigen::MatrixXd matCPose;

	//buffers:
	//tested - all elements of buffers initialized with given value:
	//state vector: x, y, z, x', y', z', roll, pitch, yaw
	Eigen::VectorXd stateQ[SIZE];
	Eigen::MatrixXd covP[SIZE];

	double imuTime[SIZE];
	Eigen::VectorXd inputU[SIZE];
	Eigen::MatrixXd matA[SIZE];
	Eigen::MatrixXd matB[SIZE];
	Eigen::VectorXd imuMeasZ[SIZE];
	Eigen::VectorXd imuInnovY[SIZE];
	Eigen::MatrixXd imuInnovCovS[SIZE];

	//constants?:
	Eigen::MatrixXd inputCovQ;
	Eigen::MatrixXd measCovRImu;
	Eigen::MatrixXd measCovRPose;

	//sensors positions and orientations:
	Eigen::Vector3d sensor1Position;
	Eigen::Vector3d sensor1Orientation;
	Eigen::Vector3d sensor2Position;
	Eigen::Vector3d sensor2Orientation;

	//inversed matrices
	tf::Matrix3x3 RPose1Inv, RPose2Inv;

	//imu offset:
	Eigen::Vector3d linAccOffset;
	Eigen::Vector3d angVelOffset;
  
	ros::Subscriber imuSub;
	//ros::Subscriber imuGTSub = n.subscribe<sensor_msgs::Imu>("/firefly/ground_truth/imu", 1000, imuCallback);
	
	ros::Subscriber pose1Sub;
	ros::Subscriber pose2Sub;
  
  
  
	KalmanPoseImuStateEstimatorOptions options;

	ros::NodeHandle nodeHandle;
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
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
};

} /* namespace telekyb_state */
#endif /* VICONSTATEESTIMATOR_HPP_ */
