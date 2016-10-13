/*
 * FlowImuStateEstimator.hpp
 *
 *  Created on: Jan 03, 2016
 *      Author: modelga
 */

#ifndef FLOWIMUSTATEESTIMATOR_HPP_
#define FLOWIMUSTATEESTIMATOR_HPP_

#define ACCGAIN 0.0015
#define ALPHA 0.0
#define YAW45 0.7071

#include <telekyb_base/ROS.hpp>

#include "ros/ros.h"
#include <math.h>

#include <tk_draft_msgs/TKSmallImu.h>
#include <telekyb_msgs/TKState.h>
#include <telekyb_msgs/Behavior.h>
#include <px_comm/OpticalFlow.h>

#include <eigen_conversions/eigen_msg.h>

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

//debug:
#include <fstream>

using namespace TELEKYB_NAMESPACE;

namespace state_estimators_plugin {

class FlowImuEstimatorOptions : public OptionContainer {
public:
	Option<Eigen::Vector3d>* tImuScale;
	Option<Eigen::Vector3d>* tImuOffset;
	Option<Eigen::Vector3d>* tGyroScale;
	Option<Eigen::Vector3d>* tInputNoiseCov;
	Option<Eigen::Vector3d>* tMeasurementNoiseCov;
	
	Option<std::string>* tSmurfInterfaceTopicName;
	Option<std::string>* tFlowTopicName;
	Option<std::string>* tBehaviorTopicName;
	
	Option<bool>* tSaveFiles;
	Option<bool>* tUniqueNames;
	Option<std::string>* tFilesLocation;
	
	FlowImuEstimatorOptions();
};

class FlowImuStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
protected:
	FlowImuEstimatorOptions options;

	ros::NodeHandle nh_;
	ros::Subscriber imuSub_;
	ros::Subscriber flowSub_;
	ros::Subscriber behaviorSub_;

	ros::Publisher tkStatePub_;
	
	//TKstate
	telekyb_msgs::TKState stateMsg;
	
	//filter variables
	Eigen::VectorXd q_; //state vector [dx, dy, dz, z, h]
	Eigen::Vector3d u_; //input vestor [accX, accY, accZ]
	Eigen::Vector2d z1_; //measurement vector [dx, dy]
	double z2_; //measuremet [z]
	
	Eigen::MatrixXd A_; //5x5
	Eigen::MatrixXd B_; //5x3
	Eigen::MatrixXd C1_; //2x5
	Eigen::VectorXd c2_; //5x1 - needs transposition to 1x5
	
	Eigen::Matrix3d Qu_; //input covariance
	Eigen::MatrixXd Qq_; //proces covariance 5x5
	Eigen::Matrix2d R1_; //measurement covariance
	double r2_;
	
	Eigen::MatrixXd P_; //5x5
      
	//update::
	Eigen::Vector2d y1_;
	Eigen::Matrix2d S1_;
	Eigen::MatrixXd K1_;
	
	double y2_;
	double s2_;
	Eigen::VectorXd k2_; //5x1
	
	//imu:
	bool firstImu;
	double timePrevImu;
	Eigen::Vector3d gw;
	uint8_t imuCounter;
	
	double estRoll, estPitch;
	
	Eigen::Matrix3d Rqh;
	//from manual IMU calib in matlab:
	Eigen::Vector3d imuScale;
	Eigen::Vector3d imuOffset;
	Eigen::Vector3d gyroScale;
	
	Eigen::Vector3d imuAcc;
	Eigen::Vector3d imuAngVel;
	
	//flow:
	bool firstFlow;    
	double timePrevFlow;
	uint8_t flowCounter;
	bool doUpdate;
	bool onGround;
	//double yaw45;
	
	//debug:
	std::ofstream fileImu; //time, x, y, z, velX, velY, velZ, accX, accY, accZ, roll, pitch, yaw, angVelX, angVelY, angVelZ
	std::ofstream fileFlow; //time, quality, distance, velX, velY, velZ
	bool saveToFile;

public:
	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;
	
	void initVariables();
	void imuCallback(const tk_draft_msgs::TKSmallImuConstPtr& msg);
	void flowCallback(const px_comm::OpticalFlowConstPtr& msg);
	void behaviorCallback(const telekyb_msgs::BehaviorConstPtr& msg);
	void publishTKState(ros::Time stamp);
};

} /* namespace telekyb_state */
#endif /* FLOWIMUSTATEESTIMATOR_HPP_ */
