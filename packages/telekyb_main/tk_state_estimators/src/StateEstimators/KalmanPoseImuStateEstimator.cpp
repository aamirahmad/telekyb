/*
 * KalmanPoseImuStateEstimator.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */

#include <StateEstimators/KalmanPoseImuStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <telekyb_msgs/TKState.h>

#define DEBUG_WELL 0
#define DEBUG_NOT_WELL 0

PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::KalmanPoseImuStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace state_estimators_plugin {

KalmanPoseImuStateEstimatorOptions::KalmanPoseImuStateEstimatorOptions()
	: OptionContainer("KalmanPoseImuStateEstimator")
{
	tKalmanPoseImuSeTopicName = addOption<std::string>("tKalmanPoseImuSeTopicName","TopicName of KalmanPoseImu Sensor.","undef",true,true);
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tKalmanPoseImuToNEDMatrix = addOption<Eigen::Matrix3d>("tKalmanPoseImuToNEDMatrix","ConversionMatrix from KalmanPoseImu to NED", m, false, true);

	tKalmanPoseImuVelFilterFreq = addOption<double>("tKalmanPoseImuVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tKalmanPoseImuSmoothVelFilterFreq = addOption<double>("tKalmanPoseImuSmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tKalmanPoseImuAngFilterFreq = addOption<double>("tKalmanPoseImuAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tKalmanPoseImuSampleTime = addOption<double>("tKalmanPoseImuSampleTime",
			"Sampling Time of KalmanPoseImu System. Default is 120Hz", 0.008333333333, false, true);

	tKalmanPoseImuPublishSmoothVel = addOption<bool>("tKalmanPoseImuPublishSmoothVel",
			"Publish TKState with smoothed Velocity", false, false, true);
}

void KalmanPoseImuStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tKalmanPoseImuVelFilterFreq->getValue(),
				1.0,
				options.tKalmanPoseImuSampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tKalmanPoseImuSmoothVelFilterFreq->getValue(),
				1.0,
				options.tKalmanPoseImuSampleTime->getValue());
	}


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tKalmanPoseImuAngFilterFreq->getValue(),
				1.0,
				options.tKalmanPoseImuSampleTime->getValue());
	}
}

// We should outsource this somehow!
void KalmanPoseImuStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
{
	Eigen::Matrix<double, 3, 4> conversionMatrix;
	conversionMatrix << -quat.x() , quat.w(), quat.z(), -quat.y(),
						-quat.y() , -quat.z(), quat.w(), quat.x(),
						-quat.z() , quat.y(), -quat.x(), quat.w();

	Eigen::Vector4d quatRatesVec;
	quatRatesVec(0) = quatRates.w();
	quatRatesVec.tail<3>() = quatRates.vec();

	bodyOmega = (2.0 * conversionMatrix) * quatRatesVec;

}

void KalmanPoseImuStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();
}
void KalmanPoseImuStateEstimator::willBecomeActive()
{
if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::willBecomeActive");
  
  	firstImu = true;
	firstPose = true;
	busy = false;

	kalmanGain = Eigen::MatrixXd::Zero(9,9);

	matCImu = Eigen::MatrixXd::Zero(3,9);
	matCPose = Eigen::MatrixXd::Zero(6,9);

	//buffers:
	//tested - all elements of buffers initialized with given value:
	//state vector: x, y, z, x', y', z', roll, pitch, yaw
	for (int ii=0;ii<SIZE; ii++){
	  stateQ[ii] = Eigen::VectorXd::Zero(9);
	  covP[ii] = Eigen::MatrixXd::Identity(9,9);
	  
	  inputU[ii] = Eigen::VectorXd::Zero(6);
	  matA[ii] = Eigen::MatrixXd::Identity(9,9);
	  matB[ii] = Eigen::MatrixXd::Zero(9,6);
	  imuMeasZ[ii] = Eigen::VectorXd::Zero(3);
	  imuInnovY[ii] = Eigen::VectorXd::Zero(3);
	  imuInnovCovS[ii] = Eigen::MatrixXd::Zero(3,3);
	}

	//constants?:
	inputCovQ = Eigen::MatrixXd::Zero(6,6);
	measCovRImu = Eigen::MatrixXd::Zero(3,3);
	measCovRPose = Eigen::MatrixXd::Zero(6,6);

	//sensors positions and orientations:
	sensor1Position = Eigen::Vector3d(0.03, -0.07, 0.1);
	sensor1Orientation = Eigen::Vector3d(0.2, -0.1, 0.3);
	sensor2Position = Eigen::Vector3d(-0.05, 0.08, 0.1);
	sensor2Orientation = Eigen::Vector3d(-0.1, 0.2, -0.3);

	//imu offset:
	linAccOffset = Eigen::Vector3d(0.2, 0.1, -0.3);
	angVelOffset = Eigen::Vector3d(-0.02, 0.008, -0.01);
	
	
  
  
	imuSub = nodeHandle.subscribe<sensor_msgs::Imu>("/firefly/ground_truth/imu", 1000, &KalmanPoseImuStateEstimator::imuCallback, this);
	//ros::Subscriber imuGTSub = n.subscribe<sensor_msgs::Imu>("/firefly/ground_truth/imu", 1000, imuCallback);
	
// 	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(options.tViconSeTopicName->getValue(),1, &ViconStateEstimator::viconCallback, this);
	
	pose1Sub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/firefly/ground_truth/pose", 1000, &KalmanPoseImuStateEstimator::poseCallback, this);
	pose2Sub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/firefly/ground_truth/pose2", 1000, &KalmanPoseImuStateEstimator::poseCallback, this);
	//ros::Subscriber poseGTSub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/firefly/ground_truth/pose", 1000, poseCallback);
	
	posePub = new ros::Publisher(nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/firefly/kalmanFilterPose",1000));
if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::willBecomeActive end");
}
void KalmanPoseImuStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	viconSub.shutdown();
}

void KalmanPoseImuStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string KalmanPoseImuStateEstimator::getName() const
{
//	return options.tKalmanPoseImuSeTopicName->getValue();
	return "KalmanPoseImuStateEstimator";
}

void KalmanPoseImuStateEstimator::viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::viconCallback init");
  while(busy){
    usleep(100);
  }
  busy=true;

  busy=false;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::viconCallback end");
}





void KalmanPoseImuStateEstimator::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  while(busy){
    usleep(100);
  }
  busy=true;

  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback init");
	if (firstPose){
		matCPose << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Zero(3,6), 
			Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
			
		for (int i=0; i<6; i++) {
			measCovRPose(i,i) = msg->pose.covariance[i*7];
			if (measCovRPose(i,i) == 0) measCovRPose(i,i) = 0.0001;
		}
		
		RPose1Inv.setRPY(sensor1Orientation(0), sensor1Orientation(1), sensor1Orientation(2));
		RPose1Inv = RPose1Inv.transpose();
		RPose2Inv.setRPY(sensor2Orientation(0), sensor2Orientation(1), sensor2Orientation(2));
		RPose2Inv = RPose2Inv.transpose();
		
		firstPose = false;
		busy=false;
		return;
	}

  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 0");	
	//sensor number:
	int sensorID = 0; //0 - for testing with ground truth data
	if (msg->header.frame_id == "firefly/pose_sensor1_link") sensorID = 1;
	else if (msg->header.frame_id == "firefly/pose_sensor2_link") sensorID = 2;
	else if (msg->header.frame_id == "firefly/base_link") sensorID = 0;
	else { 
		ROS_ERROR("Wrong Pose Sensor Msg frame_id");
		busy=false;
		return;
	}
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 1");	
	//update 1 variables:
	Eigen::VectorXd measZ = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd innovY = Eigen::VectorXd::Zero(6);
	Eigen::MatrixXd innovCovS = Eigen::MatrixXd::Zero(6,6);
	
	double poseTime = msg->header.stamp.toSec();

	//Roll, Pitch, Yaw from msg quaternion, the most elegant way I've found so far...
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3 R(q); 
	//"substract" rotation of the sensor
	if (sensorID == 1) R = R*RPose1Inv;
	else if (sensorID == 2) R = R*RPose2Inv;
	
	R.getRPY(roll, pitch, yaw);
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 2");	
	measZ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
		  roll, pitch, yaw;  
		  
	if (sensorID == 1) {
		measZ(0) = measZ(0) - sensor1Position(0);
		measZ(1) = measZ(1) - sensor1Position(1);
		measZ(2) = measZ(2) - sensor1Position(2);
	} else if (sensorID == 2) {
		measZ(0) = measZ(0) - sensor2Position(0);
		measZ(1) = measZ(1) - sensor2Position(1);
		measZ(2) = measZ(2) - sensor2Position(2);
	}
	
	//! measZ after transformation (for both sensors) has been compared with ground truth pose - all fine!
	
	//if (sensorID == 1) file1 << msg->header.stamp << ", " << measZ(0) << ", " << measZ(1) << ", " << measZ(2) << ", " << measZ(3) << ", " << measZ(4) << ", " << measZ(5) << "\n";
	//else if (sensorID == 2) file2 << msg->header.stamp << ", " << measZ(0) << ", " << measZ(1) << ", " << measZ(2) << ", " << measZ(3) << ", " << measZ(4) << ", " << measZ(5) << "\n";
	//else if (sensorID == 0) file3 << msg->header.stamp << ", " << measZ(0) << ", " << measZ(1) << ", " << measZ(2) << ", " << measZ(3) << ", " << measZ(4) << ", " << measZ(5) << "\n";
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3");	
	//pose sensor msg is delayed:
	double dT = 1;
	int kIndex = 0;
	for (int i=SIZE-1; i>=0; i--){
		if ((imuTime[i] - poseTime) < 0){			
			if ((poseTime - imuTime[i]) < dT) kIndex = i;
			else kIndex = i+1;
			break;
		}
		dT = imuTime[i] - poseTime;
	}
	
	
	// TODO ERASE THIS CONDITION: NOT IN ORIGINAL MARCIN CODE
  if(DEBUG_NOT_WELL)std::cout << kIndex  << std::endl;
	if (kIndex == SIZE){
	  kIndex--;
	}
  if(DEBUG_NOT_WELL)std::cout << kIndex  << std::endl;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3.2");	
	//UPDATE at kIndex:
	innovY = measZ - matCPose * stateQ[kIndex];
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3.3");
  if(DEBUG_WELL)ROS_ERROR("matCPose");
  if(DEBUG_WELL)std::cout << matCPose  << std::endl;
  if(DEBUG_WELL)ROS_ERROR("kIndex");
  if(DEBUG_WELL)std::cout << kIndex  << std::endl;
  if(DEBUG_WELL)ROS_ERROR("covP[kIndex]");
  if(DEBUG_WELL)std::cout << covP[kIndex]  << std::endl;
  if(DEBUG_WELL)ROS_ERROR("measCovRPose");
  if(DEBUG_WELL)std::cout << measCovRPose  << std::endl;
	innovCovS = matCPose * covP[kIndex] * matCPose.transpose() + measCovRPose;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3.4");	
	
	kalmanGain = covP[kIndex] * matCPose.transpose() * innovCovS.inverse();
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3.5");	
	
	stateQ[kIndex] = stateQ[kIndex] + kalmanGain * innovY;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 3.6");	
	covP[kIndex] = (Eigen::MatrixXd::Identity(9,9) - kalmanGain * matCPose) * covP[kIndex];
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback 4");	
	//Recalculation from kIndex till the end of the buffers:
	for (int i=kIndex+1; i<=SIZE-1; i++){
		//Re-Precidition:
		stateQ[i] = matA[i] * stateQ[i-1] + matB[i] * inputU[i];
		covP[i] = matA[i] * covP[i-1] * matA[i].transpose() + matB[i] * inputCovQ * matB[i].transpose();
		
		//Re-Update2
		imuInnovY[i] = imuMeasZ[i] - matCImu * stateQ[i];
		imuInnovCovS[i] = covP[i].bottomRightCorner(3,3) + measCovRImu;
	
		kalmanGain = covP[i] * matCImu.transpose() * imuInnovCovS[i].inverse();
	
		stateQ[i] = stateQ[i] + kalmanGain * imuInnovY[i];
		covP[i] = (Eigen::MatrixXd::Identity(9,9) - kalmanGain * matCImu) * covP[i];	
	}
	busy=false;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::poseCallback end");
}



void KalmanPoseImuStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    while(busy){
    usleep(100);
  }
  busy=true;

  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback init");
	if (firstImu){
		//TODO: z0 = 0.08 for all tasks in tasks 3&4, except T4.3, where z=0.18
		stateQ[SIZE-1](2) = 0.08;
	
		imuTime[SIZE-1] = msg->header.stamp.toSec();
		covP[SIZE-1] = covP[SIZE-1]*0.01; 
		
		inputCovQ(0,0) = msg->linear_acceleration_covariance[0];
		inputCovQ(1,1) = msg->linear_acceleration_covariance[1];
		inputCovQ(2,2) = msg->linear_acceleration_covariance[2];
		inputCovQ(3,3) = msg->angular_velocity_covariance[0];
		inputCovQ(4,4) = msg->angular_velocity_covariance[1];
		inputCovQ(5,5) = msg->angular_velocity_covariance[2];
		
		measCovRImu(0,0) = msg->orientation_covariance[0];
		measCovRImu(1,1) = msg->orientation_covariance[4];
		measCovRImu(2,2) = msg->orientation_covariance[4]; //!
		
		for (int i=0; i<3; i++) {
			if (inputCovQ(i,i) == 0) inputCovQ(i,i) = 0.002;
			if (measCovRImu(i,i) == 0) measCovRImu(i,i) = 0.00001;
		}
		
		for (int i=3; i<6; i++) {
			if (inputCovQ(i,i) == 0) inputCovQ(i,i) = 0.00001;
		}
		
		matCImu << Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(3,3);
		
		firstImu = false;
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback end first");
  busy=false;
		return;
	}
	
	//sensorID for testing with ground truth data - to avoid offsets
	int sensorID = 1; //1 - sensor, 0 - ground truth
	if (msg->header.frame_id == "imu_link") sensorID = 0;
	
	//model, prediciton variables
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback 0");
	Eigen::VectorXd linAcc = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd angVel = Eigen::VectorXd::Zero(3);

	//shifting the buffers:
	for (int i=0; i<=(SIZE-2); i++){
		stateQ[i] = stateQ[i+1];
		covP[i] = covP[i+1];
		imuTime[i] = imuTime[i+1];
		inputU[i] = inputU[i+1];
		matA[i] = matA[i+1];
		matB[i] = matB[i+1];
		imuMeasZ[i] = imuMeasZ[i+1];
		imuInnovY[i] = imuInnovY[i+1];
		imuInnovCovS[i] = imuInnovCovS[i+1];
	}
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback 1");
	imuTime[SIZE-1] = msg->header.stamp.toSec();
	double dt = imuTime[SIZE-1] - imuTime[SIZE-2];
	
	matA[SIZE-1](0,3) = dt;
	matA[SIZE-1](1,4) = dt;
	matA[SIZE-1](2,5) = dt;

	matB[SIZE-1] << Eigen::MatrixXd::Zero(3,6), Eigen::MatrixXd::Identity(6,6)*dt;
	
	linAcc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	angVel << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	//offset compensation:
	if (sensorID != 0) {
		linAcc -= linAccOffset;
		angVel -= angVelOffset;
	}
	
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback 2");
	//Q->W transformation
	tf::Matrix3x3 RqwTf;
	RqwTf.setRPY(stateQ[SIZE-2][6], stateQ[SIZE-2][7], stateQ[SIZE-2][8]);
	Eigen::Matrix3d Rqw;
	tf::matrixTFToEigen(RqwTf, Rqw);
	
	linAcc = Rqw*linAcc;
	angVel = Rqw*angVel;
	//gravity compensation
	linAcc[2] = linAcc[2] - 9.8;
	
	inputU[SIZE-1] << linAcc, angVel;

	//PREDICTION:
	stateQ[SIZE-1] = matA[SIZE-1]*stateQ[SIZE-2] + matB[SIZE-1]*inputU[SIZE-1];
	covP[SIZE-1] = matA[SIZE-1]*covP[SIZE-2]*matA[SIZE-1].transpose() + matB[SIZE-1]*inputCovQ*matB[SIZE-1].transpose();
	
	//UPDATE Imu:
	//roll, pitch, yaw from msg quat.:
	double roll, pitch, yaw;
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->orientation, q);
	tf::Matrix3x3 R(q); 
	R.getRPY(roll, pitch, yaw);
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback 3");
	
	imuMeasZ[SIZE-1] << roll, pitch, yaw;

	imuInnovY[SIZE-1] = imuMeasZ[SIZE-1] - matCImu * stateQ[SIZE-1];
	imuInnovCovS[SIZE-1] = covP[SIZE-1].bottomRightCorner(3,3) + measCovRImu;
	
	kalmanGain = covP[SIZE-1] * matCImu.transpose() * imuInnovCovS[SIZE-1].inverse();
	
	stateQ[SIZE-1] = stateQ[SIZE-1] + kalmanGain * imuInnovY[SIZE-1];
	covP[SIZE-1] = (Eigen::MatrixXd::Identity(9,9) - kalmanGain * matCImu) * covP[SIZE-1];
	
	//publish:
	filteredPose.header.seq++;
	filteredPose.header.frame_id = "";
	filteredPose.header.stamp = msg->header.stamp;
	
	filteredPose.pose.pose.position.x = stateQ[SIZE-1](0);
	filteredPose.pose.pose.position.y = stateQ[SIZE-1](1);
	filteredPose.pose.pose.position.z = stateQ[SIZE-1](2);
  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback 4");
	
	q.setRPY(stateQ[SIZE-1](6), -stateQ[SIZE-1](7), stateQ[SIZE-1](8));
	tf::quaternionTFToMsg(q, filteredPose.pose.pose.orientation);

	filteredPose.pose.covariance[0] = covP[SIZE-1](0,0);
	filteredPose.pose.covariance[7] = covP[SIZE-1](1,1);
	filteredPose.pose.covariance[14] = covP[SIZE-1](2,2);
	filteredPose.pose.covariance[21] = covP[SIZE-1](6,6);
	filteredPose.pose.covariance[28] = covP[SIZE-1](7,7);
	filteredPose.pose.covariance[35] = covP[SIZE-1](8,8);
	
	posePub->publish(filteredPose);
	
	
	Position3D pos(stateQ[SIZE-1](0), -stateQ[SIZE-1](1), -stateQ[SIZE-1](2));
	Quaternion quat(q[3], q[0], q[1], q[2]);
	Velocity3D lin_vel(stateQ[SIZE-1](3), -stateQ[SIZE-1](4), -stateQ[SIZE-1](5));
	Velocity3D ang_vel(angVel(0), angVel(1), angVel(2));
	
		// empty msg
	TKState tStateMsg;
	tStateMsg.time = Time(msg->header.stamp);
	tStateMsg.position = pos;
	tStateMsg.linVelocity = lin_vel;
	tStateMsg.orientation = quat;
	tStateMsg.angVelocity = ang_vel;


	stateEstimatorController.activeStateCallBack(tStateMsg);

  if(DEBUG_WELL)ROS_ERROR("KalmanPoseImuStateEstimator::imuCallback end");
  busy=false;
}

} /* namespace telekyb_state */
