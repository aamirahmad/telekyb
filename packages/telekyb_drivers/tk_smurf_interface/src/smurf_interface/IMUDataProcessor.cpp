/*
 * IMUDataProcessor.cpp
 *
 *  Created on: Aug 16, 2012
 *      Author: mriedel
 */

#include "IMUDataProcessor.hpp"

// ROS Module
#include <telekyb_base/ROS/ROSModule.hpp>


// Defines
#define ACCEL_CENTER 512
#define ACCEL_MAX 1024
#define GYRO_CENTER 512
#define GYRO_MAX 1024



namespace TELEKYB_NAMESPACE {

IMUDataProcessor::IMUDataProcessor()
	: driftEstimRunning(true)
{
	ros::NodeHandle n = ROSModule::Instance().getMainNodeHandle();
	// create publisher
	imuPublisher = n.advertise<tk_draft_msgs::TKSmallImu>(options.tTopicName->getValue() ,10);
	if (options.tPublishIMUAsWrenchMsg->getValue()){
	  imuPublisherAsWrenchMsg = n.advertise<geometry_msgs::Wrench>(options.tIMUAsWrenchTopicName->getValue() ,10);
	}
}

IMUDataProcessor::~IMUDataProcessor() {
	// TODO Auto-generated destructor stub
}


bool IMUDataProcessor::init() {

	if (options.tInitialDriftEstim->getValue()) {
		Eigen::Vector3d gyroOffsets = options.tGyroOffsets->getValue();
		if (calibrator.driftEstimation(gyroOffsets)) {
			// set again after successful calibration
			options.tGyroOffsets->setValue(gyroOffsets);
			//ROS_INFO_STREAM(options.tGyroOffsets->getValue());
		} else {
			// calibration failed
			ROS_ERROR("An Error occurred during Gyro Drift Estimation");
			return false;
		}
	}

	driftEstimRunning = false;
	return true;
}

void IMUDataProcessor::processRawIMUData(const RawImuData& data)
{
	if (driftEstimRunning) {
		// forward to calibrator
		calibrator.processRawIMUData(data);
		return;
	}

	// Normal mode

	// Calculate SI Data
	Eigen::Vector3d accelData(data.accX, data.accY, data.accZ);
	Eigen::Vector3d gyroData(data.gyroRoll, data.gyroPitch, data.gyroYaw);

	// Offsets
	accelData -= options.tAccOffsets->getValue();
	gyroData -= options.tGyroOffsets->getValue();

	//ROS_INFO_STREAM("Acc: " << std::endl << accelData);

	// Scale
	gyroData = (gyroData / GYRO_MAX) - Eigen::Vector3d::Constant(0.5);
	gyroData *= options.tGyroScale->getValue();

	accelData = (accelData / ACCEL_MAX) - Eigen::Vector3d::Constant(0.5);
	accelData *= options.tAccScale->getValue();

//	ROS_INFO_STREAM("Gyro: " << std::endl << gyroData);
//	ROS_INFO_STREAM("Accel: " << std::endl << accelData);

	// process IMU Data
	rosMsg.header.stamp = ros::Time::now();

	// Acc
	rosMsg.linear_acceleration.x = -accelData(0);
	rosMsg.linear_acceleration.y = -accelData(1);
	rosMsg.linear_acceleration.z = -accelData(2);

	// Gyro
	rosMsg.angular_velocity.x = -gyroData(0); // roll inverted
	rosMsg.angular_velocity.y = -gyroData(1); // pitch inverted
	rosMsg.angular_velocity.z = gyroData(2);
	
	// IMU DATA ARE NOW IN NED BODY FRAME  
	
	if (options.tIMUReferenceFrame->getValue() == 1) // CONVERT TO NWU BODY FRAME
	{
	  rosMsg.linear_acceleration.y = -rosMsg.linear_acceleration.y;
	  rosMsg.linear_acceleration.z = -rosMsg.linear_acceleration.z;
	  rosMsg.angular_velocity.y = -rosMsg.angular_velocity.y;
	  rosMsg.angular_velocity.z = -rosMsg.angular_velocity.z;	  
	}
	
	imuPublisher.publish(rosMsg);
	
	if (options.tPublishIMUAsWrenchMsg->getValue()){
	  geometry_msgs::Wrench rosMsgWrench;
	  rosMsgWrench.force.x = -accelData(0);
	  rosMsgWrench.force.y = -accelData(1);
	  rosMsgWrench.force.z = -accelData(2);
	  rosMsgWrench.torque.x = -gyroData(0); // roll inverted
	  rosMsgWrench.torque.y = -gyroData(1); // pitch inverted
	  rosMsgWrench.torque.z = gyroData(2);
	  imuPublisherAsWrenchMsg.publish(rosMsgWrench);
	}
}

//bool IMUDataProcessor::driftEstimation()
//{
//	Timer timeoutTimer;
//
//	while(timeoutTimer.getElapsed().toDSec() < options.tDriftEstimTimeout->getValue()) {
//
//		if (driftEstimDone.x && driftEstimDone.y && driftEstimDone.z) {
//			// all done
//			//ROS_INFO("Successfully did driftEstimation!");
//			returnValue = true;
//			break;
//		}
//
//
//		usleep(1000);
//	}
//
//
//	return true;
//}

//void IMUDataProcessor::attitudeEstimation(const RawImuData& data)
//{
//	Time passed = elapsedTime.getElapsed();
//	elapsedTime.reset();
//
//	/************* ROLL ************/
//	rollDot_acc = ACC_FILT_GAIN * c_asin(data.accY - ACCEL_OFFSET_ROLL) - (ACC_FILT_GAIN * estRoll);
//	estRollVel = (-(data.gyroRoll - GYRO_OFFSET) * GYRO_SCALING) / GYRO_ZERO_VAL + driftRoll;
//	estRollVel = GYRO_MULT_NUM*estRollVel/GYRO_MULT_DEN;
//
//	rollDot = rollDot_acc	+	estRollVel;
//	estRoll = estRoll + (rollDot * passed.toDSec()); //estimation in 10000 parts of degrees
//
//
//	/************* PITCH ************/
//	pitchDot_acc = -ACC_FILT_GAIN *c_asin(data.accX - ACCEL_OFFSET_PITCH) - (ACC_FILT_GAIN*estPitch);
//	estPitchVel=(-(data.gyroPitch - GYRO_OFFSET) * GYRO_SCALING) / GYRO_ZERO_VAL + driftPitch;
//	estPitchVel = GYRO_MULT_NUM*estPitchVel/GYRO_MULT_DEN;
//
//	pitchDot = pitchDot_acc	+	estPitchVel;
//	estPitch = estPitch + (pitchDot * passed.toDSec());
//
//	if (outputTimer.getElapsed().toDSec() > 0.1)
//	{
//		ROS_INFO("EstRoll: %04f", (double)estRoll/100000);
//		ROS_INFO("EstPitch: %04f", (double)estPitch/100000);
//		outputTimer.reset();
//	}
//	/************* YAW ************/
//	estYawVel=((data.gyroYaw - GYRO_OFFSET) * GYRO_SCALING) / GYRO_ZERO_VAL + driftYaw;
//
//	/*** first-order low pass filtering of the angular velocity, to be used in the control loop, frequency at wn rad/s ***/
//	estRollVel_fil += (LOW_PASS_SCALING*(estRollVel - estRollVel_fil))/(485);
//
//	estPitchVel_fil += (LOW_PASS_SCALING*(estPitchVel - estPitchVel_fil))/(485);
//
//	estYawVel_fil += (LOW_PASS_SCALING*(estYawVel - estYawVel_fil))/(485);
//}
//
//int IMUDataProcessor::c_asin(int input) const {
//	double val = (asin((double)input/200)*180/M_PI) * 100000;
//	return (int)val;
//}

} /* namespace telekyb */
