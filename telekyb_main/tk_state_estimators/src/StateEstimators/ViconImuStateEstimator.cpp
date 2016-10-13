/*
 * ViconImuStateEstimator.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: mriedel & rspica
 */

#include <StateEstimators/ViconImuStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <telekyb_msgs/TKState.h>


PLUGINLIB_EXPORT_CLASS( telekyb_state::ViconImuStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

ViconImuStateEstimatorOptions::ViconImuStateEstimatorOptions()
	: OptionContainer("ViconImuStateEstimator")
{
	tViconSeTopicName = addOption<std::string>("tViconSeTopicName","TopicName of Vicon Sensor.","undef",true,true);
	tImuSeTopicName = addOption<std::string>("tImuSeTopicName","TopicName of Imu Sensor.","undef",true,true);

	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tViconToNEDMatrix = addOption<Eigen::Matrix3d>("tViconToNEDMatrix","ConversionMatrix from Vicon to NED", m, false, true);

	tViconToNEDQuaternion = addOption<Eigen::Quaterniond>("tViconToNEDQuaternion","ConversionQuaternion from Vicon to NED", Eigen::Quaterniond(1.0,0.0,0.0,0.0), false, true);

	tViconVelFilterFreq = addOption<double>("tViconVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tViconSmoothVelFilterFreq = addOption<double>("tViconSmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tViconAngFilterFreq = addOption<double>("tViconAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tViconSampleTime = addOption<double>("tViconSampleTime",
			"Sampling Time of Vicon System. Default is 120Hz", 0.008333333333, false, true);

	tViconPublishSmoothVel = addOption<bool>("tViconPublishSmoothVel",
			"Publish TKState with smoothed Velocity", false, false, true);

	timeStep = addOption<double>("timeStep","Time step for state publishing",4e-3,false,true);

	tIntegrateLinVel = addOption<bool>("tIntegrateLinVel",
				"Integrate linear velocity when Vicon position is not available", false, false, false);
	tIntegrateAngVel = addOption<bool>("tIntegrateAngVel",
				"Integrate angular velocity when Vicon orientation is not available", false, false, false);

}

//void ViconImuStateEstimator::ssxCallback(const telekyb_msgs::TKState::ConstPtr& stateMsg)
//{
//	// StateEstimatorController neest a telekyb::TKState
//	TKState tStateMsg(*stateMsg);
//	stateEstimatorController.activeStateCallBack(tStateMsg);
//}



void ViconImuStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tViconVelFilterFreq->getValue(),
				1.0,
				options.tViconSampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tViconSmoothVelFilterFreq->getValue(),
				1.0,
				options.tViconSampleTime->getValue());
	}
}

void ViconImuStateEstimator::initialize()
{
//	debugPub = nodeHandle.advertise<telekyb_msgs::TKState>("debugState",1);
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();

	spinning = true;
	thread = boost::thread(&ViconImuStateEstimator::spin, this);
}

void ViconImuStateEstimator::spin()
{
	while(spinning)
	{
		rtTimer.reset();
		publishState();

		Time toSleep = Time(options.timeStep->getValue()) - rtTimer.getElapsed();
		if (toSleep>0) {
			toSleep.sleep();
		} else {
			ROS_WARN("State estimation was too slow!");
		}
	}
}

void ViconImuStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			options.tViconSeTopicName->getValue(),1, &ViconImuStateEstimator::viconCallback, this);

	imuSub = nodeHandle.subscribe<sensor_msgs::Imu>(
			options.tImuSeTopicName->getValue(),1, &ViconImuStateEstimator::imuCallback, this);

	if (options.tViconPublishSmoothVel->getValue()) {
		smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}

//	debugPub = nodeHandle.advertise<telekyb_msgs::TKState>("debugState",1);
//	ssxSub = nodeHandle.subscribe<telekyb_msgs::TKState>(
//			options.tSsxSeTopicName->getValue(),1, &ViconImuStateEstimator::ssxCallback, this);

}

void ViconImuStateEstimator::willBecomeInActive()
{
	viconSub.shutdown();
	imuSub.shutdown();
	if (options.tViconPublishSmoothVel->getValue()) {
		smoothVelPub.shutdown();
	}
//	debugPub.shutdown();
}

void ViconImuStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	spinning = false;
	thread.join();
}

std::string ViconImuStateEstimator::getName() const
{
	return options.tViconSeTopicName->getValue();
}

void ViconImuStateEstimator::viconCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	Position3D posVicon(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	Quaternion quatVicon(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

	/*velocity NED */
	double output[3],outputMore[3];
	std::vector<double> input(1);
	input[0]=posNED(0);
	velFilter[0]->step(input, output[0]);
	smoothVelFilter[0]->step(input, outputMore[0]);
	input[0] = posNED(1);
	velFilter[1]->step(input, output[1]);
	smoothVelFilter[1]->step(input, outputMore[1]);
	input[0] = posNED(2);
	velFilter[2]->step(input, output[2]);
	smoothVelFilter[2]->step(input, outputMore[2]);

	boost::mutex::scoped_lock scopedLock(viconMutex);

	posNED = options.tViconToNEDQuaternion->getValue()*posVicon;
	velNED << output[0], output[1], output[2];
	smoothVelNED << outputMore[0], outputMore[1], outputMore[2];
	quatNED = options.tViconToNEDQuaternion->getValue()*quatVicon;
	scopedLock.unlock();
}

void ViconImuStateEstimator::publishState()
{
	TKState tStateMsg;

	// Integrate time
	boost::mutex::scoped_lock scopedLock(timeMutex);
	telekyb::Time elapsedTime = intTimer.getElapsed();
	intTimer.reset();
	intTime = intTime + elapsedTime;
	tStateMsg.time = intTime;
	scopedLock.unlock();


	boost::mutex::scoped_lock scopedViconLock(viconMutex);
	// empty msg
	if (options.tIntegrateLinVel->getValue()){
		posNED = posNED + velNED*elapsedTime.toDSec();
	}
	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;

	boost::mutex::scoped_lock scopedImuLock(imuMutex);

	if (options.tIntegrateAngVel->getValue()){
		double angVelSqNorm = angVelocityNED.squaredNorm();
		Eigen::Quaterniond deltaQuat;
		double deltaT = elapsedTime.toDSec();
		deltaQuat.w() = 1.0 - deltaT*deltaT*angVelSqNorm/8.0;
		deltaQuat.vec() = (.5*deltaT - deltaT*deltaT*deltaT*angVelSqNorm/24) * angVelocityNED;
		quatNED = quatNED*deltaQuat;
	}
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelocityNED;

	scopedImuLock.unlock();

	// Publish Smooth Velocity
	if (options.tViconPublishSmoothVel->getValue()) {
		telekyb_msgs::TKState smoothVelMsg;
		tStateMsg.toTKStateMsg(smoothVelMsg);
		//smoothVelMsg.header.stamp = ros::Time::now();
		smoothVelMsg.twist.linear.x = smoothVelNED(0);
		smoothVelMsg.twist.linear.y = smoothVelNED(1);
		smoothVelMsg.twist.linear.z = smoothVelNED(2);
		smoothVelPub.publish(smoothVelMsg);
	}

	scopedViconLock.unlock();

	stateEstimatorController.activeStateCallBack(tStateMsg);
//	telekyb_msgs::TKState msg;
//	tStateMsg.toTKStateMsg(msg);
//	debugPub.publish(msg);

}

void ViconImuStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	boost::mutex::scoped_lock scopedLock(imuMutex);

	angVelocityNED(0) = msg->angular_velocity.x;
	angVelocityNED(1) = msg->angular_velocity.y;
	angVelocityNED(2) = msg->angular_velocity.z;
	
	scopedLock.unlock();
	scopedLock = boost::mutex::scoped_lock(timeMutex);
	
	intTime = Time(msg->header.stamp);
	intTimer.reset();

	scopedLock.unlock();
	
}


} /* namespace telekyb_state */
