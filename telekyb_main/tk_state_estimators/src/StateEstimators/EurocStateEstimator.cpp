/*
 * EurocStateEstimator.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */
#include <StateEstimators/EurocStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKState.h>

#define DEBUG_WELL 0
#define DEBUG_NOT_WELL 0

PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::EurocStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace state_estimators_plugin {

EurocStateEstimatorOptions::EurocStateEstimatorOptions()
	: OptionContainer("EurocStateEstimator")
{
	tEurocStateEstimatorSeTopicName = addOption<std::string>("tEurocStateEstimatorSeTopicName","TopicName of EurocStateEstimator Sensor.","undef",true,true);
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tEurocStateEstimatorToNEDMatrix = addOption<Eigen::Matrix3d>("tEurocStateEstimatorToNEDMatrix","ConversionMatrix from EurocStateEstimator to NED", m, false, true);

	tEurocStateEstimatorVelFilterFreq = addOption<double>("tEurocStateEstimatorVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tEurocStateEstimatorSmoothVelFilterFreq = addOption<double>("tEurocStateEstimatorSmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tEurocStateEstimatorAngFilterFreq = addOption<double>("tEurocStateEstimatorAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tEurocStateEstimatorSampleTime = addOption<double>("tEurocStateEstimatorSampleTime",
			"Sampling Time of EurocStateEstimator System. Default is 120Hz", 0.008333333333, false, true);

	tEurocStateEstimatorPublishSmoothVel = addOption<bool>("tEurocStateEstimatorPublishSmoothVel",
			"Publish TKState with smoothed Velocity", false, false, true);
}

void EurocStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocStateEstimatorVelFilterFreq->getValue(),
				1.0,
				options.tEurocStateEstimatorSampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocStateEstimatorSmoothVelFilterFreq->getValue(),
				1.0,
				options.tEurocStateEstimatorSampleTime->getValue());
	}


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocStateEstimatorAngFilterFreq->getValue(),
				1.0,
				options.tEurocStateEstimatorSampleTime->getValue());
	}
}

// We should outsource this somehow!
void EurocStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
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

void EurocStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();
}
void EurocStateEstimator::willBecomeActive()
{
	pose1Sub = nodeHandle.subscribe<nav_msgs::Odometry>(options.tEurocStateEstimatorSeTopicName->getValue().c_str(), 1000, &EurocStateEstimator::poseCallback, this);
	
	if (options.tEurocStateEstimatorPublishSmoothVel->getValue()) {
		smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}
	
	
	TKState tStateMsg;
	tStateMsg.time = Time(0);
	tStateMsg.position = Position3D(0.0, 0.0, 0.0);
	tStateMsg.linVelocity = Velocity3D(0.0, 0.0, 0.0);
	tStateMsg.orientation = Quaternion(1.0, 0.0, 0.0, 0.0);
	tStateMsg.angVelocity = Velocity3D(0.0, 0.0, 0.0);

	stateEstimatorController.activeStateCallBack(tStateMsg);
}
void EurocStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	pose1Sub.shutdown();
}

void EurocStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string EurocStateEstimator::getName() const
{
//	return options.tEurocStateEstimatorSeTopicName->getValue();
	return "EurocStateEstimator";
}

void EurocStateEstimator::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// StateEstimatorController neest a telekyb::TKState
	Position3D posVicon(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	Quaternion quatVicon(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

	Position3D posNED = options.tEurocStateEstimatorToNEDMatrix->getValue() * posVicon;

//	ROS_INFO_STREAM("posVicon: " << posVicon);
//	ROS_INFO_STREAM("posNED: " << posNED);

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
	Velocity3D velNED(output);
	Velocity3D smoothVelNED(outputMore);
	
// 	std::cout << " " << velNED(0) << " " << velNED(1) << " " << velNED(2);
	
	velNED(0) = msg->twist.twist.linear.x;
	velNED(1) = -msg->twist.twist.linear.y;
	velNED(2) = -msg->twist.twist.linear.z;
// 	std::cout << " " << velNED(0) << " " << velNED(1) << " " << velNED(2) << std::endl;

	//ROS_INFO_STREAM("velNED: " << velNED);
	//moreFilteredVel = Velocity3D(outputMore);


	double outputQuat[4];

	input[0] = quatVicon.w();
	angFilter[0]->step(input, outputQuat[0]);
	input[0] = quatVicon.x();
	angFilter[1]->step(input, outputQuat[1]);
	input[0] = quatVicon.y();
	angFilter[2]->step(input, outputQuat[2]);
	input[0] = quatVicon.z();
	angFilter[3]->step(input, outputQuat[3]);

	// Constructor: w,x,y,z !
	Quaternion quatRates(outputQuat[0], outputQuat[1], outputQuat[2], outputQuat[3]);

	//Velocity3D vQuatVelVicon(outputVQuat);
	Velocity3D angVelocity;
	velQuatToBodyOmega(quatVicon, quatRates, angVelocity);

	Quaternion quatNED(quatVicon);
	quatNED.vec() = options.tEurocStateEstimatorToNEDMatrix->getValue() * quatNED.vec();
	//ROS_INFO_STREAM("Orientation RPY: " << quatVicon.toRotationMatrix().eulerAngles(0,1,2));
	//ROS_INFO_STREAM("OrientationNED RPY: " << quatNED.toRotationMatrix().eulerAngles(0,1,2));

	Velocity3D angVelocityNED = options.tEurocStateEstimatorToNEDMatrix->getValue() * angVelocity;
	

	//ROS_INFO_STREAM("Ang Vel NED : " << angVelocityNED);

	// empty msg
	TKState tStateMsg;
	tStateMsg.time = Time(msg->header.stamp);
	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelocityNED;


	stateEstimatorController.activeStateCallBack(tStateMsg);

	// Publish Smooth Velocity
	if (options.tEurocStateEstimatorPublishSmoothVel->getValue()) {
		telekyb_msgs::TKState smoothVelMsg;
		tStateMsg.toTKStateMsg(smoothVelMsg);
		smoothVelMsg.header.stamp = ros::Time::now();
		smoothVelMsg.twist.linear.x = smoothVelNED(0);
		smoothVelMsg.twist.linear.y = smoothVelNED(1);
		smoothVelMsg.twist.linear.z = smoothVelNED(2);
		smoothVelPub.publish(smoothVelMsg);
	}
}

} /* namespace telekyb_state */
