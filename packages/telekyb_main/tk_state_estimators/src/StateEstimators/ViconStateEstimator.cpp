/*
 * ViconStateEstimator.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */
#include <StateEstimators/ViconStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKState.h>




PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::ViconStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace state_estimators_plugin {

ViconStateEstimatorOptions::ViconStateEstimatorOptions()
	: OptionContainer("ViconStateEstimator")
{
	tViconSeTopicName = addOption<std::string>("tViconSeTopicName","TopicName of Vicon Sensor.","undef",true,true);
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tViconToNEDMatrix = addOption<Eigen::Matrix3d>("tViconToNEDMatrix","ConversionMatrix from Vicon to NED", m, false, true);


	tViconRoboCentric = addOption<bool>("tViconRoboCentric",
			"If true, tkstate is in robot's frame", false, false, true);

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
}

void ViconStateEstimator::initVelocityFilters()
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


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tViconAngFilterFreq->getValue(),
				1.0,
				options.tViconSampleTime->getValue());
	}
}

// We should outsource this somehow!
void ViconStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
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

void ViconStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();
}
void ViconStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
			options.tViconSeTopicName->getValue(),1, &ViconStateEstimator::viconCallback, this);

	if (options.tViconPublishSmoothVel->getValue()) {
		smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}
}
void ViconStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	viconSub.shutdown();
}

void ViconStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string ViconStateEstimator::getName() const
{
//	return options.tViconSeTopicName->getValue();
	return "ViconStateEstimator";
}

void ViconStateEstimator::viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// StateEstimatorController neest a telekyb::TKState
//	ROS_INFO("Received Callback!");
	Position3D posVicon(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	Quaternion quatVicon(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

	Position3D posNED = options.tViconToNEDMatrix->getValue() * posVicon;

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
	quatNED.vec() = options.tViconToNEDMatrix->getValue() * quatNED.vec();
	//ROS_INFO_STREAM("Orientation RPY: " << quatVicon.toRotationMatrix().eulerAngles(0,1,2));
	//ROS_INFO_STREAM("OrientationNED RPY: " << quatNED.toRotationMatrix().eulerAngles(0,1,2));

	Velocity3D angVelocityNED = options.tViconToNEDMatrix->getValue() * angVelocity;

	//ROS_INFO_STREAM("Ang Vel NED : " << angVelocityNED);


	// empty msg
	TKState tStateMsg;
	tStateMsg.time = Time(msg->header.stamp);
	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelocityNED;


	//to local, setting yaw to 0
	if (options.tViconRoboCentric->getValue()) {
		TELEKYB_NAMESPACE::Vector3D orientation;
		orientation = tStateMsg.getEulerRPY();
		
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(-orientation(2), Eigen::Vector3d::UnitZ());
	
		TELEKYB_NAMESPACE::Vector3D appVel = m*velNED;
		TELEKYB_NAMESPACE::Vector3D appPos = posNED;
	
		appPos(0) = 0.0;
		appPos(1) = 0.0;
		//appPos(2) = 0.0;
	
		double w,x,y,z;
	//	Setting the yaw angle to zero
		orientation(2) = 0;
		w = cos(orientation(0)/2.0) * cos(orientation(1)/2.0) * cos(orientation(2)/2.0) + sin(orientation(0)/2.0) * sin(orientation(1)/2.0) * sin(orientation(2)/2.0);
		x = sin(orientation(0)/2.0) * cos(orientation(1)/2.0) * cos(orientation(2)/2.0) - cos(orientation(0)/2.0) * sin(orientation(1)/2.0) * sin(orientation(2)/2.0);
		y = cos(orientation(0)/2.0) * sin(orientation(1)/2.0) * cos(orientation(2)/2.0) + sin(orientation(0)/2.0) * cos(orientation(1)/2.0) * sin(orientation(2)/2.0);
		z = cos(orientation(0)/2.0) * cos(orientation(1)/2.0) * sin(orientation(2)/2.0) - sin(orientation(0)/2.0) * sin(orientation(1)/2.0) * cos(orientation(2)/2.0);
		
		Quaternion newQuat(w,x,y,z);

		tStateMsg.position = appPos;
		tStateMsg.linVelocity = appVel;
		tStateMsg.orientation = newQuat;
	}
	
	stateEstimatorController.activeStateCallBack(tStateMsg);

	// Publish Smooth Velocity
	if (options.tViconPublishSmoothVel->getValue()) {
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
