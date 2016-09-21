/*
 * ViconVisionStateEstimator.cpp
 *
 *  Created on: Oct 8, 2013
 *      Author: vgrabe
 */

#include <telekyb_base/ROS.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include "StateEstimators/ViconVisionStateEstimator.hpp"

PLUGINLIB_EXPORT_CLASS( telekyb_state::ViconVisionStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

using namespace TELEKYB_NAMESPACE;
namespace telekyb_state {

ViconVisionStateEstimatorOptions::ViconVisionStateEstimatorOptions()
	: OptionContainer("ViconVisionStateEstimator"){
	tViconSeTopicName = addOption<std::string>("tViconSeTopicName","TopicName of Vicon Sensor","undef",true,true);
	tVisionSeTopicName = addOption<std::string>("tVisionSeTopicName","TopicName of visual velocity estimate","/est_vInB_scaled",false, true);
	tJoyTopicName = addOption<std::string>("tJoystickTopicName","TopicName of the joystick","/TeleKyb/tJoy/joy_exp",false, true);

	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tViconToNEDMatrix = addOption<Eigen::Matrix3d>("tViconToNEDMatrix","ConversionMatrix from Vicon to NED", m, false, true);

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



ViconVisionStateEstimator::ViconVisionStateEstimator():
		useVision(false),
		numberOfDerivations(0)
{

}

ViconVisionStateEstimator::~ViconVisionStateEstimator() {
	// TODO Auto-generated destructor stub
}


void ViconVisionStateEstimator::initVelocityFilters()
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

void ViconVisionStateEstimator::velQuatToBodyOmega(const Eigen::Quaterniond& quat, const Eigen::Quaterniond& quatRates, Eigen::Vector3d& bodyOmega)
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

void ViconVisionStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	initVelocityFilters();
}
void ViconVisionStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
			options.tViconSeTopicName->getValue(),1, &ViconVisionStateEstimator::viconCallback, this);
	visionSub = nodeHandle.subscribe<geometry_msgs::Vector3Stamped>(
			options.tVisionSeTopicName->getValue(),1, &ViconVisionStateEstimator::visionCallback, this);
	joySub = nodeHandle.subscribe<sensor_msgs::Joy>(
			options.tJoyTopicName->getValue(),1, &ViconVisionStateEstimator::joystickCallback, this);
	if (options.tViconPublishSmoothVel->getValue()) {
        smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}
	debugPublisher = nodeHandle.advertise<geometry_msgs::Vector3Stamped>("viconVelDebug", 1);

}
void ViconVisionStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	viconSub.shutdown();
}

void ViconVisionStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}
	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

void ViconVisionStateEstimator::submitState(const TKState& state, bool isVision){
	if(useVision==isVision){
		stateEstimatorController.activeStateCallBack(state);
	}
}

std::string inline ViconVisionStateEstimator::getName() const
{
	return "ViconVisionStateEstimator";
}

void ViconVisionStateEstimator::viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	Eigen::Vector3d posVicon(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	Eigen::Quaterniond quatVicon(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

	Eigen::Vector3d posNED = options.tViconToNEDMatrix->getValue() * posVicon;

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
	Eigen::Vector3d velNED(output);
	Eigen::Vector3d smoothVelNED(outputMore);


	double outputQuat[4];
	input[0] = quatVicon.w();
	angFilter[0]->step(input, outputQuat[0]);
	input[0] = quatVicon.x();
	angFilter[1]->step(input, outputQuat[1]);
	input[0] = quatVicon.y();
	angFilter[2]->step(input, outputQuat[2]);
	input[0] = quatVicon.z();
	angFilter[3]->step(input, outputQuat[3]);

	//prevent artifacts during the first computation of the derivative
	if(numberOfDerivations < 10){
		numberOfDerivations++;
		return;
	}

	Eigen::Quaterniond quatRates(outputQuat[0], outputQuat[1], outputQuat[2], outputQuat[3]);

	Eigen::Vector3d angVelocity;
	velQuatToBodyOmega(quatVicon, quatRates, angVelocity);

	Eigen::Quaterniond quatNED(quatVicon);
	quatNED.vec() = options.tViconToNEDMatrix->getValue() * quatNED.vec();

	Eigen::Vector3d angVelocityNED = options.tViconToNEDMatrix->getValue() * angVelocity;

	// empty msg
	TKState tStateMsg;
	tStateMsg.time = Time(msg->header.stamp);
	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelocityNED;

	geometry_msgs::Vector3Stamped debugMsg;
	debugMsg.header.stamp = ros::Time::now();
	debugMsg.vector.x = tStateMsg.linVelocity(0);
	debugMsg.vector.y = tStateMsg.linVelocity(1);
	debugMsg.vector.z = tStateMsg.linVelocity(2);
	debugPublisher.publish(debugMsg);

	submitState(tStateMsg, false);
//	stateEstimatorController.activeStateCallBack(tStateMsg);

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

	//For the simulation of pure velocity state estimates only
//	TKState visionState;
//	visionState.time = Time(msg->header.stamp);
//	visionState.position = Eigen::Vector3d::Zero();
//	visionState.linVelocity = tStateMsg.linVelocity;
//	visionState.orientation = Eigen::Quaterniond::Identity();
//	visionState.angVelocity = Eigen::Vector3d::Zero();
//	submitState(visionState, true);

}

void ViconVisionStateEstimator::visionCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{

	//TODO: Filter

	Eigen::Vector3d velocityFromVision(msg->vector.x, msg->vector.y, msg->vector.z);
//	ROS_INFO_STREAM("Received vision input: " << velocityFromVision);

	TKState visionState;
	visionState.time = Time(msg->header.stamp);
	visionState.position = Eigen::Vector3d::Zero();
	visionState.linVelocity = velocityFromVision;
	visionState.orientation = Eigen::Quaterniond::Identity();
	visionState.angVelocity = Eigen::Vector3d::Zero();
	submitState(visionState, true);
}

void ViconVisionStateEstimator::joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	if(!useVision && msg->buttons[5] == 1){
		ROS_INFO_STREAM("Request to use visual state estimator received!");
		useVision = true;
	} else if(useVision && msg->buttons[5] == 0){
		ROS_INFO_STREAM("Fall-back to tracking system as requested!");
		useVision = false;
	}
}

} /* namespace TELEKYB_NAMESPACE */
