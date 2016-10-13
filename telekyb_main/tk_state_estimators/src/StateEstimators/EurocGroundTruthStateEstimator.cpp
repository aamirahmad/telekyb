/*
 * EurocGroundTruthStateEstimator.cpp
 *
 *  Created on: Dec 6, 2011
 *      Author: mriedel
 */
#include <StateEstimators/EurocGroundTruthStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKState.h>

#define DEBUG_WELL 0
#define DEBUG_NOT_WELL 0

PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::EurocGroundTruthStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace state_estimators_plugin {

EurocGroundTruthStateEstimatorOptions::EurocGroundTruthStateEstimatorOptions()
	: OptionContainer("EurocGroundTruthStateEstimator")
{
	tEurocGroundTruthSeTopicName = addOption<std::string>("tEurocGroundTruthSeTopicName","TopicName of EurocGroundTruth Sensor.","undef",true,true);
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tEurocGroundTruthToNEDMatrix = addOption<Eigen::Matrix3d>("tEurocGroundTruthToNEDMatrix","ConversionMatrix from EurocGroundTruth to NED", m, false, true);

	tEurocGroundTruthVelFilterFreq = addOption<double>("tEurocGroundTruthVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tEurocGroundTruthSmoothVelFilterFreq = addOption<double>("tEurocGroundTruthSmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tEurocGroundTruthAngFilterFreq = addOption<double>("tEurocGroundTruthAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tEurocGroundTruthSampleTime = addOption<double>("tEurocGroundTruthSampleTime",
			"Sampling Time of EurocGroundTruth System. Default is 120Hz", 0.008333333333, false, true);

    tEurocGroundTruthPublishSmoothVel = addOption<bool>("tEurocGroundTruthPublishSmoothVel",
            "Publish TKState with smoothed Velocity", false, false, true);
    tEurocGroundTruthUseFrame_id = addOption<bool>("tEurocGroundTruthUseFrame_id",
            "using a frame id", false, false, true);
    tEurocGroundTruthFrame_id = addOption<std::string>("tEurocGroundTruthFrame_id",
            "Specifiy the frame id", "/map", false, true);

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
}

void EurocGroundTruthStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocGroundTruthVelFilterFreq->getValue(),
				1.0,
				options.tEurocGroundTruthSampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocGroundTruthSmoothVelFilterFreq->getValue(),
				1.0,
				options.tEurocGroundTruthSampleTime->getValue());
	}


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tEurocGroundTruthAngFilterFreq->getValue(),
				1.0,
				options.tEurocGroundTruthSampleTime->getValue());
	}
}

// We should outsource this somehow!
void EurocGroundTruthStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
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

void EurocGroundTruthStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();
}
void EurocGroundTruthStateEstimator::willBecomeActive()
{
	pose1Sub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>(options.tEurocGroundTruthSeTopicName->getValue().c_str(), 1000, &EurocGroundTruthStateEstimator::poseCallback, this);
	
	if (options.tEurocGroundTruthPublishSmoothVel->getValue()) {
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
void EurocGroundTruthStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	pose1Sub.shutdown();
}

void EurocGroundTruthStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string EurocGroundTruthStateEstimator::getName() const
{
//	return options.tEurocGroundTruthSeTopicName->getValue();
	return "EurocGroundTruthStateEstimator";
}

void EurocGroundTruthStateEstimator::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	// StateEstimatorController neest a telekyb::TKState
	Position3D posVicon(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	Quaternion quatVicon(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

	Position3D posNED = options.tEurocGroundTruthToNEDMatrix->getValue() * posVicon;

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
	quatNED.vec() = options.tEurocGroundTruthToNEDMatrix->getValue() * quatNED.vec();
	//ROS_INFO_STREAM("Orientation RPY: " << quatVicon.toRotationMatrix().eulerAngles(0,1,2));
	//ROS_INFO_STREAM("OrientationNED RPY: " << quatNED.toRotationMatrix().eulerAngles(0,1,2));

	Velocity3D angVelocityNED = options.tEurocGroundTruthToNEDMatrix->getValue() * angVelocity;

	//ROS_INFO_STREAM("Ang Vel NED : " << angVelocityNED);

	// empty msg
	TKState tStateMsg;
	tStateMsg.time = Time(msg->header.stamp);

    // if the frame id should not be used it defaults to an empty string
    tStateMsg.frame_id = "";
    if(options.tEurocGroundTruthUseFrame_id->getValue())
        tStateMsg.frame_id = options.tEurocGroundTruthFrame_id->getValue();


	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelocityNED;


	stateEstimatorController.activeStateCallBack(tStateMsg);

	// Publish Smooth Velocity
	if (options.tEurocGroundTruthPublishSmoothVel->getValue()) {
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
