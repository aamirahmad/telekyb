/*
 * DVOStateEstimator.cpp
 *
 *  Created on: Aug 8, 2013
 *      Author: pstegagno
 */

#include <StateEstimators/DVOStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <telekyb_msgs/TKState.h>

PLUGINLIB_EXPORT_CLASS( telekyb_state::DVOStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

DVOStateEstimatorOptions::DVOStateEstimatorOptions()
	: OptionContainer("DVOStateEstimator")
{
	tViconTopicName = addOption<std::string>("tViconTopicName","TopicName of Vicon Sensor.","undef",true,true);
	tImuTopicName = addOption<std::string>("tImuTopicName","TopicName of Imu Sensor.","undef",true,true);
	tDvoTopicName = addOption<std::string>("tDvoTopicName","TopicName of Velocity (Dvo) Sensor.","undef",true,true);
	tDvoPoseTopicName = addOption<std::string>("tDvoPoseTopicName","TopicName of Velocity (Dvo pose) Sensor.","undef",true,true);
	
	
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tDVOToNEDMatrix = addOption<Eigen::Matrix3d>("tDVOToNEDMatrix","ConversionMatrix from DVO to NED", m, false, true);

	tDVOVelFilterFreq = addOption<double>("tDVOVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tDVOSmoothVelFilterFreq = addOption<double>("tDVOSmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tDVOAngFilterFreq = addOption<double>("tDVOAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tDVOSampleTime = addOption<double>("tDVOSampleTime",
			"Sampling Time of DVO System. Default is 120Hz", 0.008333333333, false, true);

	tDVOPublishSmoothVel = addOption<bool>("tDVOPublishSmoothVel",
			"Publish TKState with smoothed Velocity", false, false, true);
}

void DVOStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tDVOVelFilterFreq->getValue(),
				1.0,
				options.tDVOSampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tDVOSmoothVelFilterFreq->getValue(),
				1.0,
				options.tDVOSampleTime->getValue());
	}


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tDVOAngFilterFreq->getValue(),
				1.0,
				options.tDVOSampleTime->getValue());
	}
}

// We should outsource this somehow!
void DVOStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
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

void DVOStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
		std::cout << "0" << std::endl;

	initVelocityFilters();
			std::cout << "1" << std::endl;

	firstImu = true;
	firstDvo = true;
	firstDvoPose = true;
	astate = Eigen::Vector2d(0.0, 0.0);
	pstate = Eigen::Vector3d(0.0, 0.0, 0.0);
		std::cout << "2" << std::endl;
}


void DVOStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
			options.tViconTopicName->getValue(),1, &DVOStateEstimator::viconCallback, this);
	if (options.tDVOPublishSmoothVel->getValue()) {
		smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}
		std::cout << "a" << std::endl;
	
	imuSub = nodeHandle.subscribe<tk_draft_msgs::TKSmallImu>(
			options.tImuTopicName->getValue(),1, &DVOStateEstimator::imuCallback, this);
		std::cout << "b" << std::endl;
	
	dvoSub = nodeHandle.subscribe<geometry_msgs::TwistStamped>(
			options.tDvoTopicName->getValue(),1, &DVOStateEstimator::dvoCallback, this);
		std::cout << "c" << std::endl;
	
	dvoPoseSub = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
			options.tDvoPoseTopicName->getValue(),1, &DVOStateEstimator::dvoPoseCallback, this);
	
		std::cout << "d" << std::endl;
	kalmanVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("KalmanVelocity",1);
}


void DVOStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	viconSub.shutdown();
	imuSub.shutdown();
	dvoSub.shutdown();
	dvoPoseSub.shutdown();
}

void DVOStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string DVOStateEstimator::getName() const
{
//	return options.tDVOSeTopicName->getValue();
	return "DVOStateEstimator";
}

void DVOStateEstimator::viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// StateEstimatorController neest a telekyb::TKState
//	ROS_INFO("Received Callback!");
	Position3D posDVO(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	Quaternion quatDVO(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

	Position3D posNED = options.tDVOToNEDMatrix->getValue() * posDVO;

//	ROS_INFO_STREAM("posDVO: " << posDVO);
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

	input[0] = quatDVO.w();
	angFilter[0]->step(input, outputQuat[0]);
	input[0] = quatDVO.x();
	angFilter[1]->step(input, outputQuat[1]);
	input[0] = quatDVO.y();
	angFilter[2]->step(input, outputQuat[2]);
	input[0] = quatDVO.z();
	angFilter[3]->step(input, outputQuat[3]);

	// Constructor: w,x,y,z !
	Quaternion quatRates(outputQuat[0], outputQuat[1], outputQuat[2], outputQuat[3]);

	//Velocity3D vQuatVelDVO(outputVQuat);
	Velocity3D angVelocity;
	velQuatToBodyOmega(quatDVO, quatRates, angVelocity);

	Quaternion quatNED(quatDVO);
	quatNED.vec() = options.tDVOToNEDMatrix->getValue() * quatNED.vec();
	//ROS_INFO_STREAM("Orientation RPY: " << quatDVO.toRotationMatrix().eulerAngles(0,1,2));
	//ROS_INFO_STREAM("OrientationNED RPY: " << quatNED.toRotationMatrix().eulerAngles(0,1,2));

	Velocity3D angVelocityNED = options.tDVOToNEDMatrix->getValue() * angVelocity;

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
// 	if (options.tDVOPublishSmoothVel->getValue()) {
// 		telekyb_msgs::TKState smoothVelMsg;
// 		tStateMsg.toTKStateMsg(smoothVelMsg);
// 		smoothVelMsg.header.stamp = ros::Time::now();
// 		smoothVelMsg.twist.linear.x = smoothVelNED(0);
// 		smoothVelMsg.twist.linear.y = smoothVelNED(1);
// 		smoothVelMsg.twist.linear.z = smoothVelNED(2);
// 		smoothVelPub.publish(smoothVelMsg);
// 	}
}



void DVOStateEstimator::imuCallback(const tk_draft_msgs::TKSmallImu::ConstPtr& msg)
{
	bool savefile = false;
	if (firstImu){
		
		// rot matrix quad/camera
		Rcq <<	-0.2043,    0.7772,    0.5951,
					0.3355,    0.6267,   -0.7033,
					-0.9196,    0.0560,   -0.3888;
		// translation quad/camera
		pcq << 0.0261, -0.0106, 0.0206;
		
		RNEDNWU <<	1.0,  0.0,  0.0,
						0.0, -1.0,  0.0,
						0.0,  0.0, -1.0;
		
		Wqq <<	0.0, 0.0, 0.0,
						0.0, 0.0, 0.0,
						0.0, 0.0, 0.0;
		
		if (savefile){
			dataFile.open("/home/mbasile/Desktop/imu.txt");
		}
		
		imuAngVel[0] = msg->angular_velocity.x;
		imuAngVel[1] = msg->angular_velocity.y;
		imuAngVel[2] = msg->angular_velocity.z;
		
		lastimutime = msg->header.stamp;
		
		estRoll=0.0;
		estPitch=0.0;
		
		firstImu = false;
		
		smoothVelFilterDVOX = new OneEuroFilter(33, 1.0, 1.0, 1.0);
		smoothVelFilterDVOY = new OneEuroFilter(33, 1.0, 1.0, 1.0);
		smoothVelFilterDVOZ = new OneEuroFilter(33, 1.0, 1.0, 1.0);
		return;
	}
	
	double dt = (msg->header.stamp-lastimutime).toSec();
	lastimutime = msg->header.stamp;
	
	double innoFilter = 0.2;
	
	imuAngVel[0] = (1.0-innoFilter)*imuAngVel[0] + innoFilter*msg->angular_velocity.x;
	imuAngVel[1] = (1.0-innoFilter)*imuAngVel[1] + innoFilter*msg->angular_velocity.y;
	imuAngVel[2] = (1.0-innoFilter)*imuAngVel[2] + innoFilter*msg->angular_velocity.z;
	
		imuAcc[0]=msg->linear_acceleration.x;
		imuAcc[1]=msg->linear_acceleration.y;
		imuAcc[2]=msg->linear_acceleration.z;
		
		double estRollVel  = msg->angular_velocity.x; //don't use the drift value during the estimation
		double estPitchVel = msg->angular_velocity.y;
		
		double ACC_GAIN = 1.0;
		
		//------------- ROLL -------------//
		estRoll = estRoll + ( ACC_GAIN*((-asin(std::min(std::max(imuAcc[1]/9.81,-1.0),1.0))-estRoll) + estRollVel)*dt); //10000 parts of degrees
		
		//------------- PITCH -------------//
		estPitch = estPitch + ( ACC_GAIN*((+asin(std::min(std::max(imuAcc[0]/9.81,-1.0),1.0))-estPitch) + estPitchVel) * dt); //10000 parts of degrees
		
		
	if (savefile){
		dataFile << std::setprecision(30) << msg->header.stamp;
// 		dataFile << msg->angular_velocity.x << "  " << msg->angular_velocity.y << "  " << msg->angular_velocity.z ;
		dataFile << std::setprecision(10);
// 		dataFile << "  " << dt ;
// 		dataFile << "  " << msg->linear_acceleration.x << "  " << msg->linear_acceleration.y << "  " << msg->linear_acceleration.z;
// 		dataFile << "  " << imuAngVel[0] << "  " << imuAngVel[1] << "  " << imuAngVel[2];
// 		dataFile << "  " << rollRateLowPass << "  " << pitchRateLowPass << "  " << yawRateLowPass ;
// 		dataFile << "  " << estRollVel << "  " << estPitchVel << "  " << estYawVel;
// 		dataFile << "  " << estRoll << "  " << estPitch << "  " << estPitch;
		dataFile << "  " << imuAngVel[2];
		dataFile << std::endl;
	}
// 	std::cout << "  " << estRoll*180.0/M_PI << "  " << estPitch*180.0/M_PI ;
// 	std::cout << "  " << imuAcc[0] << "  " << imuAcc[1] << std::endl;
	
	// make the skew-symm matrix of the ang vel
	Wqq(0,1)= -imuAngVel[2]; Wqq(0,2)=imuAngVel[1];
	Wqq(1,0)=imuAngVel[2]; Wqq(1,2)=-imuAngVel[0];
	Wqq(2,0)=-imuAngVel[1]; Wqq(2,1)=imuAngVel[0];
	
	// compute the quadcopter velocity in the frame of the quadcopter
	vqq = Rcq * vcc - Wqq * pcq;
	
	Eigen::Matrix<double,3,3> Rroll;
	Rroll <<	1.0,          0.0,           0.0,
						0.0, cos(estRoll), -sin(estRoll),
						0.0, sin(estRoll),  cos(estRoll);
	
	Eigen::Matrix<double,3,3> Rpitch;
	Rpitch <<  cos(estPitch), 0.0, sin(estPitch),
						           0.0, 1.0,           0.0,
						-sin(estPitch), 0.0, cos(estPitch);
						
	vqh = Rpitch*Rroll*RNEDNWU*vqq;
	
	double cEstRoll_2 = cos(estRoll/2.0), sEstRoll_2 = sin(estRoll/2.0);
	double cEstPitc_2 = cos(estPitch/2.0), sEstPitc_2 = sin(estPitch/2.0);
	double w =  cEstRoll_2 * cEstPitc_2;
	double x =  sEstRoll_2 * cEstPitc_2;
	double y =  cEstRoll_2 * sEstPitc_2;
	double z = (-sEstRoll_2 * sEstPitc_2);

	msg_state.header = msg->header;
	
// 	msg_state.pose.position.x = estRoll*180.0/M_PI;
// 	msg_state.pose.position.y = estPitch*180.0/M_PI;
	msg_state.pose.position.x = 0.0;
	msg_state.pose.position.y = 0.0;
	msg_state.pose.position.z = 0.0;
	
	msg_state.pose.orientation.w = w;
	msg_state.pose.orientation.x = x;
	msg_state.pose.orientation.y = y;
	msg_state.pose.orientation.z = z;
	
	msg_state.twist.linear.x = vqh[0];
	msg_state.twist.linear.y = vqh[1];
	msg_state.twist.linear.z = vqh[2];
	
	msg_state.twist.angular.x = imuAngVel[0];
	msg_state.twist.angular.y = imuAngVel[1];
	msg_state.twist.angular.z = imuAngVel[2];
	
	kalmanVelPub.publish(msg_state);
	
	
		// Publish Smooth Velocity
	if (options.tDVOPublishSmoothVel->getValue()) {
	  
	  
		double xxx = smoothVelFilterDVOX->filter(vqh[0], msg->header.stamp.toSec());
		double yyy = smoothVelFilterDVOY->filter(vqh[1], msg->header.stamp.toSec());
		double zzz = smoothVelFilterDVOZ->filter(vqh[2], msg->header.stamp.toSec());
	  
	  
		telekyb_msgs::TKState smoothVelMsg;
		smoothVelMsg.header.stamp = ros::Time::now();
		smoothVelMsg.twist.linear.x = -xxx*cos(M_PI/4)-yyy*sin(M_PI/4);
		smoothVelMsg.twist.linear.y =  xxx*sin(M_PI/4)-yyy*cos(M_PI/4);
		smoothVelMsg.twist.linear.z = zzz;
		
// 		smoothVelMsg.twist.linear.x /= 2.0;
// 		smoothVelMsg.twist.linear.y /= 2.0;
// 		smoothVelMsg.twist.linear.z /= 2.0;
		
		smoothVelPub.publish(smoothVelMsg);
	}
	
}





void DVOStateEstimator::dvoCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	
// 	if (firstDvo){
// 		
// 		lastdvotime = msg->header.stamp;
// 		firstDvo = false;
// 		
// 		pstate[0] = 0.0;
// 		pstate[1] = 0.0;
// 		pstate[2] = 0.0;
// 		
// 		astate[0] = 0.0;
// 		astate[1] = 0.0;
// 		
// 		msg_state.angular_velocity.x = 0;
// 		msg_state.angular_velocity.y = 0;
// 		msg_state.angular_velocity.z = 0;
// 		
// 		msg_state.linear_acceleration.x = 0;
// 		msg_state.linear_acceleration.y = 0;
// 		msg_state.linear_acceleration.z = 0;
// 		msg_state.header = msg->header;
// 		
// 		return;
// 	}
// 	
// 	ros::Time curT(msg->header.stamp);
// 	double deT=(curT-lastdvotime).toSec();
// 	
// // 	std::cout << "    " << curT << " " << lastdvotime << " " << deT << std::endl;
// 	lastdvotime = curT;
// 	
// 	bool debug = false;
// 	if(debug) std::cout << "----------------------------------" << std::endl;
// 	if(debug) std::cout << stateCov << std::endl;
// 
// 	Eigen::Vector3d vcc(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
// 	vcc = vcc/deT;
// 	
// 	Eigen::Matrix<double,3,3> Wqq; // skew-symm matrix of the ang vel
// 	Wqq <<	 0.0,					  -imuAngVel[2],		 imuAngVel[1],
// 				 imuAngVel[2],  0.0,						  -imuAngVel[0],
// 				-imuAngVel[1],  imuAngVel[0],		 0.0;
// 	
// 	// rot matrix quad/camera
// 	Eigen::Matrix<double,3,3> Rcq;
// 	Rcq <<	0.2185,		-0.6172,	0.7558,
// 			0.2185,		0.7858,		0.5786,
// 			-0.9511,	0.0387,		0.3066;
// 	
// 	// translation quad/camera
// 	Eigen::Vector3d pcq(0.07,-0.07,-0.02);
	
// 	Eigen::Vector3d vqq;
// 	vqq = Rcq * vcc - Wqq * pcq;
	
	
// 	msg_state.linear_acceleration.x = vqq[0];
// 	msg_state.linear_acceleration.y = vqq[1];
// 	msg_state.linear_acceleration.z = vqq[2];
// 	
// 	msg_state.angular_velocity.x = imuAngVel[0];
// 	msg_state.angular_velocity.y = imuAngVel[1];
// 	msg_state.angular_velocity.z = imuAngVel[2];
// 	
// 	kalmanVelPub.publish(msg_state);
	
	
}









void DVOStateEstimator::dvoPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
// 	ros::Time nowTime = ros::Time::now();
	
	if (firstDvoPose){
		
		lastdvoPosetime = msg->header.stamp;
		firstDvoPose = false;
		
		pstate[0] = 0.0;
		pstate[1] = 0.0;
		pstate[2] = 0.0;
		
		astate[0] = 0.0;
		astate[1] = 0.0;
		
		float filtGain = 0.5;
		
		
		
		
		xFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		yFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		zFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		
		lastdvoPos[0] = xFilt->filter(msg->pose.pose.position.x, msg->header.stamp.toSec());
		lastdvoPos[1] = yFilt->filter(msg->pose.pose.position.y, msg->header.stamp.toSec());
		lastdvoPos[2] = zFilt->filter(msg->pose.pose.position.z, msg->header.stamp.toSec());
		
		oxFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		oyFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		ozFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		owFilt = new OneEuroFilter(33, filtGain, 1.0, 1.0);
		
		double eta  = owFilt->filter(msg->pose.pose.orientation.w, msg->header.stamp.toSec());
		double epsx = oxFilt->filter(msg->pose.pose.orientation.x, msg->header.stamp.toSec());
		double epsy = oyFilt->filter(msg->pose.pose.orientation.y, msg->header.stamp.toSec());
		double epsz = ozFilt->filter(msg->pose.pose.orientation.z, msg->header.stamp.toSec());
// 		double eta=msg->pose.pose.orientation.w;
// 		double epsx=msg->pose.pose.orientation.x;
// 		double epsy=msg->pose.pose.orientation.y;
// 		double epsz=msg->pose.pose.orientation.z;
		
		lastdvoPoseOri(0,0) = 2.0*(eta*eta+epsx*epsx)-1.0;
		lastdvoPoseOri(0,1) = 2.0*(epsx*epsy-eta*epsz);
		lastdvoPoseOri(0,2) = 2.0*(epsx*epsz+eta*epsy);
		lastdvoPoseOri(1,0) = 2.0*(epsx*epsy+eta*epsz);
		lastdvoPoseOri(1,1) = 2.0*(eta*eta+epsy*epsy)-1.0;
		lastdvoPoseOri(1,2) = 2.0*(epsy*epsz-eta*epsx);
		lastdvoPoseOri(2,0) = 2.0*(epsx*epsz-eta*epsy);
		lastdvoPoseOri(2,1) = 2.0*(epsy*epsz+eta*epsx);
		lastdvoPoseOri(2,2) = 2.0*(eta*eta+epsz*epsz)-1.0;
		
		msg_state.header = msg->header;
		
		return;
	}
	
	// here compute time diff between a DVO estimate and the previous
	ros::Time curT(msg->header.stamp);
	double deT=(curT-lastdvoPosetime).toSec();
	lastdvoPosetime = curT;
	
	
	// ### here compute the displacement between a DVO estimate and the previous ###
	
	// take the position
	Eigen::Vector3d newdvoPose;
// 	newdvoPose[0] = msg->pose.pose.position.x;
// 	newdvoPose[1] = msg->pose.pose.position.y;
// 	newdvoPose[2] = msg->pose.pose.position.z;
	newdvoPose[0] = xFilt->filter(msg->pose.pose.position.x, msg->header.stamp.toSec());
	newdvoPose[1] = yFilt->filter(msg->pose.pose.position.y, msg->header.stamp.toSec());
	newdvoPose[2] = zFilt->filter(msg->pose.pose.position.z, msg->header.stamp.toSec());
	
	// transform from quaternion to rotation matrix
	Eigen::Matrix3d newdvoPoseOri;
	double eta  = owFilt->filter(msg->pose.pose.orientation.w, msg->header.stamp.toSec());
	double epsx = oxFilt->filter(msg->pose.pose.orientation.x, msg->header.stamp.toSec());
	double epsy = oyFilt->filter(msg->pose.pose.orientation.y, msg->header.stamp.toSec());
	double epsz = ozFilt->filter(msg->pose.pose.orientation.z, msg->header.stamp.toSec());
// 	double eta=msg->pose.pose.orientation.w;
// 	double epsx=msg->pose.pose.orientation.x;
// 	double epsy=msg->pose.pose.orientation.y;
// 	double epsz=msg->pose.pose.orientation.z;
	newdvoPoseOri(0,0) = 2.0*(eta*eta+epsx*epsx)-1.0; newdvoPoseOri(0,1) = 2.0*(epsx*epsy-eta*epsz); newdvoPoseOri(0,2) = 2.0*(epsx*epsz+eta*epsy);
	newdvoPoseOri(1,0) = 2.0*(epsx*epsy+eta*epsz); newdvoPoseOri(1,1) = 2.0*(eta*eta+epsy*epsy)-1.0; newdvoPoseOri(1,2) = 2.0*(epsy*epsz-eta*epsx);
	newdvoPoseOri(2,0) = 2.0*(epsx*epsz-eta*epsy); newdvoPoseOri(2,1) = 2.0*(epsy*epsz+eta*epsx); newdvoPoseOri(2,2) = 2.0*(eta*eta+epsz*epsz)-1.0;
	
	// compute the relative displacements
	Eigen::Matrix3d rot_CB = lastdvoPoseOri.transpose()*newdvoPoseOri;
	Eigen::Vector3d p_CB = lastdvoPoseOri.transpose()*(newdvoPose - lastdvoPos);
	
	// update the last DVO pose with the new one for the next step
	lastdvoPoseOri = newdvoPoseOri;
	lastdvoPos = newdvoPose;
	
	// here use displacement to compute the velocity
	vcb[0] = p_CB[0]/deT;
	vcb[1] = p_CB[1]/deT;
	vcb[2] = p_CB[2]/deT;
	
	vcc = rot_CB.transpose()*vcb;
	
// 	ros::Time nowTime2 = ros::Time::now();
	
// 	std::cout << nowTime2-nowTime << std::endl;
	
	
// 	// make the skew-symm matrix of the ang vel
// 	Eigen::Matrix<double,3,3> Wqq;
// 	Wqq <<	 0.0,					  -imuAngVel[2],		 imuAngVel[1],
// 				 imuAngVel[2],  0.0,						  -imuAngVel[0],
// 				-imuAngVel[1],  imuAngVel[0],		 0.0;
// 	
// 	// rot matrix quad/camera
// 	Eigen::Matrix<double,3,3> Rcq;
// 	Rcq <<	-0.2043,    0.7772,    0.5951,
//     0.3355,    0.6267,   -0.7033,
//    -0.9196,    0.0560,   -0.3888;
// 	
// 	// translation quad/camera
// 	Eigen::Vector3d pcq(0.0261,-0.0106,0.0206);
// 	
// 	// compute the quadcopter velocity in the frame of the quadcopter
// 	vqq = Rcq * vcc - Wqq * pcq;
	
// 	kalmanVelPub.publish(msg_state);
	
	
}






} /* namespace telekyb_state */
