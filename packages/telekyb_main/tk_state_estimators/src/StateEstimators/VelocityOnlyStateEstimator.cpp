/*
 * VelocityOnlyStateEstimator.cpp
 *
 *  Created on: Aug 8, 2013
 *      Author: pstegagno
 */

#include <StateEstimators/VelocityOnlyStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <telekyb_msgs/TKState.h>

PLUGINLIB_EXPORT_CLASS( telekyb_state::VelocityOnlyStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

VelocityOnlyStateEstimatorOptions::VelocityOnlyStateEstimatorOptions()
	: OptionContainer("VelocityOnlyStateEstimator")
{
	tViconTopicName = addOption<std::string>("tViconTopicName","TopicName of Vicon Sensor.","undef",true,true);
	tImuTopicName = addOption<std::string>("tImuTopicName","TopicName of Imu Sensor.","undef",true,true);
	tDvoTopicName = addOption<std::string>("tDvoTopicName","TopicName of Velocity (Dvo) Sensor.","undef",true,true);
	
	
	Eigen::Matrix3d m = Eigen::Matrix3d::Zero();
	m.diagonal() << 1.0,-1.0,-1.0;
	tVelocityOnlyToNEDMatrix = addOption<Eigen::Matrix3d>("tVelocityOnlyToNEDMatrix","ConversionMatrix from VelocityOnly to NED", m, false, true);

	tVelocityOnlyVelFilterFreq = addOption<double>("tVelocityOnlyVelFilterFreq",
			"Frequency of Velocity Filter (Initial)", 40.0, false, true);
	tVelocityOnlySmoothVelFilterFreq = addOption<double>("tVelocityOnlySmoothVelFilterFreq",
			"Frequency of Smooth Velocity Filter (Initial)", 10.0, false, true);
	tVelocityOnlyAngFilterFreq = addOption<double>("tVelocityOnlyAngFilterFreq",
			"Frequency of Angular Velocity Filter (Initial)", 30.0, false, true);

	tVelocityOnlySampleTime = addOption<double>("tVelocityOnlySampleTime",
			"Sampling Time of VelocityOnly System. Default is 120Hz", 0.008333333333, false, true);

	tVelocityOnlyPublishSmoothVel = addOption<bool>("tVelocityOnlyPublishSmoothVel",
			"Publish TKState with smoothed Velocity", false, false, true);
}

void VelocityOnlyStateEstimator::initVelocityFilters()
{
	IIRFiltDeriv isDerivative;
	for(int i=0;i<3;i++){
		velFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tVelocityOnlyVelFilterFreq->getValue(),
				1.0,
				options.tVelocityOnlySampleTime->getValue());
	}

	for(int i=0;i<3;i++){
		smoothVelFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tVelocityOnlySmoothVelFilterFreq->getValue(),
				1.0,
				options.tVelocityOnlySampleTime->getValue());
	}


	for(int i=0;i<4;i++){
		angFilter[i] = new IIRFilter(
				isDerivative,
				2.0*M_PI*options.tVelocityOnlyAngFilterFreq->getValue(),
				1.0,
				options.tVelocityOnlySampleTime->getValue());
	}
}

// We should outsource this somehow!
void VelocityOnlyStateEstimator::velQuatToBodyOmega(const Quaternion& quat, const Quaternion& quatRates, Velocity3D& bodyOmega)
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

void VelocityOnlyStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();

	initVelocityFilters();
	
	firstExecution = true;
	astate = Eigen::Vector2d(0.0, 0.0);
	pstate = Eigen::Vector3d(0.0, 0.0, 0.0);
}


void VelocityOnlyStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
			options.tViconTopicName->getValue(),1, &VelocityOnlyStateEstimator::viconCallback, this);
	if (options.tVelocityOnlyPublishSmoothVel->getValue()) {
		smoothVelPub = nodeHandle.advertise<telekyb_msgs::TKState>("SmoothedVelocity",1);
	}
	
	imuSub = nodeHandle.subscribe<tk_draft_msgs::TKSmallImu>(
			options.tImuTopicName->getValue(),1, &VelocityOnlyStateEstimator::imuCallback, this);
	
	dvoSub = nodeHandle.subscribe<geometry_msgs::TwistStamped>(
			options.tDvoTopicName->getValue(),1, &VelocityOnlyStateEstimator::dvoCallback, this);
	
	kalmanVelPub = nodeHandle.advertise<tk_draft_msgs::TKSmallImu>("KalmanVelocity",1);
}


void VelocityOnlyStateEstimator::willBecomeInActive()
{
	smoothVelPub.shutdown();
	viconSub.shutdown();
	imuSub.shutdown();
	dvoSub.shutdown();
}

void VelocityOnlyStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete velFilter[i];
		delete smoothVelFilter[i];
	}

	for(int i=0;i<4;i++){
		delete angFilter[i];
	}
}

std::string VelocityOnlyStateEstimator::getName() const
{
//	return options.tVelocityOnlySeTopicName->getValue();
	return "VelocityOnlyStateEstimator";
}

void VelocityOnlyStateEstimator::viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	// StateEstimatorController neest a telekyb::TKState
//	ROS_INFO("Received Callback!");
	Position3D posVelocityOnly(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
	Quaternion quatVelocityOnly(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

	Position3D posNED = options.tVelocityOnlyToNEDMatrix->getValue() * posVelocityOnly;

//	ROS_INFO_STREAM("posVelocityOnly: " << posVelocityOnly);
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

	input[0] = quatVelocityOnly.w();
	angFilter[0]->step(input, outputQuat[0]);
	input[0] = quatVelocityOnly.x();
	angFilter[1]->step(input, outputQuat[1]);
	input[0] = quatVelocityOnly.y();
	angFilter[2]->step(input, outputQuat[2]);
	input[0] = quatVelocityOnly.z();
	angFilter[3]->step(input, outputQuat[3]);

	// Constructor: w,x,y,z !
	Quaternion quatRates(outputQuat[0], outputQuat[1], outputQuat[2], outputQuat[3]);

	//Velocity3D vQuatVelVelocityOnly(outputVQuat);
	Velocity3D angVelocity;
	velQuatToBodyOmega(quatVelocityOnly, quatRates, angVelocity);

	Quaternion quatNED(quatVelocityOnly);
	quatNED.vec() = options.tVelocityOnlyToNEDMatrix->getValue() * quatNED.vec();
	//ROS_INFO_STREAM("Orientation RPY: " << quatVelocityOnly.toRotationMatrix().eulerAngles(0,1,2));
	//ROS_INFO_STREAM("OrientationNED RPY: " << quatNED.toRotationMatrix().eulerAngles(0,1,2));

	Velocity3D angVelocityNED = options.tVelocityOnlyToNEDMatrix->getValue() * angVelocity;

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
	if (options.tVelocityOnlyPublishSmoothVel->getValue()) {
		telekyb_msgs::TKState smoothVelMsg;
		tStateMsg.toTKStateMsg(smoothVelMsg);
		smoothVelMsg.header.stamp = ros::Time::now();
		smoothVelMsg.twist.linear.x = smoothVelNED(0);
		smoothVelMsg.twist.linear.y = smoothVelNED(1);
		smoothVelMsg.twist.linear.z = smoothVelNED(2);
		smoothVelPub.publish(smoothVelMsg);
	}
}



void VelocityOnlyStateEstimator::imuCallback(const tk_draft_msgs::TKSmallImu::ConstPtr& msg)
{
	Time prevImuTime = lastImuTime;
	if (firstExecution){
		imucounter=1.0;
		
		driftAngVel[0] = msg->angular_velocity.x;
		driftAngVel[1] = msg->angular_velocity.y;
		driftAngVel[2] = msg->angular_velocity.z;
		
		driftLinAcc[0] = msg->linear_acceleration.x;
		driftLinAcc[1] = msg->linear_acceleration.y;
		driftLinAcc[2] = msg->linear_acceleration.z;
		
		prevImuTime = Time(msg->header.stamp);
		firstExecution = false;
		
		pstate[0] = 0.0;
		pstate[1] = 0.0;
		pstate[2] = 0.0;
		
		astate[0] = 0.0;
		astate[1] = 0.0;
		
		stateCov << 1.1, 0.0, 0.0, 0.0, 0.0,
								0.0, 1.1, 0.0, 0.0, 0.0,
								0.0, 0.0, 1.1, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.001, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.001;
		
		lastAngVel[0] = msg->angular_velocity.x;
		lastAngVel[1] = msg->angular_velocity.y;
		lastAngVel[2] = msg->angular_velocity.z;
		
		lastLinAcc[0] = msg->linear_acceleration.x;
		lastLinAcc[1] = msg->linear_acceleration.y;
		lastLinAcc[2] = msg->linear_acceleration.z;
		
		msg_state.angular_velocity.x = 0;
		msg_state.angular_velocity.y = 0;
		msg_state.angular_velocity.z = 0;
		
		msg_state.linear_acceleration.x = 0;
		msg_state.linear_acceleration.y = 0;
		msg_state.linear_acceleration.z = 0;
		msg_state.header = msg->header;
		
		
		return;
	}
	
	
		
// 	std::cout << "----------------------------------" << std::endl;
// 	std::cout << stateCov << std::endl;
	
	
	imucounter++;
	
	driftAngVel[0] = driftAngVel[0]*(imucounter-1.0)/imucounter + msg->angular_velocity.x/imucounter;
	driftAngVel[1] = driftAngVel[1]*(imucounter-1.0)/imucounter + msg->angular_velocity.y/imucounter;
	driftAngVel[2] = driftAngVel[2]*(imucounter-1.0)/imucounter + msg->angular_velocity.z/imucounter;
	
	driftLinAcc[0] = driftLinAcc[0]*(imucounter-1.0)/imucounter + msg->linear_acceleration.x/imucounter;
	driftLinAcc[1] = driftLinAcc[1]*(imucounter-1.0)/imucounter + msg->linear_acceleration.y/imucounter;
	driftLinAcc[2] = driftLinAcc[2]*(imucounter-1.0)/imucounter + msg->linear_acceleration.z/imucounter;
	
// 	std::cout << driftAngVel << std::endl;
// 	std::cout << driftLinAcc << std::endl;
	
	lastImuTime = Time(msg->header.stamp);
	double dT = (lastImuTime-prevImuTime).usec()/1000000.0;
	
// 	std::cout << dT << " " << pstate.size() << " " << astate.size() << " " << std::endl;
	
	Eigen::Vector2d newastate;
	Eigen::Vector3d newpstate;
	
	Eigen::Vector3d angVel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
	Eigen::Vector3d linAcc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	
	Eigen::Vector3d avm = (angVel+lastAngVel)/2.0;// mean of the ang vel in dT
	Eigen::Vector3d lam = (linAcc+lastLinAcc)/2.0;// mean of the lin acc in dT
	
	newastate[0] = astate[0] + dT*( avm[0] + tan(astate[1])*( sin(astate[0])*avm[1] + cos(astate[0])*avm[2] ) );
	newastate[1] = astate[1] + dT*( cos(astate[0])*avm[1] - sin(astate[0])*avm[2] );
	
	// 9.7803184*(1+ 0.0053024 sin(0.84648468721) - 0.0000059*sin(2*0.84648468721) ) -3.086*10^(-6)*500
// 	double g = 9.81755834101;
	double g = 7.98152;
	
	Eigen::Vector2d astm = (astate + newastate)/2.0;// mean of the astate in dT
	
	newpstate[0] = pstate[0] + dT*(lam[0] /*+0.00320829*/ + cos(astm[0])*sin(astm[1])*g );
	newpstate[1] = pstate[1] + dT*(lam[1] /*+0.04773870*/ +              sin(astm[1])*g );
	newpstate[2] = pstate[2] + dT*(lam[2]                 + cos(astm[0])*cos(astm[1])*g );
	
	
	astate = newastate;
// 	astate[0] = 0.0;
	pstate = newpstate;
	
	lastAngVel = angVel;
	lastLinAcc = linAcc;
	
	msg_state.header = msg->header;
	
	msg_state.angular_velocity.x = astate[0]*180.0/M_PI;
	msg_state.angular_velocity.y = astate[1]*180.0/M_PI;
	msg_state.angular_velocity.z = 0;
	
	msg_state.linear_acceleration.x = pstate[0];
	msg_state.linear_acceleration.y = pstate[1];
	msg_state.linear_acceleration.z = pstate[2];
	
	
	double F03, F04, F13, F23, F24, F33, F34, F43;
	F03 = -dT*sin(astate[0])*sin(astate[1])*g;
	F13 =  dT*cos(astate[0])*g;
	F23 = -dT*sin(astate[0])*cos(astate[1])*g;
	F33 =  1.0+dT * tan(astate[1])                      * ( cos(astate[0])*avm[1] - sin(astate[0])*avm[2]);
	F43 =     -dT *                                       ( sin(astate[0])*avm[1] + cos(astate[0])*avm[2]);
	
	F04 =  dT*cos(astate[0])*cos(astate[1])*g;
	F24 = -dT*cos(astate[0])*sin(astate[1])*g;
	F34 =      dT * (1.0+tan(astate[1])*tan(astate[1])) * ( sin(astate[0])*avm[1] + cos(astate[0])*avm[2]);
	
	Eigen::Matrix<double, 5, 5> F;
	F <<	1.0, 0.0, 0.0, F03, F04,
				0.0, 1.0, 0.0, F13, 0.0,
				0.0, 0.0, 1.0, F23, F24,
				0.0, 0.0, 0.0, F33, F34,
				0.0, 0.0, 0.0, F43, 1.0;
	
	double G34, G35, G44, G45;
	G34 =  dT*sin(astate[0])*tan(astate[1]);
	G35 =  dT*cos(astate[0])*tan(astate[1]);
	G44 =  dT*cos(astate[0]);
	G45 = -dT*sin(astate[0]);
	
	Eigen::Matrix<double, 5, 6> G;
	G <<	dT , 0.0, 0.0, 0.0, 0.0, 0.0,
				0.0, dT , 0.0, 0.0, 0.0, 0.0,
				0.0, 0.0, dT , 0.0, 0.0, 0.0,
				0.0, 0.0, 0.0, dT , G34, G35,
				0.0, 0.0, 0.0, 0.0, G44, G45;
				
	Eigen::Matrix<double, 6, 6> inCov;
	inCov <<	0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
						0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
						0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
						0.0, 0.0, 0.0, 2.01, 0.0, 0.0,
						0.0, 0.0, 0.0, 0.0, 2.01, 0.0,
						0.0, 0.0, 0.0, 0.0, 0.0, 2.01;
	
	
// 	std::cout << "1111111111111111111111111" << std::endl;
// 	std::cout << stateCov << std::endl;

	stateCov = F*stateCov*F.transpose();
// 	std::cout << "2222222222222222222222222" << std::endl;
// 	std::cout << stateCov << std::endl;
	
	stateCov = stateCov + G*inCov*G.transpose();
	
	std::cout << stateCov << std::endl;
	
// 	stateCov = F*stateCov*F.transpose() + G*inCov*G.transpose();
	
	kalmanVelPub.publish(msg_state);
	
}

void VelocityOnlyStateEstimator::dvoCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
// 	return;
	
	if (firstDvo){
		lastdvotime = msg->header.stamp;
		firstDvo = false;
		return;
	}
	
	ros::Time curT(msg->header.stamp);
	double deT=(curT-lastdvotime).toSec();
	
// 	std::cout << "    " << curT << " " << lastdvotime << " " << deT << std::endl;
	lastdvotime = curT;
	
	bool debug = false;
	if(debug) std::cout << "----------------------------------" << std::endl;
	if(debug) std::cout << stateCov << std::endl;

	Eigen::Vector3d z(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);
	z = z/deT;
	
// 	std::cout << deT ;
// 	std::cout << " " << z[0] << " " << z[1] << " " << z[2] << " " << std::endl;
	
	Eigen::Matrix<double,3,3> Z; // noise cov on the measurement
	Z <<	0.02, 0.0, 0.0,
				0.0, 0.02, 0.0,
				0.0, 0.0, 0.02;
	
	
	Eigen::Matrix<double,3,3> W; // skew-symm matrix of the ang vel
	W <<	 0.0,					  -lastAngVel[2],		 lastAngVel[1],
				 lastAngVel[2],  0.0,						  -lastAngVel[0],
				-lastAngVel[1],  lastAngVel[0],		 0.0;
	
	Eigen::Matrix<double,3,3> R; // rot matrix quad/camera
	R <<	 0.2185,   -0.6172,    0.7558,
    0.2185,    0.7858,    0.5786,
   -0.9511,    0.0387,    0.3066;
	
	Eigen::Matrix<double,3,5> H;
	H <<	R(0,0), R(0,1), R(0,2), 0.0, 0.0,
				R(1,0), R(1,1), R(1,2), 0.0, 0.0,
				R(2,0), R(2,1), R(2,2), 0.0, 0.0;
				
	Eigen::Vector3d p(1.07,-0.07,-0.02); // translation quad/camera
	
	Eigen::Vector3d zp; // predicted Measurement
	
	zp = R*pstate + R*W*p;
	if(debug) std::cout << "zp:" << std::endl;
	if(debug) std::cout << zp[0] << " " << zp[1] << " " << zp[2] << std::endl;
	
	Eigen::Vector3d inno(z-zp); // innovation
	
	Eigen::Matrix<double,3,3> S; // cov of innovation
	S = H*stateCov*H.transpose() + Z;
	
	Eigen::Matrix<double,5,3> K; // k gain
	K = stateCov*(H.transpose())*(S.inverse());
	
	Eigen::Matrix<double,5,1> state;
	state << pstate[0], pstate[1], pstate[2], astate[0], astate[1];
	Eigen::Matrix<double,5,1> newState;
	
	if(debug) std::cout << "state:" << std::endl;
	if(debug) std::cout << state[0] << " " << state[1] << " " << state[2] << " " << state[3] << " " << state[4] << std::endl;
	
	newState = state + K*inno;
	
	if(debug) std::cout << "K:" << std::endl;
	if(debug) std::cout << K << std::endl;
	
	if(debug) std::cout << "inno:" << std::endl;
	if(debug) std::cout << inno[0] << " " << inno[1] << " " << inno[2] << std::endl;
	
	newState = state + K*inno;
	
	pstate << newState[0], newState[1],  newState[2];
	astate << newState[3], newState[4];
	if(debug) std::cout << pstate[0] << " " << pstate[1] << " " << pstate[2] << " " << astate[0] << " " << astate[1] << std::endl;
	
	Eigen::Matrix<double,5,5> I; // identity matrix 5x5
	I <<	1.0, 0.0, 0.0, 0.0, 0.0,
				0.0, 1.0, 0.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 0.0, 1.0;
	stateCov = (I-K*H)*stateCov;
	
	
	
	
	
	
	
	// false misure di roll e pitch
	
	
	
// 	Eigen::Matrix<double,2,5> H1;
// 	H1 <<	0.0, 0.0, 0.0, 1.0, 0.0,
// 				0.0, 0.0, 0.0, 0.0, 1.0;
// 				
// 	/*Eigen::Vector2d z1(0.0, 0.0);
// 	Eigen::Vector2d zp1(astate[0], astate[0]); // predicted Measurement
// 	Eigen::Vector2d inno1(z1-zp1);
// 	
// 	Eigen::Matrix<double,2,2> Z1;
// 	Z1 << 0.01, 0.0,
// 				0.0, 0.01;
// 	
// 	Eigen::Matrix<double,2,2> S1;
// 	S1 = H1*stateCov*H1.transpose() + Z1;
// 	
// 	Eigen::Matrix<double,5,2> K1;
// 	K1 = stateCov*H1.transpose()*S1.inverse();
// 	
// 	newState = newState + K1*inno1;
// 	stateCov = (I-K1*H1)*stateCov;
// 	pstate << newState[0], newState[1],  newState[2];
// 	astate << newState[3], newState[4];
// 	
// 	std::cout << "state1:" << std::endl;
// 	std::cout << newState[0] << " " << newState[1] << " " << newState[2] << " " << newState[3] << " " << newState[4] << std::endl;
// 	std::cout << "K1:" << std::endl;
// 	std::cout << K1 << std::endl;
// 	std::cout << "inno1:" << std::endl;
// 	std::cout << inno1[0] << " " << inno1[1] << " " << std::endl;*/
	
	
}


} /* namespace telekyb_state */
