/*
 * FlowImuStateEstimator.cpp
 *
 *  Created on: Jan 03, 2016
 *      Author: modelga
 * 
 *  Initial version: FlowImuKalmanFilter in tk_modelga
 * 
 */

#include <StateEstimators/FlowImuStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::FlowImuStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace state_estimators_plugin {
  
FlowImuEstimatorOptions::FlowImuEstimatorOptions()
	: OptionContainer("FlowImuStateEstimator")
{
	tImuScale = addOption<Eigen::Vector3d>("tImuScale","Accelerometer calibration scale factor.",Eigen::Vector3d(0.9801,0.9810,1.1983),false,true);
	tImuOffset = addOption<Eigen::Vector3d>("tImuOffset","Accelerometer calibration offset value.",Eigen::Vector3d(-0.1932,-0.2038,-0.0673),false,true);
	tGyroScale = addOption<Eigen::Vector3d>("tGyroScale","Gyro calibration scale factor.",Eigen::Vector3d(1.3508,1.3532,1.3682),false,true);
	tInputNoiseCov = addOption<Eigen::Vector3d>("tInputNoiseCov","Covariance of the model (imu) noise.",Eigen::Vector3d(1.0,1.0,10.0),false, true);
	tMeasurementNoiseCov = addOption<Eigen::Vector3d>("tMeasurementNoiseCov","Covariance of the measurement (flow and distance) noise.",Eigen::Vector3d(0.025,0.025,0.01),false, true);
	
	tSmurfInterfaceTopicName = addOption<std::string>("tSmurfInterfaceTopicName","TopicName of Smurf Interface.","",true,true);
	tFlowTopicName = addOption<std::string>("tFlowTopicName","TopicName of the flow sensor.","/px4flow/opt_flow",false,true);
	tBehaviorTopicName = addOption<std::string>("tBehaviorTopicName","TopicName of Active Behavior Listener.","",true,true);
	
	tSaveFiles = addOption<bool>("tSaveFiles", "Do you want to save txt files with filter output?",bool(false),false,true);
	tUniqueNames = addOption<bool>("tUniqueNames", "Do you want to add an unique id to the text file names?", bool(true), false, true);
	tFilesLocation = addOption<std::string>("tFilesLocation", "Direcory in which the files should be saved.", "/home/odroid/",false,true);
}  

void FlowImuStateEstimator::initVariables()
{
	q_ = Eigen::VectorXd::Zero(5);
	u_ << 0.0, 0.0, 0.0;
	z1_ << 0.0, 0.0;
	A_ = Eigen::MatrixXd::Identity(5,5);
	B_ = Eigen::MatrixXd::Zero(5,3);
	C1_ = Eigen::MatrixXd::Zero(2,5);
	C1_(0,0) = 1.0;
	C1_(1,1) = 1.0;
	c2_ = Eigen::VectorXd::Zero(5); // [0 0 0 1 -1]
	c2_(3) = 1.0;
	c2_(4) = -1.0;

	P_ = 0.001 * Eigen::MatrixXd::Ones(5,5);
	
	Qu_(0,0) = options.tInputNoiseCov->getValue()(0);
	Qu_(1,1) = options.tInputNoiseCov->getValue()(1);
	Qu_(2,2) = options.tInputNoiseCov->getValue()(2);
	
	Qq_ = Eigen::MatrixXd::Zero(5,5);
	//Qq_ = B_* Qu_ * B_.transpose();
	//Qq_(3,3) = 0.1; //process cov. of z
	//Qq_(4,4) = 1.0; //process cov. of h
	
	R1_ = Eigen::MatrixXd::Zero(2,2);
	R1_(0,0) = options.tMeasurementNoiseCov->getValue()(0);
	R1_(1,1) = options.tMeasurementNoiseCov->getValue()(1);
	
	r2_ = options.tMeasurementNoiseCov->getValue()(2) * 0.0001;
	
	//update::
	y1_ << 0.0, 0.0;
	S1_ = Eigen::Matrix2d::Zero(2,2);
	K1_ = Eigen::MatrixXd::Zero(5,2);
	
	y2_ = 0.0;
	s2_ = 0.0;
	k2_ = Eigen::VectorXd::Zero(5);
	
	//misc:
	firstFlow = true;
	firstImu = true;
	
	//flow:
	flowCounter = 0;
	doUpdate = true;
	onGround = true;
	//yaw45 = sqrt(2.0)/2.0;
	
	//state:
	stateMsg.header.seq = 0;
	
	//IMU:
	imuCounter = 0;
	gw << 0.0, 0.0, -9.81;
	//from manual IMU calib in matlab:
	imuScale = options.tImuScale->getValue();
	imuOffset = options.tImuOffset->getValue();
	gyroScale = options.tGyroScale->getValue();
	
	//debug:
	saveToFile = options.tSaveFiles->getValue();
	if (saveToFile) {
		std::string filenameImu = "flow_imu";
		std::string filenameFlow = "flow_flow";
		
		if (options.tUniqueNames->getValue()) {
			int i = (int)ros::Time::now().toSec() % 1000000;
		
			std::cout << "i: " << i << "\n";
			
			filenameImu = filenameImu + "_" + boost::to_string(i) + ".txt";
			filenameFlow = filenameFlow + "_" + boost::to_string(i) + ".txt";
		} else {
			filenameImu += ".txt";
			filenameFlow += ".txt";
		}
		
		std::string locationImu = options.tFilesLocation->getValue() + filenameImu;
		std::string locationFlow = options.tFilesLocation->getValue() + filenameFlow;
		
		fileImu.open(locationImu.c_str(), std::ios::trunc);
		fileFlow.open(locationFlow.c_str(), std::ios::trunc);
		
		std::cout << "locationImu: " << locationImu << "\n";
		std::cout << "locationFlow: " << locationFlow << "\n";
	}
}
  
void FlowImuStateEstimator::initialize()
{
	nh_ = ROSModule::Instance().getMainNodeHandle();
	initVariables();
}

void FlowImuStateEstimator::willBecomeActive()
{
	imuSub_ = nh_.subscribe<tk_draft_msgs::TKSmallImu>(
			options.tSmurfInterfaceTopicName->getValue(),100, &FlowImuStateEstimator::imuCallback, this);
	
	flowSub_ = nh_.subscribe<px_comm::OpticalFlow>(
			options.tFlowTopicName->getValue(), 100, &FlowImuStateEstimator::flowCallback, this);
	
	behaviorSub_ = nh_.subscribe<telekyb_msgs::Behavior>(
			options.tBehaviorTopicName->getValue(), 100, &FlowImuStateEstimator::behaviorCallback, this);
	
	tkStatePub_ = nh_.advertise<telekyb_msgs::TKState>("FlowImuVelocity", 1);
}

void FlowImuStateEstimator::willBecomeInActive()
{
	imuSub_.shutdown();
	flowSub_.shutdown();
}

void FlowImuStateEstimator::destroy()
{

}

std::string FlowImuStateEstimator::getName() const
{
	return "FlowImuStateEstimator";
}

void FlowImuStateEstimator::imuCallback(const tk_draft_msgs::TKSmallImu::ConstPtr& msg)
{
	//------- IMU CALIBRATION --------//
	imuAcc[0] = (msg->linear_acceleration.x + imuOffset[0]) * imuScale[0];
	imuAcc[1] = (msg->linear_acceleration.y + imuOffset[1]) * imuScale[1]; 
	imuAcc[2] = (msg->linear_acceleration.z + imuOffset[2]) * imuScale[2];
	
	double accX = std::min(std::max(imuAcc[0]/9.81,-1.0),1.0);
	double accY = std::min(std::max(imuAcc[1]/9.81,-1.0),1.0);
	double accZ = std::min(std::max(imuAcc[2]/9.81,-1.0),1.0);
	
	if (firstImu) {
		firstImu = false;
		
		timePrevImu = msg->header.stamp.toSec();
		
		imuAngVel[0] = msg->angular_velocity.x * gyroScale[0];
		imuAngVel[1] = msg->angular_velocity.y * gyroScale[1];
		imuAngVel[2] = msg->angular_velocity.z * gyroScale[2];
		
		estRoll = atan2(-accY, -accZ);
		estPitch = atan2(accX, sqrt(accY*accY + accZ*accZ));
		
		return;
	}
	
	double dt = msg->header.stamp.toSec() - timePrevImu;
	timePrevImu = msg->header.stamp.toSec();
	
	//------- Low-Pass Filter for AngVelocity --------//
	double innoFilter = 0.2;
	
	imuAngVel[0] = (1.0-innoFilter) * imuAngVel[0] + innoFilter * msg->angular_velocity.x * gyroScale[0];
	imuAngVel[1] = (1.0-innoFilter) * imuAngVel[1] + innoFilter * msg->angular_velocity.y * gyroScale[1];
	imuAngVel[2] = (1.0-innoFilter) * imuAngVel[2] + innoFilter * msg->angular_velocity.z * gyroScale[2];
	
	//------- Roll and Pitch Estimation ----------//
	double estRollVel = imuAngVel[0];
	double estPitchVel = imuAngVel[1];
	
	double estRollAcc = atan2(-accY, -accZ);
	double estPitchAcc = atan2(accX, sqrt(accY*accY + accZ*accZ));
	
	//complementary filter:
	//double accGain = 0.0015;
	
	estRoll = (1.0 - ACCGAIN) * (estRoll + dt * estRollVel) + ACCGAIN * estRollAcc;
	estPitch = (1.0 - ACCGAIN) * (estPitch + dt * estPitchVel) + ACCGAIN * estPitchAcc;
	
	if (imuCounter < 10) { //10 samples to stabilise Roll and Pitch
		imuCounter++;
		return;
	}
	//------------- Quad to Horizontal ---------//
	Eigen::Matrix3d Rpitch, Rroll;
	Rroll <<	1.0,          0.0,           0.0,
			0.0, cos(estRoll), -sin(estRoll),
			0.0, sin(estRoll),  cos(estRoll);
	
	Rpitch <<  	cos(estPitch), 0.0, sin(estPitch), 
			0.0,           1.0,           0.0,
			-sin(estPitch), 0.0, cos(estPitch);
		  
	Rqh = Rpitch * Rroll;
	
	//------------- PREDICTION -----------------//
	A_(3,2) = dt;
	A_(4,4) = 1.0 - ALPHA;
	
	B_(0,0) = dt;
	B_(1,1) = dt;
	B_(2,2) = dt;
	
	u_ = Rqh * imuAcc - gw; //TODO: check and change?
	
	Qq_ = B_* Qu_ * B_.transpose();
	Qq_(3,3) = 0.01 * dt; //process cov. of z
	Qq_(4,4) = 0.001 * dt; //process cov. of h
	
	q_ = A_ * q_ + B_ * u_;
	P_ = A_ * P_ * A_.transpose() + Qq_;
	
	//-------------- TK State ----------------//
	publishTKState(msg->header.stamp);
	
	if (saveToFile) {
		fileImu << msg->header.stamp << "," << 0.0 << "," << 0.0 << "," << q_(3); //1,2,3,4: time, x,y,z
		fileImu << "," << q_(0) << "," << q_(1) << "," << q_(2);//5,6,7: velX, velY, velZ
		fileImu << "," << imuAcc(0) << "," << imuAcc(1) << "," << imuAcc(2);//8,9,10
		fileImu << "," << estRoll << "," << estPitch << "," << 0.0;//11,12,13
		fileImu << "," << imuAngVel(0) << "," << imuAngVel(1) << "," <<  imuAngVel(2);//14,15,16
		fileImu << "," << q_(4); //d - obstacle height 17
		fileImu << "\n";
	}
}

void FlowImuStateEstimator::flowCallback(const px_comm::OpticalFlowConstPtr& msg)
{
	if (firstFlow) {
		firstFlow = false;

		timePrevFlow = msg->header.stamp.toSec();
		z2_ = -msg->ground_distance;
		q_(3) = z2_;
		
		return;
	}
	
	if (imuCounter < 10) return;
	
	double dt = msg->header.stamp.toSec() - timePrevFlow;
	timePrevFlow = msg->header.stamp.toSec();
	
	double dz = -msg->ground_distance - z2_;
	uint8_t quality = msg->quality;
	
	double velZ = dz/dt;
	
	//TODO: add something here, distance from the sonar to the center of the robot's frame:
	z2_ = -msg->ground_distance;
	
	z1_(0) = YAW45 * (-msg->velocity_x - msg->velocity_y);
	z1_(1) = YAW45 * (msg->velocity_x - msg->velocity_y);
	
	if (onGround) {
		quality = 255;
		velZ = 0.0;
		//TODO: parametrize this
		z2_ = -0.3;
		z1_(0) = 0.0;
		z1_(1) = 0.0;
		
		r2_ = 0.0;
		R1_ = Eigen::Matrix2d::Zero(2,2);
	}
	
	//----------------- Outliers Elimination --------------//
	/*
	if (fabs(velZ) > 1.0 and doUpdate) {
		doUpdate = false; //very harsh condition, 2m/s^2
		velZ = 0.0;
	}
	
	if (doUpdate == false) {
		if (fabs(velZ) > 1.0 or flowCounter > 2) { 
			doUpdate = true;
			velZ = 0.0;
			flowCounter = 0;
		}
		flowCounter++;
	}
	*/
	if (msg->ground_distance < 0.25 or msg->ground_distance > 2.5) {
		doUpdate = false;
		std::cout << "doUpdate: false, z: " << msg->ground_distance << "\n";
	} else {
		doUpdate = true;
	}
	
	//----------------- DISTANCE UPDATE --------------//
	if (doUpdate) {
		y2_ = z2_ - c2_.transpose() * q_;//q_(3) + q_(4);
		s2_ = c2_.transpose() * P_ * c2_ + r2_;
		//TODO: check if S is invertible
		k2_ = 1/s2_ * (P_ * c2_);
		
		q_ = q_ + k2_ * y2_;
		P_ = (Eigen::MatrixXd::Identity(5,5) - k2_ * c2_.transpose()) * P_;
		
		//std::cout << "Kalman Gain k2: \n" << k2_ << "\n";
	}
	
	//---------------- VELOCITY UPDATE ---------------//
	if (quality > 1) {
		y1_ = z1_ - C1_ * q_;
		S1_ = C1_ * P_ * C1_.transpose() + R1_;
		//if (S_.FullPivLU::isInvertible()) {}; //TODO: check if S is invertible
		K1_ = P_ * C1_.transpose() * S1_.inverse();
		
		q_ = q_ + K1_ * y1_;
		P_ = (Eigen::MatrixXd::Identity(5,5) - K1_ * C1_) * P_;
	}
	
	if (saveToFile) {
	  	//ros::Time stamp = msg->header.stamp;
		fileFlow << msg->header.stamp << "," << (int)quality << "," << z2_; //time, quality, distance
		fileFlow << "," << z1_(0) << "," << z1_(1) << "," << 0.0; //vx, vy, vz
		fileFlow << "\n";
	} 	
}

void FlowImuStateEstimator::publishTKState(ros::Time stamp)
{
	Eigen::Vector3d linVelH;
	linVelH << q_(0), q_(1), q_(2); 
	
	Eigen::Vector3d angVelH;
	angVelH = Rqh * imuAngVel;
	
	Eigen::Quaternion<double> quatNEDEigen(Rqh);
	
	Velocity3D velNED(linVelH);
	Velocity3D angVelNED(angVelH);
	Position3D posNED(0.0,0.0,q_(3));
	//Quaternion from telekyb: w,x,y,z:
	Quaternion quatNED(quatNEDEigen.w(), quatNEDEigen.x(), quatNEDEigen.y(), quatNEDEigen.z());

	TKState tStateMsg;
	tStateMsg.time = Time(stamp);
	tStateMsg.position = posNED;
	tStateMsg.linVelocity = velNED;
	tStateMsg.orientation = quatNED;
	tStateMsg.angVelocity = angVelNED;

	stateEstimatorController.activeStateCallBack(tStateMsg);
	
	//-------------- TK State ----------------//
	stateMsg.header.seq += 1;
	stateMsg.header.stamp = stamp;
	
	stateMsg.pose.position.z = q_(3);
	//tf::quaternionEigenToMsg(quatNEDEigen, stateMsg.pose.orientation);
	stateMsg.pose.orientation.w = quatNEDEigen.w();	
	stateMsg.pose.orientation.x = quatNEDEigen.x();
	stateMsg.pose.orientation.y = quatNEDEigen.y();
	stateMsg.pose.orientation.z = quatNEDEigen.z();

//TODO: filtered or not? maybe from angle estimator?
	stateMsg.twist.angular.x = angVelH[0]; 
	stateMsg.twist.angular.y = angVelH[1];
	stateMsg.twist.angular.z = angVelH[2];
	
	stateMsg.twist.linear.x = linVelH(0);
	stateMsg.twist.linear.y = linVelH(1);
	stateMsg.twist.linear.z = linVelH(2);
	
	tkStatePub_.publish(stateMsg);
} 
  
void FlowImuStateEstimator::behaviorCallback(const telekyb_msgs::BehaviorConstPtr& msg)
{
//TODO: set onGround to false when the bahavior switches;
	if (onGround) {
		onGround = false;
		R1_(0,0) = options.tMeasurementNoiseCov->getValue()(0);
		R1_(1,1) = options.tMeasurementNoiseCov->getValue()(1);
		
		r2_ = options.tMeasurementNoiseCov->getValue()(2);
		std::cout << "FlowImuStateEstimator::behaviorCallback: fist behaviour change, onGround=false\n";
		std::cout << "Distance cov.: " << r2_ << "\n";
		std::cout << "VelX and VelY cov.: \n" << R1_ << "\n";
		std::cout << "Process Cov Qq: \n" << Qq_ << "\n";
	}
}

} /* namespace state_estimators_plugin */

