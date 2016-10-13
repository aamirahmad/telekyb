/*
 * ComplementaryStateEstimator.cpp
 *
 *  Created on: Jul 11, 2012
 *      Author: rspica
 */

#include <StateEstimators/ComplementaryFilter/ComplementaryStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include <telekyb_defines/physic_defines.hpp>

#include <telekyb_base/ROS.hpp>
#include <telekyb_msgs/TKState.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <boost/numeric/odeint.hpp>

#include <cmath>

PLUGINLIB_EXPORT_CLASS( telekyb_state::ComplementaryStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

ComplementaryStateEstimatorOptions::ComplementaryStateEstimatorOptions()
	: OptionContainer("ComplementaryStateEstimator")
{
	tViconSeTopicName = addOption<std::string>("tViconSeTopicName","TopicName of Vicon Sensor.","undef",true,true);
	tImuSeTopicName = addOption<std::string>("tImuSeTopicName","TopicName of Imu Sensor.","undef",true,true);

	tRepublishVicon = addOption<bool>("tRepublishVicon","Specify if Vicon must be republished.", false, false, true);
	tViconRepubTopicName = addOption<std::string>("tViconRepubTopicName","TopicName for Vicon republishing.", "RepublishedVicon", false, true);

	imuAccBias = addOption<Eigen::Vector3d>("imuAccBias","Imu accelerometer bias", Eigen::Vector3d::Zero(), false, true);
	imuGyrBias = addOption<Eigen::Vector3d>("imuGyrBias","Imu gyroscope bias", Eigen::Vector3d::Zero(), false, true);
	vicPosition = addOption<Eigen::Vector3d>("vicPosition","Position of the Vicon frame in the IMU frame", Eigen::Vector3d::Zero(), false, true);
	vicOrientation = addOption<Eigen::Vector2d>("vicOrientation","Orientation of the Vicon frame w.r.t. the IMU frame (roll and pitch)", Eigen::Vector2d::Zero(), false, true);
	worldGravity = addOption<Eigen::Vector3d>("worldGravity","Gravity vector in the world frame", Eigen::Vector3d(0.0,0.0,GRAVITY), false, true);

	predStep = addOption<double>("predStep", "Step size for state estimation", 1e-3, false, true);
	pubStep = addOption<double>("pubStep", "Step size for state publication", 1e-2, false, true);
	normalizationGain = addOption<double>("normalizationGain", "Gain for quaternon normalization", 1, false, true);

	positionGain = addOption<Eigen::Vector3d>("positionGain","Position gain for the estimation", 33*Eigen::Vector3d::Ones(), false, true);
	velocityGain = addOption<Eigen::Vector3d>("velocityGain","Velocity gain for the estimation", 362*Eigen::Vector3d::Ones(), false, true);
	rotationGain = addOption<Eigen::Vector3d>("rotationGain","Orientation gain for the estimation", 10*Eigen::Vector3d::Ones(), false, true);
	biasGain = addOption<Eigen::Vector3d>("biasGain","Bias gain for the estimation", Eigen::Vector3d::Zero(), false, true);

	matrixA = addOption<Eigen::Matrix3d>("matrixA","Matrix A", Eigen::Matrix3d::Identity(), false, true);
	vectorB = addOption<Eigen::Vector3d>("vectorB","Vector B", Eigen::Vector3d::Zero(), false, true);

	tOmegaFilterFreq = addOption<double>("tOmegaFilterFreq", "Omega filter cutoff frequency in Hz", 30.0, false, true);
	tOmegaSampleTime = addOption<double>("tOmegaSampleTime", "Omega filter sample time in s", 1.0/1080.0, false, true);
}


void ComplementaryStateEstimator::initVelocityFilters()
{
	IIRLowPass isLowPass;
	for(int i=0;i<3;i++){
		omegaFilter[i] = new IIRFilter(
				isLowPass,
				2.0*M_PI*options.tOmegaFilterFreq->getValue(),
				1.0,
				options.tOmegaSampleTime->getValue());
	}
}



ComplementaryStateEstimator::ComplementaryStateEstimator():
	spinning(false),
	isInitialized(false),
	receivedFirstInput(false),
	receivedFirstMeasure(false){
}

void ComplementaryStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
	system = DynamicSystem(options.positionGain->getValue(), options.velocityGain->getValue(), options.rotationGain->getValue(), options.biasGain->getValue(),
			options.worldGravity->getValue(), options.normalizationGain->getValue(), options.matrixA->getValue(), options.vectorB->getValue());

	initVelocityFilters();


}

void ComplementaryStateEstimator::initializationDone()
{
	boost::mutex::scoped_lock(threadMutex);
	if (!isInitialized) {
		isInitialized = true;

		spinning = true;
		ROS_INFO("Initialization Done. Starting State Estimation Thread...");
		thread = boost::thread(&ComplementaryStateEstimator::spin, this);
	}
}

void ComplementaryStateEstimator::spin()
{
	pubTimer.reset();
	ros::Rate rate(1/options.predStep->getValue());
	while(spinning)
		{
//			rtTimer.reset();

			prediction(internalState, options.predStep->getValue());

/*			if (pubTimer.getElapsed()>=options.pubStep->getValue()){
				publishState();
				pubTimer.reset();
			}*/
			publishState();

//			Time toSleep = Time(options.predStep->getValue()) - rtTimer.getElapsed();
//			if (toSleep>0) {
//				//toSleep.sleep();
//			} else {
//				ROS_WARN("State estimation was too slow!");
//			}
			rate.sleep();
	}
}

// Prediction function ====================================================
void ComplementaryStateEstimator::prediction(State& state, double predStep)
{

	ROS_DEBUG_STREAM("Performing prediction with state... " << std::endl
		<< "Position: [" << state.position.transpose() << "]'" << std::endl
		<< "Velocity: [" << state.lin_velocity.transpose() << "]'" << std::endl
		<< "Orientation: [" << state.orientation.w() << " " << state.orientation.vec().transpose() << "]'" << std::endl
		<< "... input ... " << std::endl
		<< "Linear acceleration: [" << system.getInput().lin_acc.transpose() << "]'" << std::endl
		<< "Angular velocity: [" << system.getInput().ang_vel.transpose() << "]'" << std::endl
		<< "... and measure... " << std::endl
		<< "Position: [" << system.getMeasure().position.transpose() << "]'" << std::endl
		<< "Orientation: [" << system.getMeasure().orientation.w() << " " << system.getMeasure().orientation.vec().transpose() << "]'" << std::endl
		<< "Prediction step: " << predStep << std::endl);

	typedef boost::numeric::odeint::runge_kutta4< State, double, State, double, boost::numeric::odeint::vector_space_algebra> stepper;
	boost::numeric::odeint::integrate_const( stepper() , system , state , 0.0, predStep, predStep);

	ROS_DEBUG_STREAM("Resulting in the state... " << std::endl
			<< "Position: [" << state.position.transpose() << "]'" << std::endl
			<< "Velocity: [" << state.lin_velocity.transpose() << "]'" << std::endl
			<< "Orientation: [" << state.orientation.w() << " " << state.orientation.vec().transpose() << "]'" << std::endl);

	return;
}

void ComplementaryStateEstimator::inputCallback(const tk_draft_msgs::TKSmallImuConstPtr& msg)
{
	boost::mutex::scoped_lock scopedLock(imuMutex);

	/* Filter omega */
	double output[3];
	std::vector<double> input(1);
	input[0]=msg->angular_velocity.x;
	omegaFilter[0]->step(input, output[0]);
	input[0] = msg->angular_velocity.y;
	omegaFilter[1]->step(input, output[1]);
	input[0] = msg->angular_velocity.z;
	omegaFilter[2]->step(input, output[2]);

	angVelFiltered = Eigen::Vector3d(output[0], output[1], output[2]);

	/* Note: sending non filtered angular velocity to the complementary filter */
	Input lastInput;
	lastInput.ang_vel = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

	lastInput.lin_acc = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	system.setInput(lastInput);


	ROS_DEBUG_STREAM("Imu received at system time: " << ros::Time::now() << std::endl
		<< "Linear acceleration: [" << lastInput.ang_vel.transpose() << "]'" << std::endl
		<< "Angular velocity: [" << lastInput.lin_acc.transpose() << "]'" << std::endl);

	if (!isInitialized){
		if(!receivedFirstInput) {
			ROS_INFO_STREAM("Complementary state estimator received the first input containing: " << std::endl
							<< "Angular velocity: [" << lastInput.ang_vel.transpose() << "]'" << std::endl
							<< "Linear acceleration: [" << lastInput.lin_acc.transpose() << "]'");

			receivedFirstInput = true;
		}
		if(receivedFirstMeasure){
			internalState.position = system.getMeasure().position;
			internalState.lin_velocity = Eigen::Vector3d::Zero(); //assumes we start from zero velocity
			internalState.orientation = system.getMeasure().orientation;
			internalState.bias = options.imuAccBias->getValue();

			initializationDone();
			ROS_INFO_STREAM("Complementary state estimator is ready to go!" << std::endl
					<< "Initialized with:" << std::endl
					<< "Position: [" << internalState.position.transpose() << "]'" << std::endl
					<< "Velocity: [" << internalState.lin_velocity.transpose() << "]'" << std::endl
					<< "Orientation: [" << internalState.orientation.w() << " " << internalState.orientation.vec().transpose() << "]'" << std::endl);
		}
	}

	scopedLock.unlock();

}


void ComplementaryStateEstimator::measureCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	boost::mutex::scoped_lock scopedLock(viconMutex);

	Eigen::Quaterniond receivedQuaternion(-msg->pose.orientation.w, -msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
	receivedQuaternion = receivedQuaternion*Eigen::Quaterniond(cos(0.5*options.vicOrientation->getValue()(1)),0.0,-sin(0.5*options.vicOrientation->getValue()(1)),0.0)*
					       Eigen::Quaterniond(cos(0.5*options.vicOrientation->getValue()(0)),-sin(0.5*options.vicOrientation->getValue()(0)),0.0,0.0);
	if (abs(receivedQuaternion.squaredNorm()-1)>1e-5){
		ROS_WARN("Received non valid quaternion. Ignoring measure!");
		return;
	}

	Measure lastMeasure;
	lastMeasure.position = Eigen::Vector3d(msg->pose.position.x, -msg->pose.position.y, -msg->pose.position.z);
	lastMeasure.orientation = receivedQuaternion;
	system.setMeasure(lastMeasure);
	scopedLock.unlock();

	if (options.tRepublishVicon->getValue()){
		geometry_msgs::PoseStamped repubMsg(*msg);
		repubMsg.header.stamp = ros::Time::now();
		viconRepublisher.publish(repubMsg);
	}

	ROS_DEBUG_STREAM("Measure received at system time: " << ros::Time::now() << std::endl
		<< "Position: [" << lastMeasure.position.transpose() << "]'" << std::endl
		<< "Orientation: [" << lastMeasure.orientation.w() << " " << lastMeasure.orientation.vec().transpose() << "]'" << std::endl);

	if (!isInitialized){
		if(!receivedFirstMeasure){
			receivedFirstMeasure = true;
			ROS_INFO_STREAM("Complementary state estimator received the first measure containing: " << std::endl
								<< "Position: [" << lastMeasure.position.transpose() << "]'" << std::endl
								<< "Orientation: [" << lastMeasure.orientation.w() << " " << lastMeasure.orientation.vec().transpose() << "]'" << std::endl);
		}
		if(receivedFirstInput){
			internalState.position = system.getMeasure().position;
			internalState.lin_velocity = Eigen::Vector3d::Zero(); //assumes we start from zero velocity
			internalState.orientation = system.getMeasure().orientation;
			internalState.bias = options.imuAccBias->getValue();
			//isInitialized = true;
			initializationDone();
			ROS_INFO_STREAM("Complementary state estimator is ready to go!" << std::endl
					<< "Initialized with:" << std::endl
					<< "Position: [" << internalState.position.transpose() << "]'" << std::endl
					<< "Velocity: [" << internalState.lin_velocity.transpose() << "]'" << std::endl
					<< "Orientation: [" << internalState.orientation.w() << " " << internalState.orientation.vec().transpose() << "]'" << std::endl);
		}
	}

}

void ComplementaryStateEstimator::publishState(){

	TKState tStateMsg;

	tStateMsg.time = ros::Time::now();

	// empty msg
	tStateMsg.position = internalState.position;
	tStateMsg.linVelocity = internalState.lin_velocity;
	tStateMsg.orientation = internalState.orientation;

	boost::mutex::scoped_lock scopedImuLock(imuMutex);
	tStateMsg.angVelocity = angVelFiltered;
	scopedImuLock.unlock();

	stateEstimatorController.activeStateCallBack(tStateMsg);

	geometry_msgs::Vector3Stamped biasMsg;
	biasMsg.header.stamp = tStateMsg.time.toRosTime();
	biasMsg.vector.x = internalState.bias(0);
	biasMsg.vector.y = internalState.bias(1);
	biasMsg.vector.z = internalState.bias(2);

	biasPub.publish(biasMsg);
}


void ComplementaryStateEstimator::willBecomeActive()
{
	viconSub = nodeHandle.subscribe<geometry_msgs::PoseStamped>(
			options.tViconSeTopicName->getValue(),1, &ComplementaryStateEstimator::measureCallback, this);


	imuSub = nodeHandle.subscribe<tk_draft_msgs::TKSmallImu>(
			options.tImuSeTopicName->getValue(),1, &ComplementaryStateEstimator::inputCallback, this);

	biasPub = nodeHandle.advertise<geometry_msgs::Vector3Stamped>("AccelerometerBias",1);

	if (options.tRepublishVicon->getValue()){
		viconRepublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>(options.tViconRepubTopicName->getValue(),1);
	}
}

void ComplementaryStateEstimator::willBecomeInActive()
{
	viconSub.shutdown();
	imuSub.shutdown();
	biasPub.shutdown();
}

void ComplementaryStateEstimator::destroy()
{
	for(int i=0;i<3;i++){
		delete omegaFilter[i];
	}

	spinning = false;
	thread.join();
}

std::string ComplementaryStateEstimator::getName() const
{
	return std::string("ComplementaryStateEstimator");
}

} /* namespace telekyb_state */
