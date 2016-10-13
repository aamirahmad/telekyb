/*
 * ServoInterface.cpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */
#include <math.h>
#include <limits>
#include <StateEstimators/KalmanStateEstimator.hpp>

#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

#include <telekyb_msgs/TKState.h>
#include <telekyb_defines/physic_defines.hpp>

PLUGINLIB_EXPORT_CLASS( telekyb_state::KalmanStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

KalmanStateEstimatorOptions::KalmanStateEstimatorOptions()
	: OptionContainer("Kalman")
{
	imuTopic = addOption<std::string>("imuTopic", "Imu data topic name", "TKImu", false, true);
	viconTopic = addOption<std::string>("viconTopic", "Vicon data topic name", "TKVicon", false, true);
	stateTopic = addOption<std::string>("stateTopic", "Topic on which the state must be published", "TKKalman", false, true);

	Eigen::Matrix<double,6,6> rt;
	//TODO set a meaningful default covariance matrix
	rt.Zero();
	imuCov = addOption<Eigen::Matrix<double,6,6> >("imuCov", "IMU noise covariance matrix", rt, false, true);

	imuAccBias = addOption<Eigen::Vector3d>("imuAccBias", "IMU accelerometer bias", Eigen::Vector3d::Zero(), false, true);
	imuGyrBias = addOption<Eigen::Vector3d>("imuGyrBias", "IMU gyroscope bias", Eigen::Vector3d::Zero(), false, true);

	Eigen::Matrix<double,9,9> initC;
	//TODO set a meaningful default covariance matrix
	initC.Zero();
	initStateCov = addOption<Eigen::Matrix<double,9,9> >("initStateCov", "Initial state covariance estimate", initC, false, true);

	vicPosition = addOption<Eigen::Vector3d>("vicPosition", "Initial estimate for Vicon position w.r.t. the IMU", Eigen::Vector3d(0.0,0.0,0.0), false, true);
	vicOrientation = addOption<Eigen::Quaterniond>("vicOrientation", "Initial estimate for Vicon orientation w.r.t. the IMU", Eigen::Quaterniond(1.0,0.0,0.0,0.0), false, true);

	predStep = addOption<double>("predStep", "Prediction time step", 1e-3, false, true);
	minStep = addOption<double>("minStep", "Minimum prediction time step", 1e-3, false, true);
	saveStep = addOption<double>("minStep", "Time step for saving states", 1e-3, false, true);

	lTpred = addOption<double>("lTpred", "Multiplier for quaternion prediction normalization", 0.8, false, true);

	maxStateBufferSize = addOption<int>("maxStateBufferSize", "Maximum number of states to be saved in the state buffer", 200, false, true);
	maxMeasureBufferSize = addOption<int>("maxMeasureBufferSize", "Maximum number of measures to be saved in the measure buffer", 200, false, true);
	maxInputBufferSize = addOption<int>("maxInputBufferSize", "Maximum number of inputs to be saved in the input buffer", 200, false, true);

}

void KalmanStateEstimator::initialize()
{
	isInitialized = false;
	isInitializedLinVel = false;
	isInitializedPose = false;
	isInitializedAngVel = false;

	newMeasureReceived = false;
	newInputReceived = false;

	nodeHandle = ROSModule::Instance().getMainNodeHandle();
}

void KalmanStateEstimator::core()
{
	boost::mutex::scoped_lock scopedInputMutex(newInputMutex);
	if (newInputReceived)
	{
		//delete first input if the buffer is too big
		if (inputBuffer.size()==options.maxInputBufferSize->getValue()) inputBuffer.pop_front();

		//insert new input in the buffer
		inputBuffer.push_back(newInput);

		//synchronize internal time with the IMU
		intTime = Time(newInput.timeStamp);
	}
	scopedInputMutex.unlock();

	boost::mutex::scoped_lock scopedMeasureMutex(newMeasureMutex);
	if (newMeasureReceived)
	{
		//Check if the current measure is newer than the oldest saved state, input and measure.
		std::deque<StateBufferElement>::iterator stateBufferIndex;
		std::deque<InputBufferElement>::iterator inputBufferIndex;
		std::deque<MeasureBufferElement>::iterator measureBufferIndex;

		if (((stateBufferIndex = searchInBuffer<StateBufferElement>(stateBuffer,newMeasure.timeStamp)) < stateBuffer.begin()) ||
			((inputBufferIndex = searchInBuffer<InputBufferElement>(inputBuffer,newMeasure.timeStamp)) < inputBuffer.begin()) ||
			((measureBufferIndex = searchInBuffer<MeasureBufferElement>(measureBuffer,newMeasure.timeStamp)) < measureBuffer.begin()))
		{
			scopedMeasureMutex.unlock();
			ROS_WARN("A measure has been neglected because it was too old. Consider increasing buffer size");
		} else {
			// add measure to the buffer (delete oldest entry if the buffer is too big)
			if ((int)measureBuffer.size()==options.maxMeasureBufferSize->getValue()) measureBuffer.pop_front();
			measureBuffer.insert(measureBufferIndex-1,newMeasure);
			scopedMeasureMutex.unlock();

			// Delete newer states from buffer (must be recomputed)
			stateBuffer.erase(stateBufferIndex,stateBuffer.end());

			// set filter position
			currentInputIndex = inputBufferIndex;
			nextMeasureIndex = measureBufferIndex;
			predTime = stateBufferIndex->timeStamp;
			saveTimer.reset();
		}
	}

	// wait until initialization is completed before proceeding to the actual estimation
	if (!isInitialized) return;


	double nextMeasureTime, nextInputTime, step;
	if (nextMeasureIndex > measureBuffer.end())
	{
		nextMeasureTime = INFINITY;
	} else {
		nextMeasureTime = nextMeasureIndex->timeStamp;
	}

	if (currentInputIndex >= inputBuffer.end())
	{
		nextInputTime = INFINITY;
	} else {
		nextInputTime = (currentInputIndex+1)->timeStamp;
	}

	bool done = false;
	while (!done)
	{
		int condition = tools.min(Eigen::Vector4d(options.predStep->getValue(),
													intTime.toDSec()-predTime,
													nextInputTime-predTime,
													nextMeasureTime-predTime), step);
		if (step>=options.minStep->getValue())
		{
			prediction(internalState,*currentInputIndex,step);
		}
		predTime += step;

		switch (condition)
		{
			case (1): //prediction is complete
				done = true;
				break;
			case (2): //input must be changed
				currentInputIndex++;
				if (currentInputIndex == inputBuffer.end())
				{
					nextInputTime = INFINITY;
				} else {
					nextInputTime = (currentInputIndex+1)->timeStamp;
				}
				break;

			case (3): //update is required
				viconHandler.update(internalState,*nextMeasureIndex.measure);
				nextMeasureIndex++;
				if (nextMeasureIndex == measureBuffer.end())
					{
						nextMeasureTime = INFINITY;
					} else {
						nextMeasureTime = (nextMeasureIndex)->timeStamp;
					}
				break;
			default:
				ROS_ERROR("Unexpected condition");
				break;
		}

		if (saveTimer.getElapsed() >= options.saveStep->getValue())
		{
			stateBuffer.push_back(internalState);
			saveTimer.reset();
		}
	}

	//TODO publish timer (if desired)
	publishEstimate(internalState);
}

void KalmanStateEstimator::publishEstimate(const StateBufferElement & estimate)
{
	statePub.publish(estimate.state);
	return;
}


void KalmanStateEstimator::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	boost::mutex::scoped_lock scopedLockMutex(newInputMutex);
	//save the new input in the buffer
	newInput.timeStamp = msg->header.stamp.sec;
	newInput.input = *msg;
	newInputReceived = true;

	if (!isInitialized)
	{
		if (!isInitializedAngVel)
		{
			internalState.state.twist.angular = msg->angular_velocity;
			isInitializedAngVel = true;
		}
		if (isInitializedLinVel)
		{
			internalState.covariance = options.initStateCov->getValue();
			isInitialized = true;
		}
	}

	scopedLockMutex.unlock();
}


void KalmanStateEstimator::viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	boost::mutex::scoped_lock scopedLockMutex(newMeasureMutex);
	MeasureBufferElement measure;
	newMeasure.timeStamp = msg->header.stamp.sec;
	newMeasure.measure = *msg;
	newMeasureReceived = true;
	if(!isInitialized)
	{
		if (!isInitializedLinVel)
		{
			if(!isInitializedPose)
			{
				internalState.state.pose.position.x = msg->transform.translation.x;
				internalState.state.pose.position.y = msg->transform.translation.y;
				internalState.state.pose.position.z = msg->transform.translation.z;

				internalState.state.pose.orientation.w = msg->transform.rotation.w;
				internalState.state.pose.orientation.x = msg->transform.rotation.x;
				internalState.state.pose.orientation.y = msg->transform.rotation.y;
				internalState.state.pose.orientation.z = msg->transform.rotation.z;

				internalState.timeStamp = msg->header.stamp.toSec();
				internalState.state.header.stamp.fromSec(internalState.timeStamp);
				isInitializedPose = true;
			} else {
				internalState.state.twist.linear.x = (msg->transform.translation.x - internalState.state.pose.position.x)/(msg->header.stamp.toSec()-internalState.timeStamp);
				internalState.state.twist.linear.y = (msg->transform.translation.y - internalState.state.pose.position.y)/(msg->header.stamp.toSec()-internalState.timeStamp);
				internalState.state.twist.linear.z = (msg->transform.translation.z - internalState.state.pose.position.z)/(msg->header.stamp.toSec()-internalState.timeStamp);
				isInitializedLinVel = true;
			}
		} else if (isInitializedAngVel)
			isInitialized = true;
	}

	scopedLockMutex.unlock();
}

// Prediction function ====================================================
void KalmanStateEstimator::prediction(StateBufferElement& state, const InputBufferElement& u, double Tpred)
{
	double lTpred = options.lTpred->getValue();

	//read the state
	Eigen::Vector3d  r(state.state.pose.position.x,state.state.pose.position.y,state.state.pose.position.z);
	Eigen::Vector3d dr(state.state.twist.linear.x,state.state.twist.linear.y,state.state.twist.linear.z);
	Eigen::Quaterniond q(state.state.pose.orientation.w,state.state.pose.orientation.x,state.state.pose.orientation.y,state.state.pose.orientation.z);

	//read input commands
	Eigen::Vector3d w(u.input.angular_velocity.x,u.input.angular_velocity.y,u.input.angular_velocity.z);
	Eigen::Vector3d a(u.input.linear_acceleration.x,u.input.linear_acceleration.y,u.input.linear_acceleration.z);

	w -= options.imuGyrBias->getValue();
	a -= options.imuAccBias->getValue();

	//compute coefficients
	Eigen::Matrix4d qqT;
	qqT(0) = q.w()*q.w();
	qqT.block<3,3>(1,1) = q.vec()*q.vec().transpose();
	qqT.block<3,1>(0,1) = q.w()*q.vec();
	qqT.block<1,3>(1,0) = qqT.block<3,1>(0,1).transpose();
	double qTq = qqT.trace();

	double s = .5*w.norm()*Tpred;

	double sss,f3;
	if (abs(s)<std::numeric_limits<double>::epsilon())
	{
		sss = 1;
		f3 = -1/3*pow(.5*Tpred,3);
	} else {
		sss = sin(s)/s;
		f3 = pow(.5*Tpred,3)*(cos(s)-sss)/pow(s,2);
	}

	double k = 1-qTq;
	double f1 = cos(s)+lTpred*k;
	double f2 = .5*Tpred*sss;

	r += dr*Tpred;
	dr += (q*a-Eigen::Vector3d(0,0,GRAVITY))*Tpred;
	q = tools.toQuaternion((Eigen::Vector4d)(f1*tools.toVector(q) + f2*tools.toVector(Eigen::Quaterniond(0.0,w(1),w(2),w(3))*q)));

	state.state.pose.position.x = r(0);
	state.state.pose.position.y = r(1);
	state.state.pose.position.z = r(2);

	state.state.twist.linear.x = dr(0);
	state.state.twist.linear.y = dr(1);
	state.state.twist.linear.z = dr(2);

	state.state.pose.orientation.w = q.w();
	state.state.pose.orientation.x = q.vec()(0);
	state.state.pose.orientation.y = q.vec()(1);
	state.state.pose.orientation.z = q.vec()(2);

	state.state.twist.angular.x = w(0);
	state.state.twist.angular.y = w(1);
	state.state.twist.angular.z = w(2);

	//Compute state-transition matrix
	Eigen::Matrix<double, 9, 9> G;
	G.Zero();

	G.block<3,3>(0,0).Identity();
	G.block<3,3>(0,3) = Tpred*Eigen::Matrix3d::Identity();

	G.block<3,3>(3,3).Identity();
	G.block<3,3>(3,6) = Tpred*tools.gamma(q,a)*tools.jacqp(q);

	Eigen::Matrix4d jacqq;
	jacqq = f1*Eigen::Matrix4d::Identity() + f2*tools.qLeft(Eigen::Quaterniond(0,w(0),w(1),w(2))) - 2*lTpred*qqT;
	G.block<3,3>(6,6) = tools.jacpq(q)*jacqq*tools.jacqp(q);

	//Compute noise matrix

	Eigen::Matrix<double, 4, 3> T;
	T = tools.matW(q)*(f2*Eigen::Matrix3d::Identity()+f3*(w*w.transpose()))-.5*Tpred*f2*tools.toVector(q)*w.transpose();

	Eigen::Matrix<double, 9, 6> F;
	F << 	Eigen::Matrix3d::Zero(),	Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), Tpred*q.toRotationMatrix(),
				   tools.jacpq(q)*T,	Eigen::Matrix3d::Zero();

	//Compute covariance prediction
	state.covariance = G*state.covariance*G.transpose() + (F*options.imuCov->getValue()*F.transpose() + options.discCov->getValue())*Tpred;

	state.timeStamp = state.state.header.stamp.toSec() + Tpred;
	state.state.header.stamp.fromSec(state.timeStamp);
}


void KalmanStateEstimator::willBecomeActive()
{
	imuSub = nodeHandle.subscribe(options.imuTopic->getValue(),1, &KalmanStateEstimator::imuCallback, this);
	statePub = nodeHandle.advertise<telekyb_msgs::TKState>(options.stateTopic->getValue(),1);
}


void KalmanStateEstimator::willBecomeInActive()
{
	imuSub.shutdown();
}

void KalmanStateEstimator::destroy(){}

std::string KalmanStateEstimator::getName() const
{
	return options.viconTopic->getValue();
}

} /* namespace telekyb_state */
