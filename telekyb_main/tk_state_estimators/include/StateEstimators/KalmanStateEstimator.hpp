/*
 * KalmanStateEstimator.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */

#ifndef KALMANSTATEESTIMATOR_HPP_
#define KALMANSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>
#include <EigenTools.hpp>
// plugin stuff
#include <pluginlib/class_list_macros.h>

// ROS
#include <sensor_msgs/Imu.h>
#include <telekyb_msgs/TKState.h>
#include <geometry_msgs/TransformStamped.h>
#include <telekyb_base/Time.hpp>

#include <stdint.h>
#include <deque>

#include <StateEstimators/MeasureHandler.hpp>

// Boost
#include <boost/thread/mutex.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class KalmanStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* imuTopic;
	Option<std::string>* viconTopic;
	Option<std::string>* stateTopic;

	Option<Eigen::Matrix<double,6,6> >* imuCov;
	Option<Eigen::Matrix<double,6,6> >* vicCov;
	Option<Eigen::Matrix<double,9,9> >* discCov;

	Option<Eigen::Matrix<double,9,9> >* initStateCov;


	Option<Eigen::Vector3d>* imuAccBias;
	Option<Eigen::Vector3d>* imuGyrBias;

	Option<Eigen::Vector3d>* vicPosition;
	Option<Eigen::Quaterniond>* vicOrientation;

	Option<double>* predStep;
	Option<double>* minStep;
	Option<double>* saveStep;
	Option<double>* lTpred;

	Option<int>* maxStateBufferSize;
	Option<int>* maxMeasureBufferSize;
	Option<int>* maxInputBufferSize;

	KalmanStateEstimatorOptions();
};

class KalmanStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
public:

	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

protected:
	KalmanStateEstimatorOptions options;

	ros::NodeHandle nodeHandle;

	ros::Subscriber imuSub;
	ros::Subscriber vicSub;

	ros::Publisher statePub;

	std::deque<StateBufferElement, Eigen::aligned_allocator<StateBufferElement> > stateBuffer;
	std::deque<MeasureBufferElement, Eigen::aligned_allocator<MeasureBufferElement> >  measureBuffer;
	std::deque<InputBufferElement, Eigen::aligned_allocator<InputBufferElement> > inputBuffer;

	std::deque<MeasureBufferElement, Eigen::aligned_allocator<StateBufferElement> >::const_iterator nextMeasureIndex;
	std::deque<InputBufferElement, Eigen::aligned_allocator<InputBufferElement> >::const_iterator currentInputIndex;

	StateBufferElement internalState;
	bool isInitialized;
	bool isInitializedLinVel;
	bool isInitializedPose;
	bool isInitializedAngVel;

	bool newMeasureReceived;
	bool newInputReceived;
	MeasureBufferElement newMeasure;
	InputBufferElement newInput;

	// Mutexes
	boost::mutex newMeasureMutex;
	boost::mutex newInputMutex;


	//Core functions
	void core();
	void prediction(StateBufferElement& state, const InputBufferElement& u, double Tpred);
	//void update(StateBufferElement& state, const MeasureBufferElement& z);
	MeasureHandler viconHanler;
	void publishEstimate(const StateBufferElement & estimate);

	// Timers
	Time intTime;
	double predTime;
	Timer saveTimer;

	// Eigen tools;
	EigenTools tools;

	template <typename _BufferElement>
	typename std::deque<_BufferElement, Eigen::aligned_allocator<_BufferElement> >::iterator searchInBuffer(std::deque<_BufferElement, Eigen::aligned_allocator<_BufferElement> >& buffer, double time);

};


template <typename _BufferElement>
typename std::deque<_BufferElement, Eigen::aligned_allocator<_BufferElement> >::iterator KalmanStateEstimator::searchInBuffer(std::deque<_BufferElement, Eigen::aligned_allocator<_BufferElement> >& buffer, double time) {
	typename std::deque<_BufferElement, Eigen::aligned_allocator<_BufferElement> >::iterator current;
	for(current = buffer.begin(); current <= buffer.end(); current++)
	{
		if (time >= current->timeStamp) break;
	}
	return current;
}

} /* namespace telekyb_state */
#endif /* KALMANSTATEESTIMATOR_HPP_ */
