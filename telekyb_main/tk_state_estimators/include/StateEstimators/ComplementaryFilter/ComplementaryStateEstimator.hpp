/*
 * ComplementaryStateEstimator.hpp
 *
 *  Created on: Jun 19, 2012
 *      Author: rspica
 */

#ifndef COMPLEMENTARYSTATEESTIMATOR_HPP_
#define COMPLEMENTARYSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <tk_state/StateEstimator.hpp>

#include <telekyb_base/Filter/IIRFilter.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

#include "ComplementaryDataTypes.hpp"

#include <tk_draft_msgs/TKSmallImu.h>
#include <geometry_msgs/PoseStamped.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <telekyb_base/Time.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class ComplementaryStateEstimatorOptions : public OptionContainer {
public:

	Option<Eigen::Vector3d>* imuAccBias;
	Option<Eigen::Vector3d>* imuGyrBias;
	Option<Eigen::Vector3d>* vicPosition;
	Option<Eigen::Vector2d>* vicOrientation;
	Option<double>* predStep;
	Option<double>* pubStep;
	Option<double>* normalizationGain;
	Option<Eigen::Vector3d>* worldGravity;
	Option<std::string>* tViconSeTopicName;
	Option<std::string>* tImuSeTopicName;

	Option<bool>* tRepublishVicon;
	Option<std::string>* tViconRepubTopicName;

	Option<Eigen::Vector3d>* positionGain;
	Option<Eigen::Vector3d>* velocityGain;
	Option<Eigen::Vector3d>* rotationGain;
	Option<Eigen::Vector3d>* biasGain;

	Option<Eigen::Matrix3d>* matrixA;
	Option<Eigen::Vector3d>* vectorB;

	Option<double>* tOmegaFilterFreq;
	Option<double>* tOmegaSampleTime;
	ComplementaryStateEstimatorOptions();
};

class ComplementaryStateEstimator : public TELEKYB_NAMESPACE::StateEstimator {
public:

	ComplementaryStateEstimator();
    void inputCallback(const tk_draft_msgs::TKSmallImuConstPtr& msg);
	void measureCallback(const geometry_msgs::PoseStampedConstPtr& msg);

	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

protected:
	IIRFilter* omegaFilter[3];
	void initVelocityFilters();
	Eigen::Vector3d angVelFiltered;


	ros::NodeHandle nodeHandle;
	ros::Subscriber viconSub;
	ros::Subscriber imuSub;

	ros::Publisher biasPub;
	ros::Publisher viconRepublisher;


	bool spinning;
	bool isInitialized, receivedFirstInput, receivedFirstMeasure;

	boost::thread thread;
	boost::mutex threadMutex;

	boost::mutex imuMutex;
	boost::mutex viconMutex;
	telekyb::Timer rtTimer;
	telekyb::Timer pubTimer;

	void spin();
	/* Options */
	ComplementaryStateEstimatorOptions options;

	State internalState;

	DynamicSystem system;

	void initializationDone();



	/* Core functions */
	void prediction(State& state, double Tpred);
	void publishState();

};

} /* namespace telekyb_state */
#endif /* COMPLEMENTARYSTATEESTIMATOR_HPP_ */
