/*
 * SSXStateEstimator.h
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef SSXSTATEESTIMATOR_HPP_
#define SSXSTATEESTIMATOR_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <tk_state/StateEstimator.hpp>

// plugin stuff
#include <pluginlib/class_list_macros.h>

// ros
#include <ros/subscriber.h>


using namespace TELEKYB_NAMESPACE;

namespace telekyb_state {

class SSXStateEstimatorOptions : public OptionContainer {
public:
	Option<std::string>* tSSXSeTopicName;
	Option<Eigen::Vector3d>* tSSXPositionOffset;

	SSXStateEstimatorOptions();
};

class SSXStateEstimator : public StateEstimator {
protected:
	// Options
	SSXStateEstimatorOptions options;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Subscriber ssxSubscriber;


public:
	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

	void ssxCallback(const telekyb_msgs::TKState::ConstPtr& msg);


};

} //namespace

#endif /* SSXSTATEESTIMATOR_HPP_ */
