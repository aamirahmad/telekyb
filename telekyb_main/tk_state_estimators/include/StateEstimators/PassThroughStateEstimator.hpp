/*
 * PassThroughStateEstimator.hpp
 *
 *  Created on: Mar 31, 2013
 *      Author: vgrabe
 */

#ifndef PASSTHROUGHSTATEESTIMATOR_HPP_
#define PASSTHROUGHSTATEESTIMATOR_HPP_

#include <pluginlib/class_list_macros.h>

#include <telekyb_msgs/TKState.h>
#include <telekyb_base/Options.hpp>

#include "tk_state/StateEstimator.hpp"

using namespace TELEKYB_NAMESPACE;

namespace state_estimators_plugin {

class PassThroughStateEstimatorOptions : public telekyb::OptionContainer {
public:
	telekyb::Option<std::string>* tStateTopicName;
	PassThroughStateEstimatorOptions();
};

class PassThroughStateEstimator: public telekyb::StateEstimator {
public:
	PassThroughStateEstimator();
	virtual ~PassThroughStateEstimator();

	virtual void initialize();
	virtual void willBecomeActive();
	virtual void willBecomeInActive();
	virtual void destroy();

	virtual std::string getName() const;

	void stateCallback(const telekyb_msgs::TKStateConstPtr& msg);

protected:
	ros::NodeHandle nodeHandle;
	ros::Subscriber stateSub;
	PassThroughStateEstimatorOptions options;

};

} /* namespace sad */
#endif /* PASSTHROUGHSTATEESTIMATOR_HPP_ */
