/*
 * SSXStateEstimator.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#include <StateEstimators/SSXStateEstimator.hpp>
#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

// Declare
PLUGINLIB_EXPORT_CLASS( telekyb_state::SSXStateEstimator, TELEKYB_NAMESPACE::StateEstimator);

namespace telekyb_state {

SSXStateEstimatorOptions::SSXStateEstimatorOptions()
	: OptionContainer("SSXStateEstimator")
{
	tSSXSeTopicName = addOption<std::string>("tSSXSeTopicName","TopicName of SSX InternalState Sensor.","undef",true, true);
	tSSXPositionOffset = addOption<Eigen::Vector3d>("tSSXPositionOffset","Position Offset added to Sensor Input",
			Eigen::Vector3d::Zero(), false, true);
}

void SSXStateEstimator::initialize()
{
	nodeHandle = ROSModule::Instance().getMainNodeHandle();
}
void SSXStateEstimator::willBecomeActive()
{
	ssxSubscriber = nodeHandle.subscribe<telekyb_msgs::TKState>(
			options.tSSXSeTopicName->getValue(),1, &SSXStateEstimator::ssxCallback, this);
}
void SSXStateEstimator::willBecomeInActive()
{
	ssxSubscriber.shutdown();
}

void SSXStateEstimator::destroy()
{
// nothing to do here
}

std::string SSXStateEstimator::getName() const
{
//	return options.tSSXSeTopicName->getValue();
	return "SSXStateEstimator";
}

void SSXStateEstimator::ssxCallback(const telekyb_msgs::TKState::ConstPtr& stateMsg)
{
	// StateEstimatorController neest a telekyb::TKState
	TKState tStateMsg(*stateMsg);
	tStateMsg.position += options.tSSXPositionOffset->getValue();
	stateEstimatorController.activeStateCallBack(tStateMsg);
}

} // namespace
