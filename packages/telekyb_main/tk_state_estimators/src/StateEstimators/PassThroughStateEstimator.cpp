/*
 * PassThroughStateEstimator.cpp
 *
 *  Created on: Mar 31, 2013
 *      Author: vgrabe
 */

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Messages.hpp>
#include <tk_state/StateEstimatorController.hpp>
#include "StateEstimators/PassThroughStateEstimator.hpp"

PLUGINLIB_EXPORT_CLASS( state_estimators_plugin::PassThroughStateEstimator, TELEKYB_NAMESPACE::StateEstimator);


namespace state_estimators_plugin {


PassThroughStateEstimatorOptions::PassThroughStateEstimatorOptions()
	: OptionContainer("PassThroughStateEstimator"){
	tStateTopicName = addOption<std::string>("tStateTopicName","Topic name of state topic to pass to controller.","undef",true,true);
}

PassThroughStateEstimator::PassThroughStateEstimator() {
}

PassThroughStateEstimator::~PassThroughStateEstimator() {
}

void PassThroughStateEstimator::initialize(){
	nodeHandle = telekyb::ROSModule::Instance().getMainNodeHandle();
}
void PassThroughStateEstimator::willBecomeActive(){
	stateSub = nodeHandle.subscribe<telekyb_msgs::TKState>(options.tStateTopicName->getValue(),1, &PassThroughStateEstimator::stateCallback, this);
}
void PassThroughStateEstimator::willBecomeInActive(){
	stateSub.shutdown();
}

void PassThroughStateEstimator::destroy(){
}
std::string PassThroughStateEstimator::getName() const{
	return "PassThroughStateEstimator";
}

void PassThroughStateEstimator::stateCallback(const telekyb_msgs::TKStateConstPtr& msg){
	stateEstimatorController.activeStateCallBack(telekyb::TKState(*msg));
}

} /* namespace */
