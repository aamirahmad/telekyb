/*
 * StateEstimatorController.cpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#include <tk_state/StateEstimatorController.hpp>

#include <telekyb_base/ROS.hpp>

namespace TELEKYB_NAMESPACE {

// Init static class Loader
//pluginlib::ClassLoader<StateEstimator> StateEstimatorController::seLoader(
//		"tk_state", std::string( TELEKYB_NAMESPACE_STRING ) + "::StateEstimator" );

StateEstimatorController::StateEstimatorController()
	: seLoader( "tk_state", std::string( TELEKYB_NAMESPACE_STRING ) + "::StateEstimator" ),
// 	  activeStateEstimator( NULL ),
	  recvFirstState( false ),
	  nodeHandle( ROSModule::Instance().getMainNodeHandle(), TELEKYB_SENSOR_NODESUFFIX )
{
	tkStatePublisher = nodeHandle.advertise<telekyb_msgs::TKState>(options.tPublisherTopic->getValue(), 1);
	transformStampedPub = nodeHandle.advertise<geometry_msgs::TransformStamped>(options.tTransformStampedTopic->getValue(), 10);

}

StateEstimatorController::~StateEstimatorController()
{
	if (activeStateEstimator) {
		activeStateEstimator->willBecomeInActive();
		activeStateEstimator->destroy();
// 		delete activeStateEstimator;
	}
}

void StateEstimatorController::initialize()
{
	try {
		activeStateEstimator = seLoader.createInstance(options.tPluginLookupName->getValue());
		// Currently RunTime Switch is not supported. This has to be changed then.
		activeStateEstimator->initialize();
		activeStateEstimator->willBecomeActive();

	} catch (pluginlib::PluginlibException& e) {
		ROS_FATAL("StateEstimator Plugin %s failed to load: %s", options.tPluginLookupName->getValue().c_str(), e.what());
		//ROS_BREAK();
		ros::shutdown();
	}
}

void StateEstimatorController::publishTransform(const TKState& tStateMsg)
{
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(tStateMsg.position(0),tStateMsg.position(1),tStateMsg.position(2)));
	transform.setRotation( tf::Quaternion(
			tStateMsg.orientation.x(),
			tStateMsg.orientation.y(),
			tStateMsg.orientation.z(),
			tStateMsg.orientation.w()));

	tfBroadCaster.sendTransform(
			tf::StampedTransform(
					transform,
					tStateMsg.time.toRosTime(),
					options.tTransformParentID->getValue(),
					activeStateEstimator->getName()) // activeStateEstimator->getName()
	);
}

void StateEstimatorController::publishTransformStamped(const TKState& tStateMsg)
{
	geometry_msgs::TransformStamped msg;
	msg.header.frame_id = activeStateEstimator->getName();
	msg.header.stamp = tStateMsg.time.toRosTime();
	msg.child_frame_id = options.tTransformParentID->getValue();

	msg.transform.translation.x = tStateMsg.position(0);
	msg.transform.translation.y = tStateMsg.position(1);
	msg.transform.translation.z = tStateMsg.position(2);

	msg.transform.rotation.w = tStateMsg.orientation.w();
	msg.transform.rotation.x = tStateMsg.orientation.x();
	msg.transform.rotation.y = tStateMsg.orientation.y();
	msg.transform.rotation.z = tStateMsg.orientation.z();
	transformStampedPub.publish(msg);
}

void StateEstimatorController::activeStateCallBack(const TKState& tStateMsg)
{
	//ROS_INFO("activeStateCallBack called!");
	// high Priority. Publish
	telekyb_msgs::TKStatePtr tmStateMsg(new telekyb_msgs::TKState);
	tStateMsg.toTKStateMsg(*tmStateMsg);
	tkStatePublisher.publish(tmStateMsg);

	// lower prio. Save State
	boost::mutex::scoped_lock(lastStateMutex);
	lastState = tStateMsg;

	// Ok this is always written. Maybe there is a nice way to improve this.
	recvFirstState = true;

	if (options.tPublishRosTransform->getValue()) {
		publishTransform(tStateMsg);
	}

	if (options.tPublishRosTransformStamped->getValue()) {
		publishTransformStamped(tStateMsg);
	}
}

const boost::shared_ptr<telekyb::StateEstimator> StateEstimatorController::getActiveStateEstimator() const
{
	return activeStateEstimator;
}

std::string StateEstimatorController::getSePublisherTopic() const
{
	return tkStatePublisher.getTopic();
}

bool StateEstimatorController::waitForFirstState(Time timeout) const
{
	Time rate(0.1); // 1/10 s
	while(ros::ok() && timeout.toDSec() > 0.0) {
		//ROS_INFO_STREAM("Timeout: " << timeout.toString());
		// check
		if ( recvFirstState ) {
			// success
			return true;
		}

		timeout -= rate;
		// sleep
		rate.sleep();
	}

	// did not get first state in timeout
	return false;
}

TKState StateEstimatorController::getLastState() const
{
	boost::mutex::scoped_lock(lastStateMutex);
	return lastState;
}

const ros::NodeHandle& StateEstimatorController::getSensorNodeHandle() const
{
	return nodeHandle;
}


//---------------
// Singleton Stuff
StateEstimatorController* StateEstimatorController::instance = NULL;

StateEstimatorController& StateEstimatorController::Instance() {
	if (!instance) {
		instance = new StateEstimatorController();
		instance->initialize();
	}
	return *instance;
}

const StateEstimatorController* StateEstimatorController::InstancePtr() {
	if (!instance) {
		instance = new StateEstimatorController();
		instance->initialize();
	}

	return instance;
}

bool StateEstimatorController::HasInstance()
{
	return (instance != NULL);
}

void StateEstimatorController::ShutDownInstance() {
	if (instance) {
		delete instance;
	}

	instance = NULL;
}


}
