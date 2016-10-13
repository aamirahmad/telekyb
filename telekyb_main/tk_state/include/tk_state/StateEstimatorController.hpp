/*
 * StateEstimatorController.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef STATEESTIMATORCONTROLLER_HPP_
#define STATEESTIMATORCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Messages.hpp>

#include <tk_state/StateEstimator.hpp>
#include <tk_state/StateEstimatorControllerOptions.hpp>

// Time
#include <telekyb_base/Time.hpp>

// Class Loading
#include <pluginlib/class_loader.h>

// Locking
#include <boost/thread/mutex.hpp>

//TF
// The TF class provides an easy way to publish coordinate frame transform information.
// It will handle all the messaging and stuffing of messages.
// And the function prototypes lay out all the necessary data needed for each message.
#include <tf/transform_broadcaster.h>
// ros
#include <ros/console.h>

namespace TELEKYB_NAMESPACE {

class StateEstimatorController {
private:
	// Singleton Stuff
	static StateEstimatorController* instance;

	StateEstimatorController();
	virtual ~StateEstimatorController();

	StateEstimatorController(const StateEstimatorController &);
	StateEstimatorController& operator=(const StateEstimatorController &);


protected:
	// Options
	StateEstimatorControllerOptions options;

	// ClassLoader
	pluginlib::ClassLoader<StateEstimator> seLoader;

	// active State
	boost::shared_ptr<telekyb::StateEstimator> activeStateEstimator;

	// TKState lastState
	TKState lastState;
	mutable boost::mutex lastStateMutex;
	bool recvFirstState;

	// ROS
	ros::NodeHandle nodeHandle;
	ros::Publisher tkStatePublisher;


	// initialize. This does further setup AFTER the object has been created.
	// This is needed, since Objects that reference the Behavior Controller can only be created after it returns from the constuctor
	// (Singleton Issue).
	//** This is like a Constructor. It's called by the Singleton creator DIRECTLY AFTER the actual constructor. **/
	void initialize();

	// TF
	tf::TransformBroadcaster tfBroadCaster;
	void publishTransform(const TKState& tStateMsg);

	ros::Publisher transformStampedPub;
	void publishTransformStamped(const TKState& tStateMsg);


public:
	// Callback from activeState
	void activeStateCallBack(const TKState& tStateMsg);
	const boost::shared_ptr<telekyb::StateEstimator> getActiveStateEstimator() const;

	std::string getSePublisherTopic() const;

	// block till timeout or first state becomes available
	bool waitForFirstState(Time timeout) const;

	// Returns last Received State.
	TKState getLastState() const;

	const ros::NodeHandle& getSensorNodeHandle() const;


	// Singleton Stuff
	static StateEstimatorController& Instance();
	static const StateEstimatorController* InstancePtr();
	static bool HasInstance();
	static void ShutDownInstance();

	// For Eigen Alignment (especially on 32 bit machines)
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}

#endif /* STATEESTIMATORCONTROLLER_HPP_ */
