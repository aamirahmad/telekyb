/*
 * TrajectoryProcessorController.hpp
 *
 *  Created on: Dec 13, 2011
 *      Author: mriedel
 */

#ifndef TRAJECTORYPROCESSORCONTROLLER_HPP_
#define TRAJECTORYPROCESSORCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/ClassInterfaces.hpp>

#include <tk_trajprocessor/TrajectoryProcessorControllerOptions.hpp>
#include <tk_trajprocessor/TrajectoryModuleContainer.hpp>

#include <tk_state/StateEstimatorController.hpp>

#include <boost/thread/mutex.hpp>

namespace TELEKYB_NAMESPACE {

class TrajectoryController;

class TrajectoryProcessorController {
private:
	// Singleton Stuff
	static TrajectoryProcessorController* instance;

	TrajectoryProcessorController();
	virtual ~TrajectoryProcessorController();

	TrajectoryProcessorController(const TrajectoryProcessorController &);
	TrajectoryProcessorController& operator=(const TrajectoryProcessorController &);


protected:
	//** This is like a Constructor. It's called by the Singleton creator DIRECTLY AFTER the actual constructor. **/
	void initialize();

	// Options
	TrajectoryProcessorControllerOptions options;

	// BehaviorNodeHandle
	ros::NodeHandle trajProcessorNodeHandle;

	// State Subscription
	ros::Subscriber tStateSub;

	// StateEstimatorController Instance!
	StateEstimatorController& seController;

	// where to send the TKTrajInput
	TrajectoryController& trajController;

	// Callback
	TKState lastState;
	boost::mutex lastStateMutex;
	void tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg);


	// Modules
	TrajectoryModuleContainer trajModuleContainer;

	// Active?
	bool active;
	BehaviorSwitcher* switcher;

public:
	void setActive(BehaviorSwitcher* switcher_);
	void setInActive();
	bool isActive() const;

	BehaviorSwitcher* getBehaviorSwitcher() const;

	// TKTrajInputReceiver Delegate
	void trajInputStep(const TKTrajectory& input);


	// Singleton Stuff
	static TrajectoryProcessorController& Instance();
	static const TrajectoryProcessorController* InstancePtr();
	static bool HasInstance();
	static void ShutDownInstance();

	// For Eigen Alignment (especially on 32 bit machines)
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} /* namespace telekyb */
#endif /* TRAJECTORYPROCESSORCONTROLLER_HPP_ */
