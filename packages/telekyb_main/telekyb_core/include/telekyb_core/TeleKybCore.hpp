/*
 * TeleKybCore.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef TELEKYBCORE_HPP_
#define TELEKYBCORE_HPP_

#include <dynamic_reconfigure/server.h>
#include <telekyb_core/telekyb_coreConfig.h>

#include <telekyb_defines/telekyb_defines.hpp>
// Options
#include <telekyb_core/TeleKybCoreOptions.hpp>
// Behavior
#include <tk_behavior/BehaviorController.hpp>
// State
#include <tk_state/StateEstimatorController.hpp>
// TrajController
#include <tk_trajctrl/TrajectoryController.hpp>
// TrajProcessor
#include <tk_trajprocessor/TrajectoryProcessorController.hpp>

// Interface
#include <telekyb_core/TeleKybCoreInterface.hpp>

// ros
#include <ros/ros.h>


namespace TELEKYB_NAMESPACE {

class TeleKybCore {
protected:
	//options
	TeleKybCoreOptions options;

	// ROS
	ros::NodeHandle nodeHandle;


	// Interface
	TeleKybCoreInterface* interface;

public:
	TeleKybCore();
	virtual ~TeleKybCore();
};

}

#endif /* TELEKYBCORE_HPP_ */
