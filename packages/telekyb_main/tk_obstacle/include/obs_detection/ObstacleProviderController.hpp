/*
 * ObstacleProviderController.hpp
 *
 *  Created on: Dec 14, 2011
 *      Author: mriedel
 */

#ifndef OBSTACLEPROVIDERCONTROLLER_HPP_
#define OBSTACLEPROVIDERCONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <obs_detection/ObstacleProviderControllerOptions.hpp>
#include <obs_detection/ObstacleProviderContainer.hpp>

#include <ros/ros.h>

namespace TELEKYB_NAMESPACE {

class ObstacleProviderController {
protected:
	ObstacleProviderControllerOptions options;
	// Container
	ObstacleProviderContainer obsContainer;

	// ROS
	// NodeHandle
	ros::NodeHandle nodeHandle;
	// State Subscription
	ros::Subscriber tStateSub;
	// PointArray Publisher
	ros::Publisher tObsPointsPub;

	// Callback
	TKState lastState;
	boost::mutex lastStateMutex;
	void tkStateCB(const telekyb_msgs::TKState::ConstPtr& tkStateMsg);
	bool recvFirstTKState;

	// Helper Functions
	void loadObstacleProviders();
	void obstacleProviderStep();

	std::vector< Position3D > allObstaclePositions;

public:
	ObstacleProviderController();
	virtual ~ObstacleProviderController();

	void spin();
};

} /* namespace telekyb */
#endif /* OBSTACLEPROVIDERCONTROLLER_HPP_ */
