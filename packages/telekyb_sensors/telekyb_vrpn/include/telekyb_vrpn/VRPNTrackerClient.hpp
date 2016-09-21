/*
 * VRPNTrackerClient.hpp
 *
 *  Created on: Dec 11, 2011
 *      Author: mriedel
 */

#ifndef VRPNTRACKERCLIENT_HPP_
#define VRPNTRACKERCLIENT_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vrpn_Tracker.h>

#include <telekyb_defines/telekyb_defines.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <telekyb_vrpn/VRPNTrackerClientOptions.hpp>

namespace TELEKYB_NAMESPACE {

// Forward declaration
void VRPN_CALLBACK viconTracker_handleTracker(void* userData, const vrpn_TRACKERCB t );

class VRPNTrackerClient {
protected:
	ros::NodeHandle nodeHandle;
	ros::Publisher pub;
	geometry_msgs::PoseStamped poseStamped; // message
	std::string childFrameID;

	VRPNTrackerClientOptions& options; // Singleton. Initialized before!

	tf::TransformBroadcaster transBroadcaster;

	vrpn_Tracker_Remote* vrpnTracker;

	friend void VRPN_CALLBACK viconTracker_handleTracker(void* userData, const vrpn_TRACKERCB t );
public:
	VRPNTrackerClient(const std::string& objectName_);
	virtual ~VRPNTrackerClient();

	void spin();

	void handleVRPNTrackerCB(const vrpn_TRACKERCB t);
};

} /* namespace telekyb */
#endif /* VRPNTRACKERCLIENT_HPP_ */
