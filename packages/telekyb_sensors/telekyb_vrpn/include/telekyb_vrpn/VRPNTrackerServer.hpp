/*
 * VRPNTrackerServer.hpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#ifndef VRPNTRACKERSERVER_HPP_
#define VRPNTRACKERSERVER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_vrpn/VRPNTrackerServerOptions.hpp>
#include <geometry_msgs/TransformStamped.h>


#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

#include <telekyb_base/Spaces/R3.hpp>

namespace TELEKYB_NAMESPACE {

class VRPNTrackerServer {
protected:
	vrpn_Connection* c;
	vrpn_Tracker_Server* vts;

	vrpn_float64 position[3];
	vrpn_float64 rotation[4];
	timeval packageTime;

	ros::NodeHandle nodeHandle;
	ros::Subscriber sub;
	VRPNTrackerServerOptions& options;

	void transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
	void setPose(const Position3D& rosPosition, const Quaternion& rosRotation);
	void sendPose();

public:
	VRPNTrackerServer(const std::string& topicName_, vrpn_Connection* c_);
	virtual ~VRPNTrackerServer();

	void spin();


};

}

#endif /* VRPNTRACKERSERVER_HPP_ */
