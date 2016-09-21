/*
 * VRPNTrackerClient.cpp
 *
 *  Created on: Dec 11, 2011
 *      Author: mriedel
 */

#include <telekyb_vrpn/VRPNTrackerClient.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

namespace TELEKYB_NAMESPACE {

void VRPN_CALLBACK viconTracker_handleTracker(void* userData, const vrpn_TRACKERCB t ) {
	VRPNTrackerClient* client = (VRPNTrackerClient *) userData;
	client->handleVRPNTrackerCB(t);
}


VRPNTrackerClient::VRPNTrackerClient(const std::string& objectName_)
	: nodeHandle(ROSModule::Instance().getMainNodeHandle()), options(VRPNTrackerClientOptions::Instance())
{
	std::string vrpnObjectName = objectName_ + "@" + options.tVRPNHostname->getValue();
	pub = nodeHandle.advertise<geometry_msgs::PoseStamped>(objectName_, 10);

	childFrameID = objectName_;
	poseStamped.header.frame_id = options.tChildFrameID->getValue();

	// Tracker
	vrpnTracker = new vrpn_Tracker_Remote(vrpnObjectName.c_str());
	vrpnTracker->register_change_handler(this, viconTracker_handleTracker);
}

VRPNTrackerClient::~VRPNTrackerClient()
{
	vrpnTracker->unregister_change_handler(this, viconTracker_handleTracker);
	delete vrpnTracker;
}

void VRPNTrackerClient::spin()
{
	vrpnTracker->mainloop();
}

void VRPNTrackerClient::handleVRPNTrackerCB(const vrpn_TRACKERCB t)
{
	Eigen::Vector3d pose(t.pos[0], t.pos[1], t.pos[2]);
	Eigen::Vector3d rotPose = options.tVRPNRotationMatrix->getValue()*pose;
	poseStamped.header.stamp = ros::Time(t.msg_time.tv_sec, t.msg_time.tv_usec*1000);
	poseStamped.pose.position.x = rotPose[0];
	poseStamped.pose.position.y = rotPose[1];
	poseStamped.pose.position.z = rotPose[2];

	Eigen::Vector3d ori(t.quat[0], t.quat[1], t.quat[2]);
	Eigen::Vector3d rotOri = options.tVRPNRotationMatrix->getValue()*ori;
	poseStamped.pose.orientation.x = rotOri[0];
	poseStamped.pose.orientation.y = rotOri[1];
	poseStamped.pose.orientation.z = rotOri[2];
	poseStamped.pose.orientation.w = t.quat[3];

	pub.publish(poseStamped);

	if (options.tEnableTF->getValue()) {

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z));
		transform.setRotation(tf::Quaternion(poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w));

		tf::StampedTransform tfStampedTransform(transform, poseStamped.header.stamp, poseStamped.header.frame_id, childFrameID);
		transBroadcaster.sendTransform(tfStampedTransform);
	}
}

} /* namespace telekyb */
