/*
 * VRPNTrackerServer.cpp
 *
 *  Created on: Jan 9, 2012
 *      Author: mriedel
 */

#include <telekyb_vrpn/VRPNTrackerServer.hpp>

#include <telekyb_base/ROS/ROSModule.hpp>

#include <boost/filesystem.hpp>

namespace TELEKYB_NAMESPACE {

VRPNTrackerServer::VRPNTrackerServer(const std::string& topicName_, vrpn_Connection* c_)
	: c(c_), nodeHandle(ROSModule::Instance().getMainNodeHandle()), options(VRPNTrackerServerOptions::Instance())
{
	std::string vrpnName = boost::filesystem::path(topicName_).filename().string();

	vts = new vrpn_Tracker_Server(vrpnName.c_str(),c);

	position[0] = 0.0;
	position[1] = 0.0;
	position[2] = 0.0;

	rotation[0] = 0.0;
	rotation[1] = 0.0;
	rotation[2] = 0.0;
	rotation[3] = 0.0;

	ROS_INFO("Set up VRPN Tracker Server with Service %s listening on port %d",
			vrpnName.c_str(), vrpn_DEFAULT_LISTEN_PORT_NO);


	sub = nodeHandle.subscribe(topicName_, 10, &VRPNTrackerServer::transformStampedCallback, this);
}

VRPNTrackerServer::~VRPNTrackerServer() {
	delete vts;
}

void VRPNTrackerServer::setPose(const Position3D& rosPosition, const Quaternion& rosRotation)
{
	position[0] = rosPosition(0);
	position[1] = rosPosition(1);
	position[2] = rosPosition(2);

	rotation[0] = rosRotation.x();
	rotation[1] = rosRotation.y();
	rotation[2] = rosRotation.z();
	rotation[3] = rosRotation.w();
}

// send current values
void VRPNTrackerServer::sendPose()
{
//	std::stringstream ss;
//	ss << "(" << position[0] << "," << position[1] << "," << position[2] << ")";
//	ss << "(" << rotation[0] << "," << rotation[1] << "," << rotation[2] << "," << rotation[3] << ")";
//	ROS_INFO("Send: [%s]", ss.str().c_str());

	// send
	gettimeofday(&packageTime, NULL);
	vts->report_pose(0,packageTime,position,rotation);
}

void VRPNTrackerServer::transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	Position3D rosPositon(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z);
	Quaternion rosRotation(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);

	// Transform from NED back to Vicon
	rosPositon = options.tViconToNEDMatrix->getValue() * rosPositon;
	rosRotation.vec() = options.tViconToNEDMatrix->getValue() * rosRotation.vec();

	setPose(rosPositon, rosRotation);
	sendPose();
}

void VRPNTrackerServer::spin()
{
	vts->mainloop();
	c->mainloop();
}

}
