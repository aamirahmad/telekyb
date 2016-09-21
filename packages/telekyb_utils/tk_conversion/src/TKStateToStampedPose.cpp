/*
 * TKStateToStampedPose.cpp
 *
 *  Created on: Feb 15, 2012
 *      Author: mriedel
 */

#include "TKStateToStampedPose.hpp"

#include <ros/ros.h>

#include <telekyb_base/ROS/ROSModule.hpp>

TKStateToStampedPoseOptions::TKStateToStampedPoseOptions()
	: OptionContainer("State2Pose")
{
	tInputState = addOption<std::string>("tInputState", "Define Input TKState", "undef", true, true);
	tOutputPose = addOption<std::string>("tOutputPose", "Define Output PoseStamped", "undef", true, true);
}

TKStateToStampedPose::TKStateToStampedPose() {
	n = ROSModule::Instance().getMainNodeHandle();
	stateSub = n.subscribe(options.tInputState->getValue(), 1, &TKStateToStampedPose::tkStateCB, this);
	posePub = n.advertise<geometry_msgs::PoseStamped>(options.tOutputPose->getValue(), 1);
}

TKStateToStampedPose::~TKStateToStampedPose() {
	// TODO Auto-generated destructor stub
}

void TKStateToStampedPose::tkStateCB(const telekyb_msgs::TKState::ConstPtr& msg)
{
	geometry_msgs::PoseStamped poseMsg;
	poseMsg.header = msg->header;
	poseMsg.header.frame_id = "world";
	poseMsg.pose = msg->pose;
	posePub.publish(poseMsg);
}


int main(int argc, char **argv) {
	TeleKyb::init(argc,argv, "state2pose", ros::init_options::AnonymousName);

	TKStateToStampedPose *o = new TKStateToStampedPose();
	ros::waitForShutdown();
	delete o;

	TeleKyb::shutdown();
}
