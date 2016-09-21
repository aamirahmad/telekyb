/*
 * TKHapticOutputToVector3Stamped.cpp
 *
 *  Created on: May 02, 2013
 *      Author: tnestmeyer
 */

#include "TKHapticOutputToVector3Stamped.hpp"

#include <ros/ros.h>

#include <telekyb_base/ROS/ROSModule.hpp>

TKHapticOutputToVector3StampedOptions::TKHapticOutputToVector3StampedOptions()
	: OptionContainer("Haptic2Vector")
{
	tInputHaptic = addOption<std::string>("tInputHaptic", "Define Input TKHapticOutput", "undef", true, true);
	tOutputVector = addOption<std::string>("tOutputVector", "Define Output Vector3Stamped", "undef", true, true);
}

TKHapticOutputToVector3Stamped::TKHapticOutputToVector3Stamped() {
	n = ROSModule::Instance().getMainNodeHandle();
	hapticSub = n.subscribe(options.tInputHaptic->getValue(), 1, &TKHapticOutputToVector3Stamped::tkHapticCB, this);
	vectorPub = n.advertise<geometry_msgs::Vector3Stamped>(options.tOutputVector->getValue(), 1);
}

TKHapticOutputToVector3Stamped::~TKHapticOutputToVector3Stamped() {
	// TODO Auto-generated destructor stub
}

void TKHapticOutputToVector3Stamped::tkHapticCB(const tk_haptics_msgs::TKHapticOutput::ConstPtr& msg)
{
	geometry_msgs::Vector3Stamped vectorMsg;
	vectorMsg.header = msg->header;
	vectorMsg.header.frame_id = "world";
	vectorMsg.vector.x = msg->pose.position.x;
	vectorMsg.vector.y = msg->pose.position.y;
	vectorMsg.vector.z = msg->pose.position.z;
	vectorPub.publish(vectorMsg);
}


int main(int argc, char **argv) {
	TeleKyb::init(argc,argv, "haptic2vector", ros::init_options::AnonymousName);

	TKHapticOutputToVector3Stamped *o = new TKHapticOutputToVector3Stamped();
	ros::waitForShutdown();
	delete o;

	TeleKyb::shutdown();
}
