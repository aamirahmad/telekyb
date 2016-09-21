/*
 * ROSBagMsgConvert.hpp
 *
 *  Created on: Dec 17, 2012
 *      Author: mriedel
 */

#ifndef ROSBAGMSGCONVERT_HPP_
#define ROSBAGMSGCONVERT_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <string>


// Messages
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/CompressedImage.h>
#include <tk_haptics_msgs/TKHapticOutput.h>
#include <telekyb_msgs/TKState.h>
#include <telekyb_msgs/TKTrajectory.h>

namespace TELEKYB_NAMESPACE {

namespace ROSBagMsgNS {

template<class _TYPE>
struct Convert {
	static std::string getCSVString(const _TYPE& msg) {
		return "Not implemented!";
	}
};


template<>
struct Convert< geometry_msgs::Vector3Stamped > {
	static std::string getCSVString(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
		std::stringstream ss;
		ss << msg->header.seq  << ",";
		ss << msg->header.stamp  << ",";
		ss << msg->vector.x << ",";
		ss << msg->vector.y << ",";
		ss << msg->vector.z;
		return ss.str();
	}
};

template<>
struct Convert< sensor_msgs::CompressedImage > {
	static std::string getCSVString(const sensor_msgs::CompressedImage::ConstPtr& msg) {
		std::stringstream ss;
		ss << msg->header.seq  << ",";
		ss << msg->header.stamp;
		return ss.str();
	}
};

template<>
struct Convert< tk_haptics_msgs::TKHapticOutput > {
	static std::string getCSVString(const tk_haptics_msgs::TKHapticOutput::ConstPtr& msg) {
		std::stringstream ss;
		ss << msg->header.seq  << ",";
		ss << msg->header.stamp  << ",";
		ss << msg->force.x << ",";
		ss << msg->force.y << ",";
		ss << msg->force.z << ",";
		ss << msg->frequency << ",";
		ss << msg->pose.position.x << ",";
		ss << msg->pose.position.y << ",";
		ss << msg->pose.position.z << ",";
		ss << msg->pose.orientation.w << ",";
		ss << msg->pose.orientation.x << ",";
		ss << msg->pose.orientation.y << ",";
		ss << msg->pose.orientation.z << ",";
		ss << msg->linear.x << ",";
		ss << msg->linear.y << ",";
		ss << msg->linear.z << ",";
		ss << msg->primaryButton;
		return ss.str();
	}
};

template<>
struct Convert< telekyb_msgs::TKTrajectory > {
	static std::string getCSVString(const telekyb_msgs::TKTrajectory::ConstPtr& msg) {
		std::stringstream ss;
		ss << msg->header.seq  << ",";
		ss << msg->header.stamp  << ",";
		ss << msg->position.x << ",";
		ss << msg->position.y << ",";
		ss << msg->position.z << ",";
		ss << msg->velocity.x << ",";
		ss << msg->velocity.y << ",";
		ss << msg->velocity.z << ",";
		ss << msg->acceleration.x << ",";
		ss << msg->acceleration.y << ",";
		ss << msg->acceleration.z << ",";
		ss << msg->jerk.x << ",";
		ss << msg->jerk.y << ",";
		ss << msg->jerk.z << ",";
		ss << msg->snap.x << ",";
		ss << msg->snap.y << ",";
		ss << msg->snap.z << ",";
		ss << (int)msg->xAxisCtrlType << ",";
		ss << (int)msg->yAxisCtrlType << ",";
		ss << (int)msg->zAxisCtrlType << ",";
		ss << msg->yawAngle << ",";
		ss << msg->yawRate << ",";
		ss << msg->yawAcceleration << ",";
		ss << (int)msg->yawCtrlType;
		return ss.str();
	}
};

template<>
struct Convert< telekyb_msgs::TKState > {
	static std::string getCSVString(const telekyb_msgs::TKState::ConstPtr& msg) {
		std::stringstream ss;
		ss << msg->header.seq  << ",";
		ss << msg->header.stamp  << ",";
		ss << msg->pose.position.x << ",";
		ss << msg->pose.position.y << ",";
		ss << msg->pose.position.z << ",";
		ss << msg->pose.orientation.w  << ",";
		ss << msg->pose.orientation.x << ",";
		ss << msg->pose.orientation.y << ",";
		ss << msg->pose.orientation.z << ",";
		ss << msg->twist.linear.x << ",";
		ss << msg->twist.linear.y << ",";
		ss << msg->twist.linear.z << ",";
		ss << msg->twist.angular.x << ",";
		ss << msg->twist.angular.y << ",";
		ss << msg->twist.angular.z;
		return ss.str();
	}
};




}

}



#endif /* ROSBAGMSGCONVERT_HPP_ */
