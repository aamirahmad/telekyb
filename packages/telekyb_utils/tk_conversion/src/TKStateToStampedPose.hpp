/*
 * TKStateToStampedPose.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: mriedel
 */

#ifndef TKSTATETOSTAMPEDPOSE_HPP_
#define TKSTATETOSTAMPEDPOSE_HPP_

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Options.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <telekyb_msgs/TKState.h>

using namespace telekyb;

class TKStateToStampedPoseOptions : public OptionContainer
{
public:
	Option<std::string>* tInputState;
	Option<std::string>* tOutputPose;
	TKStateToStampedPoseOptions();
};

class TKStateToStampedPose {
private:
	TKStateToStampedPoseOptions options;

	ros::NodeHandle n;
	ros::Subscriber stateSub;
	ros::Publisher posePub;

	void tkStateCB(const telekyb_msgs::TKState::ConstPtr& msg);

public:
	TKStateToStampedPose();
	virtual ~TKStateToStampedPose();
};

#endif /* TKSTATETOSTAMPEDPOSE_HPP_ */
