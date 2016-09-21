/*
 * TKHapticOutputToVector3Stamped.hpp
 *
 *  Created on: May 02, 2013
 *      Author: tnestmeyer
 */

#ifndef TKHAPTICOUTPUTTOVECTOR3STAMPED_HPP_
#define TKHAPTICOUTPUTTOVECTOR3STAMPED_HPP_

#include <telekyb_base/TeleKyb.hpp>
#include <telekyb_base/Options.hpp>

#include <geometry_msgs/Vector3Stamped.h>
#include <tk_haptics_msgs/TKHapticOutput.h>

using namespace telekyb;

class TKHapticOutputToVector3StampedOptions : public OptionContainer
{
public:
	Option<std::string>* tInputHaptic;
	Option<std::string>* tOutputVector;
	TKHapticOutputToVector3StampedOptions();
};

class TKHapticOutputToVector3Stamped {
private:
	TKHapticOutputToVector3StampedOptions options;

	ros::NodeHandle n;
	ros::Subscriber hapticSub;
	ros::Publisher vectorPub;

	void tkHapticCB(const tk_haptics_msgs::TKHapticOutput::ConstPtr& msg);

public:
	TKHapticOutputToVector3Stamped();
	virtual ~TKHapticOutputToVector3Stamped();
};

#endif /* TKHAPTICOUTPUTTOVECTOR3STAMPED_HPP_ */
