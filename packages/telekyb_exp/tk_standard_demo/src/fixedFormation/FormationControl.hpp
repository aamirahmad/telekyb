/*
 * FormationControl.hpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#ifndef FORMATIONCONTROL_HPP_
#define FORMATIONCONTROL_HPP_

#include <telekyb_base/Options.hpp>

#include <sensor_msgs/Joy.h>

#include "FormationControlElement.hpp"


using namespace telekyb;

class FormationControlOptions : public OptionContainer
{
public:
	Option< std::vector<int> >* tRobotIDs;
	Option<std::string>* tJoystickTopic;
	Option<std::string>* tVelocityInputTopic;
	Option<bool>* tUseMKInterface;
	Option< std::vector<int> >* tUsesHumanInput;
	FormationControlOptions();
};

class FormationControl {
protected:
	FormationControlOptions options;
	// Elements
	std::vector< FormationControlElement* > formationElements;

	// ROS
	ros::NodeHandle mainNodeHandle;
	ros::Subscriber joySub;

public:
	FormationControl();
	virtual ~FormationControl();

	// setup Behaviors
	void setupFormationControl();

	void joystickCB(const sensor_msgs::Joy::ConstPtr& msg);


};

#endif /* FORMATIONCONTROL_HPP_ */
