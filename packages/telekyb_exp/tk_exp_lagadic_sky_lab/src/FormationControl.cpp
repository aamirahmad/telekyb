/*
 * FormationControl.cpp
 *
 *  Created on: Nov 15, 2011
 *      Author: mriedel
 */

#include "FormationControl.hpp"

#include <telekyb_base/ROS.hpp>
#include <telekyb_base/Time.hpp>

#include <telekyb_base/Spaces.hpp>

// Options
FormationControlOptions::FormationControlOptions()
	: OptionContainer("FormationControlOptions")
{
	robotIDs = addOption< std::vector<int> >("robotIDs",
			"Specifies the ID of the Robots to connect to", std::vector<int>() , true, true);
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
}

FormationControl::FormationControl()
	: mainNodeHandle( ROSModule::Instance().getMainNodeHandle() )
{
	setupFormationControl();
}

FormationControl::~FormationControl()
{
	for (unsigned int i = 0; i < formationElements.size(); i++) {
		delete formationElements[i];
	}
}

void FormationControl::setupFormationControl()
{
	// create elements
	std::vector<int> robotIDs = options.robotIDs->getValue();
	// The following instruction works only for 2 quadrotors. I have to extend it.
	ROS_WARN_STREAM("robotIDs: " << robotIDs[0] << ", " << robotIDs[1]);
	//ROS_INFO();
	formationElements.resize(robotIDs.size());
	for (unsigned int i = 0; i < formationElements.size(); i++) {
		formationElements[i] = new FormationControlElement(robotIDs[i], options.tUseMKInterface->getValue());

		/*
		// setup neighbor and distances
		telekyb_interface::Behavior formationBehavior = formationElements[i]->getFormationBehavior();

		std::vector<int> neighborVector = robotIDs;
		neighborVector.erase(neighborVector.begin() + i);
		std::vector<double> neighborDistances(neighborVector.size(),1.5);
		formationBehavior.getOptionContainer().getOption("tNeighbors").set(neighborVector);
		formationBehavior.getOptionContainer().getOption("tNeighborDistances").set(neighborDistances);
		// tJoystickTopic
		formationBehavior.getOptionContainer().getOption("tJoystickTopic").set("/TeleKyb/tJoy/joy_for");

		formationBehavior.setParameterInitialized(true);
		*/
	}

		// lastly start Controller only for quadrotor 0.
		joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue()
				, 10, &FormationControl::joystickCB, this);
}




void FormationControl::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// use back (7) + start (8) button of the JoystickF310
	// Test new Behavior fschiano
	if (msg->buttons[7] && msg->buttons[8]) {
		ROS_WARN("Fschiano Activated a behavior");

		//			linearFly.setParameterInitialized(true);
		//			flyto1.setParameterInitialized(true);
		//			flyto2.setParameterInitialized(true);
		//			flyto3.setParameterInitialized(true);

		//			takeOff.setNextBehavior(linearFly);
		//			linearFly.setNextBehavior(flyto1);
		//			flyto1.setNextBehavior(flyto2);
		//			flyto2.setNextBehavior(flyto3);
		//			//flyto3.setNextBehavior(land);

	}

	// Analog DX button pressed
	if (msg->buttons[10]) {
		ROS_WARN("Analog DX button pressed!");
	}


	// use button 2
	if (msg->buttons.size() < 6) {
		ROS_ERROR("Joytick does not publish enough buttons.");
		return;
	}

	// Emergency
	if (msg->buttons[6]) {
		ROS_WARN("Emergency Button pressed! Landing all");
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->mkSetEmergency();
		}
	}


	if (msg->buttons[0]) {
		// this only works on ground.
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->mkToggleMotors();
		}
		ROS_INFO("Toggling motors");

	}

	if (msg->buttons[2]) {	// X
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->liftland();
		}
	}

	if (msg->buttons[3]) {	// Y
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->switchIntoFormation();
		}
	}

	if (msg->buttons[4]) {	// A
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->switchIntoNormalBrake();
		}
	}

//	if (msg->buttons[4]) {	// A
//		for (unsigned int i = 0; i < formationElements.size(); i++) {
//			formationElements[i]->switchIntoFlyBack();
//		}
//	}




}

