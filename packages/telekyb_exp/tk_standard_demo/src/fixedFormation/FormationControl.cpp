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
	tRobotIDs = addOption< std::vector<int> >("tRobotIDs",
			"Specifies the ID of the Robots to connect to", std::vector<int>() , true, true);
	tJoystickTopic = addOption<std::string>("tJoystickTopic",
			"Joysticktopic to use (sensor_msgs::Joy)", "/TeleKyb/tJoy/joy", false, true);
	tVelocityInputTopic = addOption<std::string>("tVelocityInputTopic",
			"Velocity Input Topic to use (geometry_msgs::Vector3Stamped)", "FormationVelocityInput", false, true);
	tUseMKInterface = addOption<bool>("tUseMKInterface", "Set to true with MKInterface!", false, false, true);
	tUsesHumanInput = addOption< std::vector<int> >("tUsesHumanInput",
			"If the robot should use the human control input", std::vector<int>(), true, true);
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
	std::vector<int> robotIDs = options.tRobotIDs->getValue();
	formationElements.resize(robotIDs.size());


	std::vector<int> usesHumanInputInt = options.tUsesHumanInput->getValue();


	if (robotIDs.size() != usesHumanInputInt.size()) {
		ROS_ERROR("sizes of tRobotIDs and tUsesHumanInput don't match!");
		ros::shutdown();
	}


	Eigen::Vector3d center(0,0,-0.5);
//	double halfEdgeLength = 1.0;


	for (unsigned int i = 0; i < formationElements.size(); i++) {
		formationElements[i] = new FormationControlElement(robotIDs[i], options.tUseMKInterface->getValue());


		formationElements[i]->takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Vector3D(0.0,0.0,-1.0));
		formationElements[i]->takeOff.getOptionContainer().getOption("tTakeOffVertically").set<bool>(true);

		std::vector<int> neighborVector = robotIDs;
		neighborVector.erase(neighborVector.begin() + i); // remove oneself
		formationElements[i]->formation.getOptionContainer().getOption("tNeighbors").set(neighborVector);
		// tJoystickTopic
		formationElements[i]->formation.getOptionContainer().getOption("tJoystickTopic").set(options.tJoystickTopic->getValue());
		formationElements[i]->formation.getOptionContainer().getOption("tVelocityInputTopic").set(options.tVelocityInputTopic->getValue());

		bool usesHumanInput_i = (bool)usesHumanInputInt[i];
		
		
		//ROS_INFO("usesHumanInput_i = %i", usesHumanInput_i);
		formationElements[i]->formation.getOptionContainer().getOption("tUsesHumanInput").set(usesHumanInput_i);
		
		formationElements[i]->formation.getOptionContainer().getOption("tVelocityInputEnabled").set(usesHumanInput_i);
		
		formationElements[i]->formation.setParameterInitialized(true);
	}

//	for (unsigned int i = 0; i < formationElements.size()-1; i++) {
//		std::vector<int> neighborVector = robotIDs;
//		neighborVector.erase(neighborVector.size()-1);
//		std::vector<double> neighborDistanceVector(formationElements.size()-1,2.0);
//
//	}


	// lastly start Controller
	joySub = mainNodeHandle.subscribe(options.tJoystickTopic->getValue()
			, 10, &FormationControl::joystickCB, this);
}




void FormationControl::joystickCB(const sensor_msgs::Joy::ConstPtr& msg)
{
	// use button 2
	if (msg->buttons.size() < 9) {
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
	}

	if (msg->buttons[2]) {
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			formationElements[i]->liftland();
		}
	}

	if (msg->buttons[3]) {
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			//formationElements[i]->switchIntoFormation();
			formationElements[i]->bController->switchBehavior(formationElements[i]->formation);
		}
	}


//	if (msg->buttons[3]) {
//		bController->switchBehavior(calibrator);
//	}


	// what to do

}

