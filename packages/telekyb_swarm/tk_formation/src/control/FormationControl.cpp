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


	Eigen::Vector3d center(0,0,-0.5);
//	double halfEdgeLength = 1.0;

	// init random
	srand( time(NULL) );
	for (unsigned int i = 0; i < formationElements.size(); i++) {
		formationElements[i] = new FormationControlElement(robotIDs[i], options.tUseMKInterface->getValue());

		// setup neighbor an distances
		//telekyb_interface::Behavior formationBehavior = formationElements[i]->formation;

//		Eigen::Vector3d qcPos;
//		for (int j = 0; j < 3; ++j) {
//			int toggle = i & (1 << j);
//			toggle = toggle ? 1 : -1;
//			qcPos(j) = center(j) + toggle*halfEdgeLength;
//		}
//		ROS_WARN_STREAM("Takeoff Position: " << std::endl << qcPos);
//
//
		double liftOffVariance = (rand() % 101) / 100.0; // between 0 and 1
		formationElements[i]->takeOff.getOptionContainer().getOption("tTakeOffDestination").set(Vector3D(0.0,0.0,-0.5-liftOffVariance));
		formationElements[i]->takeOff.getOptionContainer().getOption("tTakeOffVertically").set<bool>(true);

		std::vector<int> neighborVector = robotIDs;
		neighborVector.erase(neighborVector.begin() + i); // remove oneself
		formationElements[i]->formation4.getOptionContainer().getOption("tNeighbors").set(neighborVector);
		// tJoystickTopic
		formationElements[i]->formation4.getOptionContainer().getOption("tJoystickTopic").set(options.tJoystickTopic->getValue());
		formationElements[i]->formation4.getOptionContainer().getOption("tVelocityInputTopic").set(options.tVelocityInputTopic->getValue());
		formationElements[i]->formation4.setParameterInitialized(true);


		// Tetraeder
		std::vector<double> tetraederDistanceVector(neighborVector.size(),2.5);
		// Also configure ReformationBehavior
		formationElements[i]->tetraeder.getOptionContainer().getOption("tNeighbors").set(neighborVector);
		formationElements[i]->tetraeder.getOptionContainer().getOption("tNeighborDistances").set(tetraederDistanceVector);
		formationElements[i]->tetraeder.setParameterInitialized(true);


		// Square
		std::vector<double> squareDistanceVector(neighborVector.size(),2.0);
		squareDistanceVector[(i+1)%3] = sqrt(2.0*2.0 + 2.0*2.0);

		formationElements[i]->square.getOptionContainer().getOption("tNeighbors").set(neighborVector);
		formationElements[i]->square.getOptionContainer().getOption("tNeighborDistances").set(squareDistanceVector);
		formationElements[i]->square.setParameterInitialized(true);


		// automatic change
		formationElements[i]->tetraeder.setNextBehavior(formationElements[i]->formation4);
		formationElements[i]->square.setNextBehavior(formationElements[i]->formation4);

		// Subset 3 Triangle
		if (i < 3) {
			std::vector<int> neighborVector3 = robotIDs;
			neighborVector3.erase(neighborVector3.begin() + i); // remove oneself
			neighborVector3.erase(neighborVector3.begin()+2, neighborVector3.end()); // only keep the first two!
			std::vector<double> triangleDistanceVector(neighborVector3.size(),2.5);

			// 3 formationControl
			formationElements[i]->formation3.getOptionContainer().getOption("tNeighbors").set(neighborVector3);
			// tJoystickTopic
			formationElements[i]->formation3.getOptionContainer().getOption("tJoystickTopic").set(options.tJoystickTopic->getValue());
			formationElements[i]->formation3.getOptionContainer().getOption("tVelocityInputTopic").set(options.tVelocityInputTopic->getValue());

			formationElements[i]->formation3.setParameterInitialized(true);

			// Also configure ReformationBehavior
			formationElements[i]->triangle.getOptionContainer().getOption("tNeighbors").set(neighborVector3);
			formationElements[i]->triangle.getOptionContainer().getOption("tNeighborDistances").set(triangleDistanceVector);
			formationElements[i]->triangle.setNextBehavior(formationElements[i]->formation3);

			formationElements[i]->triangle.setParameterInitialized(true);

		}
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
			formationElements[i]->bController->switchBehavior(formationElements[i]->tetraeder);
		}
	}

	if (msg->buttons[4]) {
		for (unsigned int i = 0; i < formationElements.size(); i++) {
			//formationElements[i]->switchIntoFormation();
			formationElements[i]->bController->switchBehavior(formationElements[i]->square);
		}
	}
	if (msg->buttons[5]) {
		for (unsigned int i = 0; i < formationElements.size()-1; i++) {
			//formationElements[i]->switchIntoFormation();
			formationElements[i]->bController->switchBehavior(formationElements[i]->triangle);
		}

		formationElements[formationElements.size()-1]->bController->switchBehavior(formationElements[formationElements.size()-1]->land);
	}

	// 6 is taken by emerygency
	if (msg->buttons[7]) {
//		for (unsigned int i = 0; i < formationElements.size()-1; i++) {
//			//formationElements[i]->switchIntoFormation();
//			formationElements[i]->bController->switchBehavior(formationElements[i]->triangle);
//		}

		formationElements[formationElements.size()-1]->bController->switchBehavior(formationElements[formationElements.size()-1]->takeOff);
	}

//	if (msg->buttons[3]) {
//		bController->switchBehavior(calibrator);
//	}


	// what to do

}

