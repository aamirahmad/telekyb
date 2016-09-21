/*
 * FormationControlElement.hpp
 *
 *  Created on: Feb 10, 2012
 *      Author: mriedel
 */

#ifndef FORMATIONCONTROLELEMENT_HPP_
#define FORMATIONCONTROLELEMENT_HPP_

#include <telekyb_interface/TeleKybCore.hpp>
#include <telekyb_interface/MKInterface.hpp>

using namespace telekyb;

class FormationControlElement : telekyb_interface::ActiveBehaviorListener {
private:
	void setupFormationControlElement();

public:
	// ID
	int robotID;

	// the System
	telekyb_interface::TeleKybCore* core;
	// BehaviorController
	telekyb_interface::BehaviorController* bController;
	telekyb_interface::OptionController *oController;

	// Optional MKInterface
	telekyb_interface::MKInterface* mkInterface;

	// Behavior
	telekyb_interface::Behavior* activeBehaviorPtr;


	FormationControlElement(int robotID_, bool useMKInterface_);
	virtual ~FormationControlElement();

	void activeBehaviorChanged(telekyb_interface::Behavior newActiveBehavior);

	telekyb_interface::MKInterface* getMKInterfacePointer() const;
	telekyb_interface::TeleKybCore* getTeleKybCorePointer() const;

	// for convienence
	bool mkSetEmergency();

	bool mkToggleMotors();

	// lift land
	bool liftland();


//	void switchIntoFormation() const;

	// Behaviors
	// System Behaviors
	telekyb_interface::Behavior ground;
	telekyb_interface::Behavior hover;
	telekyb_interface::Behavior normalBreak;
	telekyb_interface::Behavior takeOff;
	telekyb_interface::Behavior land;

	// FormationControl
	telekyb_interface::Behavior formation4;
	telekyb_interface::Behavior formation3;
	telekyb_interface::Behavior tetraeder;
	telekyb_interface::Behavior square;
	telekyb_interface::Behavior triangle;


};

#endif /* FORMATIONCONTROLELEMENT_HPP_ */
