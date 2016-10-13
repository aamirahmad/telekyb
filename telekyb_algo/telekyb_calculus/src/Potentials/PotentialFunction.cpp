/*
 * PotentialFunction.cpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */

#include <telekyb_calculus/Potentials/PotentialFunction.hpp>

#include <telekyb_defines/telekyb_defines.hpp>

#include <ros/console.h>

namespace TELEKYB_NAMESPACE {

PotentialFunction::PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_)
	: options(potentialFunctionName_), type(type_)
{

//	//TODO: Implement exception
	if (options.tPotFuncZeroD->isOnInitialValue()) {
		ROS_ERROR("%s on initial value! This is most certainly not what you want!", options.tPotFuncZeroD->getNSName().c_str());
	}
	if (options.tPotFuncInfD->isOnInitialValue()) {
		ROS_ERROR("%s on initial value! This is most certainly not what you want!", options.tPotFuncInfD->getNSName().c_str());
	}
	if (options.tPotFuncSatValue->isOnInitialValue()) {
		ROS_ERROR("%s on initial value! This is most certainly not what you want!", options.tPotFuncSatValue->getNSName().c_str());
	}

//	optionStruct.tPotFuncZeroD = options.tPotFuncZeroD->getValue();
//	optionStruct.tPotFuncInfD = options.tPotFuncInfD->getValue();
//	optionStruct.tPotFuncSatValue = options.tPotFuncSatValue->getValue();
//	optionStruct.tPotFuncGain = options.tPotFuncGain->getValue();

	updateBoundsValues();

	// Monitor Changes
	registerOptionListeners();
}
PotentialFunction::	PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_,
		double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_)
	: options(potentialFunctionName_), type(type_)
{

	// set check automatic
	options.tPotFuncZeroD->setValue(tPotFuncZeroD_);
	options.tPotFuncInfD->setValue(tPotFuncInfD_);
	options.tPotFuncSatValue->setValue(tPotFuncSatValue_);
	options.tPotFuncGain->setValue(tPotFuncGain_);

	// no callbacks implemented yet
	updateBoundsValues();

	// Monitor Changes
	registerOptionListeners();
}

PotentialFunction::~PotentialFunction() {
	options.tPotFuncZeroD->unRegisterOptionListener(this);
	options.tPotFuncInfD->unRegisterOptionListener(this);
	options.tPotFuncSatValue->unRegisterOptionListener(this);
	options.tPotFuncGain->unRegisterOptionListener(this);
}

void PotentialFunction::registerOptionListeners() {
	options.tPotFuncZeroD->registerOptionListener(this);
	options.tPotFuncInfD->registerOptionListener(this);
	options.tPotFuncSatValue->registerOptionListener(this);
	options.tPotFuncGain->registerOptionListener(this);
}

void PotentialFunction::updateBoundsValues() {
//	ROS_INFO("Called updateBoundsValues()");
	//PotentialFunctionType type = getType();

	double zeroDistance = options.tPotFuncZeroD->getValue();
	double infDistance = options.tPotFuncInfD->getValue();

//	ROS_INFO("Values: tPotFuncZeroD: %f, tPotFuncInfD: %f", zeroDistance, infDistance);

	// Beware: Do not check for equality here. Otherwise: Infinite loop.
	if (zeroDistance < infDistance && type == PotentialFunctionType::Repulsive) {
		ROS_ERROR("Potential Function Type %s, with d_0(%f) < d_inf(%f). Inverting...", type.str(), zeroDistance, infDistance);
		options.tPotFuncZeroD->setValue(infDistance);
		options.tPotFuncInfD->setValue(zeroDistance);
	} else if (infDistance < zeroDistance && type == PotentialFunctionType::Attractive) {
		ROS_ERROR("Potential Function Type %s, with d_0(%f) > d_inf(%f). Inverting...", type.str(), zeroDistance, infDistance);
		options.tPotFuncZeroD->setValue(infDistance);
		options.tPotFuncInfD->setValue(zeroDistance);
	} else {
		// everything ok;
	}
}

PotentialFunctionType PotentialFunction::getType() const {
	return type;
}

PotentialFunctionOptions& PotentialFunction::getOptions() {
	return options;
}

double PotentialFunction::getPotential(double d) const {
	if (type == PotentialFunctionType::Repulsive) {
		// repulsive
		if (d < options.tPotFuncInfD->getValue()) {
			return options.tPotFuncSatValue->getValue();
		} else if (d >= options.tPotFuncZeroD->getValue()) {
			return 0.0;
		} else {
			return std::min(getPotentialImpl(d), options.tPotFuncSatValue->getValue());
		}

	} else {
		// attractive
		if (d <= options.tPotFuncZeroD->getValue()) {
			return 0.0;
		} else if (d > options.tPotFuncInfD->getValue()) {
			return options.tPotFuncSatValue->getValue();
		} else {
			return std::min(getPotentialImpl(d), options.tPotFuncSatValue->getValue());
		}
	}
}

void PotentialFunction::optionDidChange(const Option<double>* option_) {
	if (option_ == options.tPotFuncZeroD) {
		ROS_WARN("Changed tPotFuncZeroD");
		updateBoundsValues();
	} else if (option_ == options.tPotFuncInfD) {
		ROS_WARN("Changed tPotFuncInfD");
		updateBoundsValues();
	} else {
		// we don't care
	}
}

} /* namespace telekyb */

