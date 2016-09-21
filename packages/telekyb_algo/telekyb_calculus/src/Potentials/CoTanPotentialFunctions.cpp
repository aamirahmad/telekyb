/*
 * CoTanPotentialFunctions.cpp
 *
 *  Created on: Sep 10, 2012
 *      Author: mriedel
 */

#include <telekyb_calculus/Potentials/CoTanPotentialFunctions.hpp>

namespace TELEKYB_NAMESPACE {

CoTanRepulsiveGradient::CoTanRepulsiveGradient(const std::string& potentialFunctionName_)
	: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Repulsive)
{

}

CoTanRepulsiveGradient::CoTanRepulsiveGradient(const std::string& potentialFunctionName_,
		double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_)
: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Repulsive,
		tPotFuncZeroD_, tPotFuncInfD_, tPotFuncSatValue_, tPotFuncGain_)
{

}

double CoTanRepulsiveGradient::getPotentialImpl(double d) const {
	double z = (M_PI/2.0) *
			((d - options.tPotFuncInfD->getValue())
					/ (options.tPotFuncZeroD->getValue() - options.tPotFuncInfD->getValue()));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (options.tPotFuncZeroD->getValue() - options.tPotFuncInfD->getValue())) *
			(cot_z + z - (M_PI/2.0));
//			pow((cot_z + z - (M_PI/2.0)),options.tPotFuncGain->getValue());
	return options.tPotFuncGain->getValue() * retValue;
}


CoTanAttractiveGradient::CoTanAttractiveGradient(const std::string& potentialFunctionName_)
	: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Attractive)
{

}

CoTanAttractiveGradient::CoTanAttractiveGradient(const std::string& potentialFunctionName_,
		double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_)
: PotentialFunction(potentialFunctionName_, PotentialFunctionType::Attractive,
		tPotFuncZeroD_, tPotFuncInfD_, tPotFuncSatValue_, tPotFuncGain_)
{

}

double CoTanAttractiveGradient::getPotentialImpl(double d) const {
	double z = (M_PI/2.0) *
			((options.tPotFuncInfD->getValue() - d)
					/ (options.tPotFuncInfD->getValue() - options.tPotFuncZeroD->getValue()));
	double cot_z = cos(z)/sin(z);
	double retValue = (M_PI/2.0) * (1.0 / (options.tPotFuncInfD->getValue() - options.tPotFuncZeroD->getValue())) *
			(cot_z + z - (M_PI/2.0));
//			pow((cot_z + z - (M_PI/2.0)),options.tPotFuncGain->getValue());
	return options.tPotFuncGain->getValue() * retValue;
}


} /* namespace telekyb */
