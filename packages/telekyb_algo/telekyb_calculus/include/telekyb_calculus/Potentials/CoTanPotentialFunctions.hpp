/*
 * CoTanPotentialFunctions.hpp
 *
 *  Created on: Sep 10, 2012
 *      Author: mriedel
 */

#ifndef COTANPOTENTIALFUNCTIONS_HPP_
#define COTANPOTENTIALFUNCTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_calculus/Potentials/PotentialFunction.hpp>

namespace TELEKYB_NAMESPACE {

class CoTanRepulsiveGradient : public PotentialFunction {

public:
	CoTanRepulsiveGradient(const std::string& potentialFunctionName_);
	CoTanRepulsiveGradient(const std::string& potentialFunctionName_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);

	double getPotentialImpl(double d) const;
};


class CoTanAttractiveGradient : public PotentialFunction {

public:
	CoTanAttractiveGradient(const std::string& potentialFunctionName_);
	CoTanAttractiveGradient(const std::string& potentialFunctionName_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);

	double getPotentialImpl(double d) const;
};

} /* namespace telekyb */
#endif /* COTANPOTENTIALFUNCTIONS_HPP_ */
