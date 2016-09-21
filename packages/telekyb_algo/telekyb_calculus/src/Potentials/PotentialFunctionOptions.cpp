/*
 * PotentialFunctionOptions.cpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */

#include <telekyb_calculus/Potentials/PotentialFunctionOptions.hpp>

namespace TELEKYB_NAMESPACE {

PotentialFunctionOptions::PotentialFunctionOptions(const std::string& potentialFunctionName_)
	: OptionContainer(potentialFunctionName_)
{
	tPotFuncZeroD = addBoundsOption<double>("tPotFuncZeroD",
			"Input value where the potential turns 0. Must be positive.",
			0.0, 0.0 ,std::numeric_limits<double>::infinity(),false, false);
	tPotFuncInfD = addBoundsOption<double>("tPotFuncInfD",
			"Input value where the potential turns infinity. Must be positive",
			0.0, 0.0,std::numeric_limits<double>::infinity(),false,false);
	tPotFuncSatValue = addBoundsOption<double>("tPotFuncSatValue",
			"Input value where the potential turns infinity. Must be positive.",
			0.0, 0.0,std::numeric_limits<double>::infinity(),false,false);
	tPotFuncGain = addBoundsOption<double>("tPotFuncGain",
			"Gain Value for function! Beware, use is up to the concrete implementation. Must be >0!",
			1.0, std::numeric_limits<double>::denorm_min(),std::numeric_limits<double>::infinity(),false,false);

}

} /* namespace telekyb */
