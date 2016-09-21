/*
 * PotentialFunctionOptions.hpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */

#ifndef POTENTIALFUNCTIONOPTIONS_HPP_
#define POTENTIALFUNCTIONOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_defines/enum.hpp>
#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

TELEKYB_ENUM(PotentialFunctionType,
		(Attractive)
		(Repulsive)
)

class PotentialFunctionOptions : public OptionContainer {
public:
	Option<double>* tPotFuncZeroD;
	Option<double>* tPotFuncInfD;
	Option<double>* tPotFuncSatValue;
	Option<double>* tPotFuncGain;

	PotentialFunctionOptions(const std::string& potentialFunctionName_);
};

} /* namespace telekyb */
#endif /* POTENTIALFUNCTIONOPTIONS_HPP_ */
