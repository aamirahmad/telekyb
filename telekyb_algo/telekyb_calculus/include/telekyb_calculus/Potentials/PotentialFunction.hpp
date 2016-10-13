/*
 * PotentialFunction.hpp
 *
 *  Created on: Sep 8, 2012
 *      Author: mriedel
 */

#ifndef POTENTIALFUNCTION_HPP_
#define POTENTIALFUNCTION_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_calculus/Potentials/PotentialFunctionOptions.hpp>


namespace TELEKYB_NAMESPACE {

class PotentialFunction : OptionListener<double> {
private:
	void registerOptionListeners();

protected:
	PotentialFunctionOptions options;
	//_PotentialImpl potentialImpl;
	PotentialFunctionType type;

	void updateBoundsValues();

public:
	PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_);
	PotentialFunction(const std::string& potentialFunctionName_, PotentialFunctionType type_,
			double tPotFuncZeroD_, double tPotFuncInfD_, double tPotFuncSatValue_, double tPotFuncGain_);
	virtual ~PotentialFunction();


	// get Type
	PotentialFunctionType getType() const;
	PotentialFunctionOptions& getOptions();

	virtual void optionDidChange(const Option<double>* option_);

	double getPotential(double d) const;
	// Pure virtual, overwrite
	virtual double getPotentialImpl(double d) const = 0;
};

} /* namespace telekyb */
#endif /* POTENTIALFUNCTION_HPP_ */
