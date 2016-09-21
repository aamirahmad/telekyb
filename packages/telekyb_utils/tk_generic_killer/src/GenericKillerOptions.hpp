/*
 * GenericKillerOptions.hpp
 *
 *  Created on: Aug 13, 2012
 *      Author: tnestmeyer
 */

#ifndef GENERICKILLEROPTIONS_HPP_
#define GENERICKILLEROPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class GenericKillerOptions : OptionContainer {
public:
	Option<std::string>* tTopicName;
	Option<double>* tTimeOut;
	Option<std::string>* tProcessName;

	GenericKillerOptions();
};

} /* namespace telekyb */
#endif /* GENERICKILLEROPTIONS_HPP_ */
