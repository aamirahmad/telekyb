/*
 * LLJoystickOptions.hpp
 *
 *  Created on: Oct 29, 2011
 *      Author: mriedel
 */

#ifndef LLJOYSTICKOPTIONS_HPP_
#define LLJOYSTICKOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class LLJoystickOptions : public OptionContainer {
public:
	Option<std::string>* tJoystickTopic;
	Option<std::string>* tLLCommandsTopic;

	Option<int>* tCommandRate;

	LLJoystickOptions();
};

}

#endif /* LLJOYSTICKOPTIONS_HPP_ */
