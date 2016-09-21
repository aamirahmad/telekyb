/*
 * HandJoystickOptions.hpp
 *
 *  Created on: Jan 5, 2012
 *      Author: mriedel
 */

#ifndef HANDJOYSTICKOPTIONS_HPP_
#define HANDJOYSTICKOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class HandJoystickOptions : OptionContainer {
public:
	HandJoystickOptions();
	Option<std::string>* tJoystickTopic;
	Option<std::string>* tTransformStampedTopic;
	Option<std::string>* tJoyPubName;
	Option<std::string>* tTransformPubName;
	Option<std::string>* tChildFrameID;

	Option<double>* tMaxAxisValue;
	Option<double>* tAxisValueScale;

};

} /* namespace telekyb */
#endif /* HANDJOYSTICKOPTIONS_HPP_ */
