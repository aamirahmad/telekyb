/*
 * TeleKybCoreOptions.hpp
 *
 *  Created on: Nov 3, 2011
 *      Author: mriedel
 */

#ifndef TELEKYBCOREOPTIONS_HPP_
#define TELEKYBCOREOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>


#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class TeleKybCoreOptions : public OptionContainer {
public:
	Option<int>* tRobotID;

	TeleKybCoreOptions();
};

}

#endif /* TELEKYBCOREOPTIONS_HPP_ */
