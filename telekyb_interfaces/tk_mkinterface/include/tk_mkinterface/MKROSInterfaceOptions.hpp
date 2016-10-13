/*
 * MKROSInterfaceOptions.hpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#ifndef MKROSINTERFACEOPTIONS_HPP_
#define MKROSINTERFACEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class MKROSInterfaceOptions : public OptionContainer {
public:
	Option<std::string>* tCommandsTopic;
	Option<double>* tBatteryUpdatePeriod;
	Option<std::string>* tEmergencyLandService;
	MKROSInterfaceOptions();
};

} /* namespace telekyb */
#endif /* MKROSINTERFACEOPTIONS_HPP_ */
