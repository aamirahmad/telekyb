/*
 * MKOmegaControlROSInterfaceOptions.hpp
 *
 *  Created on: Nov 29, 2011
 *      Author: mriedel
 */

#ifndef MKOMEGACONTROLROSINTERFACEOPTIONS_HPP_
#define MKOMEGACONTROLROSINTERFACEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>

#include <telekyb_base/Options.hpp>

namespace TELEKYB_NAMESPACE {

class MKOmegaControlROSInterfaceOptions : public OptionContainer {
public:
	Option<std::string>* tCommandsTopic;
	Option<double>* tBatteryUpdatePeriod;
	Option<std::string>* tEmergencyLandService;
	MKOmegaControlROSInterfaceOptions();
};

} /* namespace telekyb */
#endif /* MKOMEGACONTROLROSINTERFACEOPTIONS_HPP_ */
