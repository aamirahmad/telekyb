/*
 * DHDDeviceOptions.hpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#ifndef DHDDEVICEOPTIONS_HPP_
#define DHDDEVICEOPTIONS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

class DHDDeviceOptions : public OptionContainer {
public:
	Option<std::string>* tHapticDeviceController;
	Option<bool>* tEnableForceAtStart;
	Option<double>* tCustomEffectorMass;
	Option<bool>* tDisableGravityCompensation;
	Option<Position3D>* tCenterTranslation;
	Option<Vector3D>* tForceOffset;

	Option< double >* tStatusOutputFreq;
	Option< std::string >* tStatusOutputTopic;

	DHDDeviceOptions(const std::string& identifier);
};

}

#endif /* DHDDEVICEOPTIONS_HPP_ */
