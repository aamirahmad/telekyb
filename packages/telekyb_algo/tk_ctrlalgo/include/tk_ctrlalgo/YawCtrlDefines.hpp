/*
 * YawCtrlElement.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef YAWCTRLELEMENT_HPP_
#define YAWCTRLELEMENT_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {


struct YawCtrlInput {
	// Type
	YawControlType crtlType;

	// Current State
	double curYawAngle;
	double curYawRate;

	// desiered State
	double desYawAngle;
	double desYawRate;
	double desYawAcceleration;
};

struct YawCtrlOutput {
	double comYaw;
};


}

#endif /* YAWCTRLELEMENT_HPP_ */
