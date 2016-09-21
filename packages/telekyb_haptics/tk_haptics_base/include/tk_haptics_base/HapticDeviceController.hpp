/*
 * HapticDeviceController.hpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#ifndef HAPTICDEVICECONTROLLER_HPP_
#define HAPTICDEVICECONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_haptics_base/HapticDeviceDefines.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE
{

class HapticDeviceController
{
public:
	// Identifier (e.g. for NodeHandle)
	virtual void setIdentifier(const std::string& identifier) = 0;

	// Get's specific Axes mapping, Set if needed
	virtual void setAxesMapping(HapticAxesMapping& xAxis, HapticAxesMapping& yAxis, HapticAxesMapping& zAxis) = 0;

	// Get the Range of each axes
	virtual void setAxesRange(const Position3D& minValues, const Position3D& maxValues) = 0;

	// Before Entering Spinloop
	virtual void willEnterSpinLoop() = 0;

	// has to be fast and should not slow down the loop
	virtual void loopCB(const HapticOuput& output, HapticInput& input) = 0;

	// After Exiting Spinloop
	virtual void didLeaveSpinLoop() = 0;
};

};

#endif /* HAPTICDEVICECONTROLLER_HPP_ */
