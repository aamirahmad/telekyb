/*
 * SampleController.hpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#ifndef SAMPLECONTROLLER_HPP_
#define SAMPLECONTROLLER_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <tk_haptics_base/HapticDeviceController.hpp>

#include <telekyb_base/Time.hpp>

using namespace TELEKYB_NAMESPACE;

namespace telekyb_haptic {

class SampleController : public HapticDeviceController {
protected:
	Timer frequencyTimer;

public:
	SampleController();
	virtual ~SampleController();

	// Identifier (e.g. for NodeHandle)
	void setIdentifier(const std::string& identifier);

	// Get's specific Axes mapping, Set if needed
	void setAxesMapping(HapticAxesMapping& xAxis, HapticAxesMapping& yAxis, HapticAxesMapping& zAxis);

	// Get the Range of each axes
	void setAxesRange(const Position3D& minValues, const Position3D& maxValues);

	void willEnterSpinLoop();

	// has to be fast and should not slow down the loop
	void loopCB(const HapticOuput& output, HapticInput& input);

	void didLeaveSpinLoop();
};

}

#endif /* SAMPLECONTROLLER_HPP_ */
