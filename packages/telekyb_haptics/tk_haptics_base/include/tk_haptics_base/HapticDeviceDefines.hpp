/*
 * HapticDeviceDefines.hpp
 *
 *  Created on: Mar 4, 2012
 *      Author: mriedel
 */

#ifndef HAPTICDEVICEDEFINES_HPP_
#define HAPTICDEVICEDEFINES_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/enum.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

struct HapticInput {
	Eigen::Vector3d force;
};

struct HapticOuput {
	Position3D position;
	Velocity3D linVelocity;
	Quaternion orientation;
	Eigen::Vector3d force;
	double frequency; // loop frequency
	int primaryButton; // 1 if pressed
};

// For Cartesian Mapping
TELEKYB_ENUM(HapticAxesMapping,
		(Forward)
		(Backward)
		(Up)
		(Down)
		(Left)
		(Right)
)


}

#endif /* HAPTICDEVICEDEFINES_HPP_ */
