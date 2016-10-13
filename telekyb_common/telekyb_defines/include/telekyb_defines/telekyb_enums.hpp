/*
 * telekyb_enums.hpp
 *
 *  Created on: Nov 8, 2011
 *      Author: mriedel
 */

#ifndef TELEKYB_ENUMS_HPP_
#define TELEKYB_ENUMS_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/enum.hpp>

namespace TELEKYB_NAMESPACE {

TELEKYB_ENUM_VALUES(PosControlType, const char*,
		(Position)("Position Control Mode")
		(Velocity)("Velocity Control Mode")
		(Acceleration)("Acceleration Control Mode")
		(Geometric)("Back-stepping Output Regulation Mode")
		(Full3DPose)("3D pose regulation")
)

TELEKYB_ENUM_VALUES(GlobalPosControlType, const char*,
		(Position)("Position Control Mode of all Axes")
		(Velocity)("Velocity Control Mode of all Axes")
		(Acceleration)("Acceleration Control Mode of all Axes")
		(Geometric)("Back-stepping Output Regulation Mode")
		(Full3DPose)("3D pose regulation")
		(Mixed)("Mixed PosControlTypes.")
		(Undef)("To not use. Used to suppress warnings.")
)

TELEKYB_ENUM_VALUES(YawControlType, const char*,
		(AngleOnBoard)("Angle Mode, Partly OnBoard")
		(RateOnBoard)("Rate Mode, Partly OnBoard")
		(AccelerationOnBoard)("Acceleration Mode, Partly OnBoard")
		(AngleOffBoard)("Angle Mode")
		(RateOffBoard)("Rate Mode")
		(AccelerationOffBoard)("Acceleration Mode")
)

}

#endif /* TELEKYB_ENUMS_HPP_ */
