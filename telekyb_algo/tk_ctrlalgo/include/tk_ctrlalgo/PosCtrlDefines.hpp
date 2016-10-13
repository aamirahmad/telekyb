/*
 * PosCtrlElement.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef POSCTRLELEMENT_HPP_
#define POSCTRLELEMENT_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

struct PosCtrlInput {
	// Control Mode for each Axis
	PosControlType crtlTypeX;
	PosControlType crtlTypeY;
	PosControlType crtlTypeZ;

	// Current State
	Position3D curPosition; // (x,y,z)
	Velocity3D curLinVelocity; // (x,y,z)
	Vector3D curOrientation; // (0,1,2) -> (roll,pitch,yaw)
	Velocity3D curAngVelocity; // (0,1,2) -> (omega_x, omega_y, omega_z)

	// Desired State
	Position3D desPosition;
	Velocity3D desLinVelocity;
	Acceleration3D desLinAcceleration;
};

struct PosCtrlOutput {
	double comRoll;
	double comPitch;
	double comThrust;
};

}

#endif /* POSCTRLELEMENT_HPP_ */
