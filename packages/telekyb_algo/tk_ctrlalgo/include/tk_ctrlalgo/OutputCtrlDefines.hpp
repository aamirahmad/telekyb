/*
 * PosCtrlElement.hpp
 *
 *  Created on: Oct 27, 2011
 *      Author: mriedel
 */

#ifndef OUTPUTCTRLDEFINES_HPP_
#define OUTPUTCTRLDEFINES_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

struct OutputCtrlInput {
    // Control Mode
    PosControlType CtrlMode;
	// Current State
	Position3D curPosition; // (x,y,z)
    Velocity3D curLinVelocity; // (dx,dy,dz)
    RotAngle3D curOrientation; // (0,1,2) -> (roll = x, pitch = y, yaw = z)
    AngVelocity3D curAngVelocity; // (0,1,2) -> (omega_x = x, omega_y = y, omega_z = z)

	// Desired State
    Position3D desPosition;     // (x_ref,y_ref,z_ref)
    Velocity3D desLinVelocity;  // (dx_ref,dy_ref,dz_ref)
    Acceleration3D desAcceleration;  // (ddx_ref,ddy_ref,ddz_ref)	
    Acceleration3D desJerk;     // (dddx_ref,dddy_ref,dddz_ref)
    RotAngle3D desOrientation;    // (0,1,2) -> (roll_ref,pitch_ref,yaw_ref)
//    Velocity3D desAngVelocity;  // (0,1,2) -> (omega_x_ref, omega_y_ref, omega_z_ref)

};

struct OutputCtrlOutput {

    // Transmission into Motor Speed
    Vector4D motorBuf; // Nominal motor speed square Vector 4x1
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> motorBuf_Hex; // motor speed vector 6x1 for Hexacopter in EuRoC Challenge
//    Vector4D NominalForce;

};

}

#endif /* OUTPUTCTRLDEFINES_HPP_ */
