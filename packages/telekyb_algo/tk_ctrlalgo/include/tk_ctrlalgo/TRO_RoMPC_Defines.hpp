#ifndef TRO_ROMPC_DEFINES_HPP
#define TRO_ROMPC_DEFINES_HPP

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_defines/telekyb_enums.hpp>

#include <telekyb_base/Spaces.hpp>

namespace TELEKYB_NAMESPACE {

struct RoMPCInput {
    // Current State
    Position3D curPosition; // (x,y,z)
    Velocity3D curLinVelocity; // (dx,dy,dz)
    RotAngle3D curOrientation; // (0,1,2) -> (roll = x, pitch = y, yaw = z)
    AngVelocity3D curAngVelocity; // (0,1,2) -> (omega_x = x, omega_y = y, omega_z = z)

    // Desired State
    Position3D desPosition;     // (x_ref,y_ref,z_ref)
    Velocity3D desLinVelocity;  // (dx_ref,dy_ref,dz_ref)
    Acceleration3D desAcceleration;  // (ddx_ref,ddy_ref,ddz_ref)
    RotAngle3D desOrientation;    // (0,1,2) -> (roll_ref,pitch_ref,yaw_ref)

    // Obstacle State
    // Matrix3D obstaclePosition; // (x1,y1,z1;x2,y2,z2;x3,y3,z3)

};

struct RoMPCOutput {

    // Transmission into Motor Speed
    Vector4D motorBuf; // Nominal motor speed square Vector 4x1
    Eigen::Matrix<double, 6, 1, Eigen::DontAlign> motorBuf_Hex; // motor speed vector 6x1 for Hexacopter in EuRoC Challenge
    Vector3D desired_force; // Nominal force vector computed from Tube MPC
    double Thrust; // Nominal Thrust
    Vector4D CtrlInput; // Nominal Thrust & Torque for nonlinear observer
};

}

#endif // TRO_ROMPC_DEFINES_HPP
