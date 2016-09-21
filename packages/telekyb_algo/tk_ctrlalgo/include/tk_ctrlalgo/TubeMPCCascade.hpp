#ifndef TUBEMPCCASCADE_HPP
#define TUBEMPCCASCADE_HPP

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include "TubeMPCCtrlDefines.hpp"
#include <telekyb_base/Messages.hpp>
#include <sensor_msgs/Imu.h>

namespace TELEKYB_NAMESPACE {

class TubeMPCControlOptions : public OptionContainer {
public:

    Option<double>* tMinThrust;
    Option<double>* tMaxThrust;

    // Weight value for cost function of the NOMINAL MPC position controller
    Option<double>* tCostWeight_pLstagexy;  //Weight value for position state stage cost 6x6, L_stage(0,0;2,2;4,4)
    Option<double>* tCostWeight_pLstagez;  //Weight value for position state stage cost 6x6, L_stage(0,0;2,2;4,4)
    Option<double>* tCostWeight_vLstage;  //Weight value for velocity state stage cost 6x6, L_stage(1,1;3,3;5,5)
    Option<double>* tCostWeight_Rxy;  //Weight value (horizontal) for input stage cost 3x3, L_stage(0,0;1,1)
    Option<double>* tCostWeight_Rz;  //Weight value (vertical) for input stage cost 3x3, L_stage(2,2)
    Option<double>* tCostWeight_pLtermxy;  //Weight value for position terminal cost 6x6, L_term(0,0;2,2;4,4)
    Option<double>* tCostWeight_pLtermz;  //Weight value for position terminal cost 6x6, L_term(0,0;2,2;4,4)
    Option<double>* tCostWeight_vLterm;  //Weight value for velocity terminal cost 6x6, L_term(1,1;3,3;5,5)

    // Parameters of the LQR stablizing gain for tube-MPC
    Option<double>* tLqrGain_pxy; // Horizontal position gain for LQR Gain Matrix K_LQR 3x6, K_LQR(0,0;1,2)
    Option<double>* tLqrGain_pz; // Vertical position gain for LQR Gain Matrix K_LQR 3x6, K_LQR(2,4)
    Option<double>* tLqrGain_vxy; // Horizontal velocity gain for LQR Gain Matrix K_LQR 3x6, K_LQR(0,1;1,3)
    Option<double>* tLqrGain_vz; // Vertical velocity gain for LQR Gain Matrix K_LQR 3x6, K_LQR(2,5)

    // Control parameters (Gains) for global output regulator for attitude
    Option<double>* tOmegaGainX_OR; //Gain of angular velocity
    Option<double>* tOmegaGainY_OR; //Gain of angular velocity
    Option<double>* tOmegaGainZ_OR; //Gain of angular velocity
    Option<double>* tRotGainX_OR;   //Gain of tracking rotation matrix
    Option<double>* tRotGainY_OR;   //Gain of tracking rotation matrix
    Option<double>* tRotGainZ_OR;   //Gain of tracking rotation matrix

    // Publish Attitude and Position information for quadrotor
    Option<bool>* tPubAttitude;
    Option<bool>* tPubPosition;
    Option<bool>* tInitialYawAngle;

    // Switch to simulate Hexacopter for EuRoC challenge
    Option<bool>* tSimulationEuRoC;

    // Switch to import known obstacle position
    Option<bool>* tObstacleAvoidance;

    // Physical Parameter
    Option<double>* tGravity;       // Gravity value
    Option<double>* tArmLength;       // Arm length on multi-rotor
    Option<double>* txAxisInertial; // Body-frame inertial value in x-axis
    Option<double>* tyAxisInertial; // Body-frame inertial value in y-axis
    Option<double>* tzAxisInertial; // Body-frame inertial value in z-axis

    Option<double>* tPropellerConst; // Torque Constant Cq divide by Thrust Constant Ct of propellers
    Option<double>* tPropellerGainConst; // Torque Constant Cq of propellers
    Option<double>* tMinMotorCMD;   // Minimum motor speed on the quadrotor
    Option<double>* tMaxMotorCMD;   // Maximum motor speed on the quadrotor

    // Parameters for Hexarotor
    Option<double>* tmotorConst; // Torque Constant Cq divide by Thrust Constant Ct of propellers
    Option<double>* tmomentConst; // Torque Constant Cq of propellers

    // Obstacle Position Data
//    Option<Eigen::MatrixX3d>* ObstaclePosition; // (x1,y1,z1;x2,y2,z2;x3,y3,z3;...)
    Option<double>* tRadiusObstacle;       // Assumed radius of obstacle
    Option<double>* tRadiusSafety;       // Assumed safe radius of multi-rotor

    TubeMPCControlOptions();
};

class TubeMPCControl {
private:
    TubeMPCControlOptions options;

    // Constant parameters
    Matrix3D InertialMatrix;
    Vector3D ExtForce;
    Matrix3D RefMatrix;
    double armLength;
    int nrMotors;
    double PropellerConst;
    double PropellerGain;
    double MinMotorCMD;
    double MaxMotorCMD;

    Matrix4D inputTransMatrix;

    Eigen::Matrix<double, 6, 1> NextState_last;
    Eigen::Matrix<double, 4, 3> ObstaclePosition;
    Eigen::Matrix<double, 3, 4> Obstacle;
    Vector3D sub_terminal_last;

    // Parameters for Hexarotor
    double motorConst;
    double momentConst;
    int nrMotors_Hex;
    Eigen::Matrix<double, 4, 6, Eigen::DontAlign> inputTransMatrix_Hex;

    double iniThrust;
    // temp variables
    double time;

    double gravity;
    int timer_subtarget;
    int timer_initialYaw;
    double iniYawAngle;

    Timer integTimer;
//    Timer SamplingTimer;    // To set up a nominal 50Hz Control Timestep

    ros::Publisher CurPosPublisher;
    ros::Publisher DesPosPublisher;
    ros::Publisher CurVelPublisher;
//    ros::Publisher CurAngVelPublisher;
    ros::Publisher CurAttPublisher;
    ros::Publisher DesAttPublisher;
    ros::NodeHandle mainNodehandle;

public:
    // gravity is provided by
    TubeMPCControl();
    virtual ~TubeMPCControl();

    void run_pos(const TKTrajectory& input, const TKState& currentState, const double mass, TubeMPCCtrlOutput& output, const sensor_msgs::Imu& unbiasedImu);


    void run_att(const Vector3D desired_force, const TKTrajectory& input, const TKState& currentState, TubeMPCCtrlOutput& output, const sensor_msgs::Imu& unbiasedImu);
};

}


#endif // TUBEMPCCASCADE_HPP
