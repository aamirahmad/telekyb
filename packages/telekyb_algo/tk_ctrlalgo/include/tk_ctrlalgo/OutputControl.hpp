#ifndef OUTPUTCONTROL_HPP_
#define OUTPUTCONTROL_HPP_

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include "OutputCtrlDefines.hpp"
#include <telekyb_base/Messages.hpp>
#include <sensor_msgs/Imu.h>

namespace TELEKYB_NAMESPACE {

class OutputControlOptions : public OptionContainer {
public:

	Option<double>* tMinThrust;
    Option<double>* tMaxThrust;
	
    // Control parameters (Gains) for back-stepping regulator for position
    Option<double>* tPosGainX_bkstp;  //Position Gain on x-axis
    Option<double>* tPosGainY_bkstp;  //Position Gain on y-axis
    Option<double>* tPosGainZ_bkstp;  //Position Gain on z-axis
    Option<double>* tVelGainX_bkstp;  //Velocity Gain on x-axis
    Option<double>* tVelGainY_bkstp;  //Velocity Gain on y-axis
    Option<double>* tVelGainZ_bkstp;  //Velocity Gain on z-axis

    // Control parameters (Gains) for disturbance estimator
    Option<double>* tEstGainX_disturbance;  //Disturbance Est Gain on x-axis
    Option<double>* tEstGainY_disturbance;  //Disturbance Est Gain on y-axis
    Option<double>* tEstGainZ_disturbance;  //Disturbance Est Gain on z-axis

    // Control parameters (Gains) for output regulator for attitude
    Option<double>* tOmegaGainX_OR; //Gain of angular velocity
    Option<double>* tOmegaGainY_OR; //Gain of angular velocity
    Option<double>* tOmegaGainZ_OR; //Gain of angular velocity
    Option<double>* tRotGainX_OR;   //Gain of tracking rotation matrix
    Option<double>* tRotGainY_OR;   //Gain of tracking rotation matrix
    Option<double>* tRotGainZ_OR;   //Gain of tracking rotation matrix

    // Weight value for cost function of MPC-based trajectory planner
    Option<double>* tCostWeight_P;  //Weight value for position state stage cost, L_stage(0,0)
    Option<double>* tCostWeight_V;  //Weight value for velocity state stage cost, L_stage(1,0)
    Option<double>* tCostWeight_A;  //Weight value for acceleration state stage cost, L_stage(2,0)
    Option<double>* tCostWeight_U;  //Weight value for input stage cost, L_stage(0,1)
    Option<double>* tCostWeight_Pt;  //Weight value for position term cost, L_stage(0,2)
    Option<double>* tCostWeight_Vt;  //Weight value for velocity term cost, L_stage(1,2)
    Option<double>* tCostWeight_At;  //Weight value for acceleration term cost, L_stage(2,2)

    // Publish Attitude and Position information for quadrotor
    Option<bool>* tPubAttitude;
    Option<bool>* tPubPosition;
    Option<bool>* tInitialYawAngle;

    // Switch to simulate Hexacopter for EuRoC challenge
    Option<bool>* tSimulationEuRoC;

    // Utilize a MPC-based online trajectory planner for quadrotor
    Option<bool>* tOnlineTrajectoryPlanning;

    // Switch to Geometric Tracking method @ GRASP for comparison
    Option<bool>* tTestGRASP;

    // Physical Parameter
    Option<double>* tGravity;       // Gravity value
    Option<double>* tArmLength;       // Gravity value
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
	
    OutputControlOptions();
};

class OutputControl {
private:
    OutputControlOptions options;

    // Constant parameters
    Matrix3D InertialMatrix;
    Vector3D ExtForce;
//    Vector3D offset;
//    Vector3D iniOffset;
    Vector3D lastPosition;
    Vector3D EstDisturbance;
    Matrix3D RefMatrix;
//    Vector3D dEstDisturbance;
    double armLength;
    int nrMotors;
    double PropellerConst;
    double PropellerGain;
    double MinMotorCMD;
    double MaxMotorCMD;
    double MaxTorqueX;
    double MaxTorqueY;
    double MaxTorqueZ;

    Matrix4D inputTransMatrix;

    // Parameters for Hexarotor
    double motorConst;
    double momentConst;
    int nrMotors_Hex;
    Eigen::Matrix<double, 4, 6, Eigen::DontAlign> inputTransMatrix_Hex;

    double iniThrust;
    // temp variables
    double time;
	
    double gravity;
    int timer_traj;
    double iniYawAngle;

    Timer integTimer;
//    Timer difTimer;

	ros::Publisher CurPosPublisher;
	ros::Publisher DesPosPublisher;
    ros::Publisher CurVelPublisher;
	ros::Publisher CurAttPublisher;
	ros::Publisher DesAttPublisher;
	ros::NodeHandle mainNodehandle;

public:
	// gravity is provided by
    OutputControl();
    virtual ~OutputControl();

    void run(const TKTrajectory& input, const TKState& currentState, const double mass, OutputCtrlOutput& output, const sensor_msgs::Imu& unbiasedImu);
};

}

#endif /* OUTPUTCONTROL_HPP_ */
