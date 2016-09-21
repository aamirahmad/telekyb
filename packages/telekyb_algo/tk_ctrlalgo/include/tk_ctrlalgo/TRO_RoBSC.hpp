#ifndef TRO_ROBSC_HPP
#define TRO_ROBSC_HPP

#include <telekyb_defines/telekyb_defines.hpp>
#include <telekyb_base/Options.hpp>
#include <telekyb_base/Time.hpp>

#include "TRO_RoBSC_Defines.hpp"
#include <telekyb_base/Messages.hpp>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace TELEKYB_NAMESPACE {

class RoBSCOptions : public OptionContainer {
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

    // Publish Attitude and Position information for quadrotor
    Option<bool>* tPubAttitude;
    Option<bool>* tPubPosition;
    Option<bool>* tInitialYawAngle;

    // Switch to simulate Hexacopter for EuRoC challenge
    Option<bool>* tSimulationEuRoC;

    // Utilize a MPC-based online trajectory planner for quadrotor
    Option<bool>* tOnlineTrajectoryPlanning;     
    Option<double>* tCost_Pxy_stage;   
    Option<double>* tCost_Vxy_stage;   
    Option<double>* tCost_Axy_stage;  
    Option<double>* tCost_Pz_stage;  
    Option<double>* tCost_Vz_stage;  
    Option<double>* tCost_Az_stage;  
    Option<double>* tCost_Ux;    
    Option<double>* tCost_Uy;    
    Option<double>* tCost_Uz;    
    Option<double>* tCost_Pxy_term;
    Option<double>* tCost_Vxy_term;
    Option<double>* tCost_Axy_term;
    Option<double>* tCost_Pz_term;
    Option<double>* tCost_Vz_term;
    Option<double>* tCost_Az_term;
    
    // Switch to Geometric Tracking method @ GRASP for comparison
    Option<bool>* tTestGRASP;

    // Switch to turn on the force estimator
    Option<bool>* tObserver;

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
	
    RoBSCOptions();
};

class RoBSC {
private:
    RoBSCOptions options;

    // Constant parameters
    Matrix3D InertialMatrix;
    Vector3D ExtForce;
//    Vector3D offset;
//    Vector3D iniOffset;
    Vector3D lastAcceleration;
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
    int count_yaw;
    double iniYawAngle;

    Timer integTimer;

	ros::Publisher CurPosPublisher;
	ros::Publisher DesPosPublisher;
	ros::Publisher CurVelPublisher;
	ros::Publisher CurAttPublisher;
	ros::Publisher WaypointPublisher;
	ros::NodeHandle mainNodehandle;

public:
	// gravity is provided by
    RoBSC();
    virtual ~RoBSC();

    void trajPlan(const double deltaT, const TKTrajectory& input, const TKState& currentState, RoBSCOutput& output, const sensor_msgs::Imu& unbiasedImu);
    
    void run(const TKTrajectory& input, const Matrix3D TrajReference, const TKState& currentState, const double mass, RoBSCOutput& output, const sensor_msgs::Imu& unbiasedImu, const geometry_msgs::Vector3Stamped& force_estimate, const geometry_msgs::Vector3Stamped& torque_estimate);
    

};

}

#endif /* TRO_ROBSC_HPP */
