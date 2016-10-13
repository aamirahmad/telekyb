/*
 * TRO_RoBSC.cpp
 *
 * A complete controller for MAV flight, including a real-time trajectory planner using model predictive control method,
 * a high-level position controller based on robust (against unknown disturbance) back-stepping method, and a low-level attitude controller based on output
 * regulation (geometric tracking) method.
 * NEED customized CVXGEN package to solve optimal control problem for trajectory planning.
 * The output of the controller is motor speed command.
 * Written by Yuyi Liu
 */

#include <tk_ctrlalgo/TRO_RoBSC.hpp>
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <telekyb_defines/physic_defines.hpp>  // Gravity Value 9.81
//#include "TrajPlanning/TrajOCPsolver.h"    //online MPC trajectory planner
#include "TrajPlanning/TrajOCPsolver.h"    //online MPC trajectory planner

#define MAX_INT_TIME_STEP 0.02

namespace TELEKYB_NAMESPACE {

// Options
RoBSCOptions::RoBSCOptions()
    : OptionContainer("RoBSC")
{
    // Physical constraints
    tMinThrust = addOption<double>( "tMinThrust", "Minimum thrust", 2.0, false, true);
    tMaxThrust = addOption<double>( "tMaxthrust", "Maximum thrust", 40.0, false, true);

    // Control gains in back-stepping regulator
    tPosGainX_bkstp = addOption<double>("tPosGainX_bkstp", "Gain of position cost in back-stepping regulator", 1.0, false, false);
    tPosGainY_bkstp = addOption<double>("tPosGainY_bkstp", "Gain of position cost in back-stepping regulator", 1.0, false, false);
    tPosGainZ_bkstp = addOption<double>("tPosGainZ_bkstp", "Gain of position cost in back-stepping regulator", 1.0, false, false);
    tVelGainX_bkstp = addOption<double>("tVelGainX_bkstp", "Gain of velocity in back-stepping regulator", 1.0, false, false);
    tVelGainY_bkstp = addOption<double>("tVelGainY_bkstp", "Gain of velocity in back-stepping regulator", 1.0, false, false);
    tVelGainZ_bkstp = addOption<double>("tVelGainZ_bkstp", "Gain of velocity in back-stepping regulator", 1.0, false, false);


    // Control parameters (Gains) for disturbance estimator
    tEstGainX_disturbance = addOption<double>("tEstGainX_disturbance", "Disturbance Est Gain on x-axis", 0.3, false, false);
    tEstGainY_disturbance = addOption<double>("tEstGainY_disturbance", "Disturbance Est Gain on x-axis", 0.3, false, false);
    tEstGainZ_disturbance = addOption<double>("tEstGainZ_disturbance", "Disturbance Est Gain on x-axis", 0.3, false, false);

    // Control gains in output regulator for attitude
    tOmegaGainX_OR = addOption<double>("tOmegaGainX_OR", "Gain of angular velocity in output regulator for attitude along x axis", 4.0, false, false);
    tOmegaGainY_OR = addOption<double>("tOmegaGainY_OR", "Gain of angular velocity in output regulator for attitude along y axis", 4.0, false, false);
    tOmegaGainZ_OR = addOption<double>("tOmegaGainZ_OR", "Gain of angular velocity in output regulator for attitude along z axis", 4.0, false, false);
    tRotGainX_OR = addOption<double>("tRotGainX_OR", "Gain of tracking rotation matrix in output regulator for attitude along x axis", 4.0, false, false);
    tRotGainY_OR = addOption<double>("tRotGainY_OR", "Gain of tracking rotation matrix in output regulator for attitude along y axis", 4.0, false, false);
    tRotGainZ_OR = addOption<double>("tRotGainZ_OR", "Gain of tracking rotation matrix in output regulator for attitude along z axis", 4.0, false, false);

    tGravity = addOption<double>( "tGravity", "Gravity Value for the Output Controller", GRAVITY, false, false);
    tArmLength = addOption<double>( "tArmLength", "Arm length for quadrotor MK Quad-XL", 0.29, false, false);

    txAxisInertial = addOption<double>("txAxisInertial", "Body-frame inertial value in x-axis", 0.027, false, false);
    tyAxisInertial = addOption<double>("tyAxisInertial", "Body-frame inertial value in y-axis", 0.026, false, false);
    tzAxisInertial = addOption<double>("tzAxisInertial", "Body-frame inertial value in z-axis", 0.047, false, false);

    tPropellerConst = addOption<double>("tPropellerThrustConst", "Thrust Constant Ct of propellers", 0.0131, false, false); // 10"4.5'EPP Ct 0.0000193815, Cq/Ct = 0.0131
    tPropellerGainConst = addOption<double>("tPropellerTorqueConst", "Torque Constant Cq of propellers", 1.0, false, false); // 10"4.5'EPP Cq 0.0000002688
    tMinMotorCMD = addOption<double>("tMinMotorCMD", "Minimum force of one motor on the quadrotor", 0.5, false, true);
    tMaxMotorCMD = addOption<double>("tMaxMotorCMD", "Maximum force of one motor on the quadrotor", 10.0, false, true);
    tPubAttitude = addOption<bool>("tPubAttitude","Publish current and desired attitude", false, false, false);
    tPubPosition = addOption<bool>("tPubPosition","Publish current and desired position", false, false, false);

    tInitialYawAngle = addOption<bool>("tInitialYawAngle","Initialize Yaw Angle", false, false, false);
    // Parameters for Hexarotor EuRoC challenge simulation
    tSimulationEuRoC = addOption<bool>("tSimulationEuRoC","Simulate the Hexacopter in EuRoC Challenge with true", false, false, false);
    tmotorConst = addOption<double>("tmotorConst", "rotor coefficient for Firefly Hexarotor", 0.00000854858, false, false); // 10"4.5'EPP Ct 0.0000193815, Cq/Ct = 0.0131
    tmomentConst = addOption<double>("tmomentConst", "moment coefficient for Firefly Hexarotor", 0.016, false, false); // 10"4.5'EPP Cq 0.0000002688

    // Parameters for online MPC trajectory planning algorithm
    tOnlineTrajectoryPlanning = addOption<bool>("tOnlineTrajectoryPlanning","Utilize an online MPC trajectory planner for quadrotor with true", false, false, false);
    tCost_Pxy_stage = addOption<double>("tCost_Pxy_stage", "Weight value for x,y position state stage cost, L_stage(0,0),L_stage(3,3)", 0.0, false, false);
    tCost_Vxy_stage = addOption<double>("tCost_Vxy_stage", "Weight value for x,y velocity state stage cost, L_stage(1,1), L_stage(4,4)", 0.0, false, false);
    tCost_Axy_stage = addOption<double>("tCost_Axy_stage", "Weight value for x,y acceleration state stage cost, L_stage(2,2), L_stage(5,5)", 0.0, false, false);
    tCost_Pz_stage = addOption<double>("tCost_Pz_stage", "Weight value for z position state stage cost, L_stage(0,0),L_stage(3,3)", 0.0, false, false);
    tCost_Vz_stage = addOption<double>("tCost_Vz_stage", "Weight value for z velocity state stage cost, L_stage(1,1), L_stage(4,4)", 0.0, false, false);
    tCost_Az_stage = addOption<double>("tCost_Az_stage", "Weight value for z acceleration state stage cost, L_stage(2,2), L_stage(5,5)", 0.0, false, false);
    tCost_Ux = addOption<double>("tCost_Ux", "Weight value for x input stage cost, R(0,0)", 0.01, false, false);
    tCost_Uy = addOption<double>("tCost_Uy", "Weight value for y input stage cost, R(1,1)", 0.01, false, false);
    tCost_Uz = addOption<double>("tCost_Uz", "Weight value for z input stage cost, R(2,2)", 0.01, false, false);
    tCost_Pxy_term = addOption<double>("tCost_Pxy_term", "Weight value for x,y position state terminal cost, L_stage(0,0),L_stage(3,3)", 20.0, false, false);
    tCost_Vxy_term = addOption<double>("tCost_Vxy_term", "Weight value for x,y velocity state terminal cost, L_stage(1,1), L_stage(4,4)", 5.0, false, false);
    tCost_Axy_term = addOption<double>("tCost_Axy_term", "Weight value for x,y acceleration state terminal cost, L_stage(2,2), L_stage(5,5)", 5.0, false, false);
    tCost_Pz_term = addOption<double>("tCost_Pz_term", "Weight value for z position state terminal cost, L_stage(0,0),L_stage(3,3)", 20.0, false, false);
    tCost_Vz_term = addOption<double>("tCost_Vz_term", "Weight value for z velocity state terminal cost, L_stage(1,1), L_stage(4,4)", 5.0, false, false);
    tCost_Az_term = addOption<double>("tCost_Az_term", "Weight value for z acceleration state terminal cost, L_stage(2,2), L_stage(5,5)", 5.0, false, false);
    

    //Switch "true" to using Geometric Tracking method @ GRASP for comparison
    tTestGRASP = addOption<bool>("tTestGRASP","Switch to Geometric Tracking method @ GRASP for comparison with true", false, false, false);

    //Switch "true" to turn on the force estimator
    tObserver = addOption<bool>("tObserver","Switch to turn on the force estimator with true", false, false, false);


}

RoBSC::RoBSC()
{
    // Initialize body frame inertial matrix for quadrotor
    InertialMatrix = Matrix3D::Zero();
    InertialMatrix(0,0) = options.txAxisInertial->getValue();
    InertialMatrix(1,1) = options.tyAxisInertial->getValue();
    InertialMatrix(2,2) = options.tzAxisInertial->getValue();

    RefMatrix = Matrix3D::Zero();

    PropellerConst = options.tPropellerConst->getValue();
    PropellerGain = options.tPropellerGainConst->getValue();

    motorConst = options.tmotorConst->getValue();
    momentConst = options.tmomentConst->getValue();

    // Initialize armlength for quadrotor
    armLength = options.tArmLength->getValue();

    // Describe external force vector
    gravity = options.tGravity->getValue();
    ExtForce = Vector3D::Zero();
    ExtForce(2) = -1.0 * gravity;
//    iniOffset = Vector3D::Zero();
//    offset = iniOffset;
    EstDisturbance = Vector3D::Zero();
//    dEstDisturbance = Vector3D::Zero();
  
    CurPosPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_240/CurPos", 1);  
    DesPosPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_240/DesPos", 1);
    CurAttPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_240/CurAtt", 1);
    WaypointPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_240/Waypoint", 1);
    CurVelPublisher = mainNodehandle.advertise<geometry_msgs::Twist>("/TeleKyb/TeleKybCore_240/CurVel", 1);

    // Initialize motor parameters and transform matrix
    nrMotors = 4;
    // [F,Tx,Ty,Tz]'= Ct * TransMatrix * [w1^2,w2^2,w3^2,w4^2]'
//    TransMatrix <<         1        ,      1         ,       1       ,      1        ,
//                           0        ,      0         , -armLength    , armLength     ,
//                     -armLength     , armLength      ,       0       ,      0        ,
//                     PropellerConst, PropellerConst, -PropellerConst, -PropellerConst;

    inputTransMatrix << 0.25, 0.0, -1/(2*armLength),  1/(4*PropellerConst),
                        0.25, 0.0,  1/(2*armLength),  1/(4*PropellerConst),
                        0.25, -1/(2*armLength), 0.0, -1/(4*PropellerConst),
                        0.25,  1/(2*armLength), 0.0, -1/(4*PropellerConst);

    nrMotors_Hex = 6;
    inputTransMatrix_Hex <<         0.5    ,  1,      0.5      ,     -0.5      , -1,      -0.5       ,
                            -0.866025403784,  0, 0.866025403784, 0.866025403784,  0, -0.866025403784 ,
                                    -1     ,  1,      -1       ,       1       , -1,         1       ,
                                     1     ,  1,       1       ,       1       ,  1,         1       ;

// Plan B: Using existing coordinate
//    inputTransMatrix << 0.25, 0.0, -1/(2*armLength),  1/(4*PropellerConst),
//                        0.25, 0.0,  1/(2*armLength),  1/(4*PropellerConst),
//                        0.25,  1/(2*armLength), 0.0, -1/(4*PropellerConst),
//                        0.25, -1/(2*armLength), 0.0, -1/(4*PropellerConst);

    MinMotorCMD = options.tMinMotorCMD->getValue();
    MaxMotorCMD = options.tMaxMotorCMD->getValue();

    MaxTorqueX = armLength * 0.25 * options.tMaxThrust->getValue();
    MaxTorqueY = armLength * 0.25 * options.tMaxThrust->getValue();
    MaxTorqueZ = PropellerConst * 0.5 * options.tMaxThrust->getValue();

    // time step recorder for position controller
    timer_traj = 0;
    // initialize comThrust
    iniThrust = 0.0;
    // initialize Yaw Angle
    count_yaw = 0;
    iniYawAngle = 0.0;
    // last LinVelocity
    lastAcceleration = Vector3D::Zero();
}

RoBSC::~RoBSC()
{

}

void RoBSC::trajPlan(const double deltaT, const TKTrajectory& input, const TKState& currentState, RoBSCOutput& output, const sensor_msgs::Imu& unbiasedImu)
{

    TrajOCPSolver trajPlanning;
    Vector3D curPosition = currentState.position;
    Vector3D curVelocity = currentState.linVelocity;
//     Vector3D curAcceleration( unbiasedImu.linear_acceleration.x, unbiasedImu.linear_acceleration.y, unbiasedImu.linear_acceleration.z - gravity);
   Vector3D curAcceleration(0.0, 0.0, 0.0);
   curAcceleration = lastAcceleration;
    curPosition(1) = -curPosition(1);
    curPosition(2) = -curPosition(2);
    curVelocity(1) = -curVelocity(1);
    curVelocity(2) = -curVelocity(2);
    Vector3D Waypoint = input.position;
    Waypoint(1) = -Waypoint(1);
    Waypoint(2) = -Waypoint(2);
    Eigen::Matrix<double, 3, 21> refPath = Eigen::Matrix<double, 3, 21>::Zero();
    Eigen::Matrix<double, 3, 5> CostFunction = Eigen::Matrix<double, 3, 5>::Zero();
    CostFunction(0,0) = options.tCost_Pxy_stage->getValue();
    CostFunction(1,0) = options.tCost_Vxy_stage->getValue();
    CostFunction(2,0) = options.tCost_Axy_stage->getValue();
    CostFunction(0,1) = options.tCost_Pz_stage->getValue();
    CostFunction(1,1) = options.tCost_Vz_stage->getValue();
    CostFunction(2,1) = options.tCost_Az_stage->getValue();    
    CostFunction(0,2) = options.tCost_Ux->getValue();
    CostFunction(1,2) = options.tCost_Uy->getValue();
    CostFunction(2,2) = options.tCost_Uz->getValue();    
    CostFunction(0,3) = options.tCost_Pxy_term->getValue();
    CostFunction(1,3) = options.tCost_Vxy_term->getValue();
    CostFunction(2,3) = options.tCost_Axy_term->getValue();
    CostFunction(0,4) = options.tCost_Pz_term->getValue();
    CostFunction(1,4) = options.tCost_Vz_term->getValue();
    CostFunction(2,4) = options.tCost_Az_term->getValue();    
    
//     std::cout << "---------------CurPos = ("<< curPosition(0) << ","<< curPosition(1) << ","<< curPosition(2) << ")----------------" << std::endl;
//     std::cout << "---------------Waypoint = ("<< Waypoint(0) << ","<< Waypoint(1) << ","<< Waypoint(2) << ")----------------" << std::endl;
        
//     output.Trajectory_Reference = trajPlanning.OCPsolDesigner(deltaT, curPosition, curVelocity, refPath, Waypoint, CostFunction); 
   output.Trajectory_Reference = trajPlanning.OCPsolDesigner(deltaT, curPosition, curVelocity, curAcceleration, refPath, Waypoint, CostFunction); 
   lastAcceleration = output.Trajectory_Reference.col(2);
    
//     std::cout << "********RefPos = ("<< output.Trajectory_Reference(0,0) << ","<< output.Trajectory_Reference(1,0) << ","<< output.Trajectory_Reference(2,0) << ")********" << std::endl;
}


void RoBSC::run(const TKTrajectory& input, const Matrix3D TrajReference, const TKState& currentState, const double mass, RoBSCOutput& output, const sensor_msgs::Imu& unbiasedImu, const geometry_msgs::Vector3Stamped& force_estimate, const geometry_msgs::Vector3Stamped& torque_estimate)
{
    // set nominal motor force in Ground Mode
    if (input.xAxisCtrl == PosControlType::Acceleration){

        output.motorBuf(0) = 0.5;
        output.motorBuf(1) = 0.5;
        output.motorBuf(2) = 0.5;
        output.motorBuf(3) = 0.5;
    if (options.tSimulationEuRoC->getValue()){
        output.motorBuf_Hex(0) = 15;
        output.motorBuf_Hex(1) = 15;
        output.motorBuf_Hex(2) = 15;
        output.motorBuf_Hex(3) = 15;
        output.motorBuf_Hex(4) = 15;
        output.motorBuf_Hex(5) = 15;
    }
    return;
    }
  
    time = ros::Time::now().toSec();
    double timeStep = integTimer.getElapsed().toDSec();
    integTimer.reset();

    //DesAngleSol AngleSolver;

    // Current State
    Position3D curPosition = currentState.position;	
    Velocity3D curLinVelocity = currentState.linVelocity;
    RotAngle3D curOrientation = currentState.getEulerRPY();
    AngVelocity3D curAngVelocity = currentState.angVelocity;
    // Desired State
    Position3D desPosition = input.position;
    Velocity3D desLinVelocity = input.velocity;
    Acceleration3D desAcceleration = input.acceleration;
    RotAngle3D desOrientation = input.rotangle;
  
//     initialize desired yaw angle into original      
   if(options.tInitialYawAngle->getValue()){
   	if (count_yaw == 0){
		std::cout << "Yaw Angle Initialized!" << std::endl;
		iniYawAngle = curOrientation(2);
		count_yaw = 1;
   	}
	desOrientation(2) = desOrientation(2) + iniYawAngle;
   }

	// from NED to NWU 
    curPosition(1)=-curPosition(1);
    curPosition(2)=-curPosition(2);	
    curLinVelocity(1)=-curLinVelocity(1);
    curLinVelocity(2)=-curLinVelocity(2);
    curOrientation(1)=-curOrientation(1);
    curOrientation(2)=-curOrientation(2);
    curAngVelocity(1)=-curAngVelocity(1);
    curAngVelocity(2)=-curAngVelocity(2);

	// from NED to NWU 
    desPosition(1)=-desPosition(1);
    desPosition(2)=-desPosition(2);
    desLinVelocity(1)=-desLinVelocity(1);
    desLinVelocity(2)=-desLinVelocity(2);
    desOrientation(1)=-desOrientation(1);
    desOrientation(2)=-desOrientation(2);

    desAcceleration(1)=-desAcceleration(1);
    desAcceleration(2)=-desAcceleration(2);
    
    Vector3D Distance = (desPosition - curPosition);
    
    if (options.tOnlineTrajectoryPlanning->getValue()){
      if (desLinVelocity.norm() == 0 && Distance.norm() > 0.3 && TrajReference(2,0) > 0){
//       if (desLinVelocity.norm() == 0 && TrajReference(2,0) > 0){
	  desPosition = TrajReference.col(0);
	  desLinVelocity = TrajReference.col(1);
	  desAcceleration = TrajReference.col(2);
// 	  std::cout << "TrajPlanning!! " << std::endl;
// 	  std::cout << "Reference = ("<< desPosition(0) << ","<< desPosition(1) << ","<< desPosition(2) << ")" << std::endl;
      }
    }

   Acceleration3D curAcceleration = Vector3D::Zero();
//     Acceleration3D curAcceleration( unbiasedImu.linear_acceleration.x, unbiasedImu.linear_acceleration.y, unbiasedImu.linear_acceleration.z - gravity);
//std::cout << "curAcceleration: " << curAcceleration(0) << ", "<< curAcceleration(1) << ", "<< curAcceleration(2) << std::endl;


    /****************************************************/
    // Back-stepping Controller for Position
    // Initialize gain parameters
    Matrix3D tPosGain_bkstp = Matrix3D::Zero(), tVelGain_bkstp = Matrix3D::Zero(), tEstGain_disturbance = Matrix3D::Zero();
    Vector3D dEstDisturbance(0.0, 0.0, 0.0);
    Matrix3D tOmegaGain_OR  = Matrix3D::Zero(), tRotGain_OR  = Matrix3D::Zero();
    double comThrust = iniThrust; double nominalThrust = iniThrust;
    Vector3D comTorque(0.0, 0.0, 0.0), StartPoint(0.0, 0.0, 1.00001);

        tPosGain_bkstp(0,0) = options.tPosGainX_bkstp->getValue();
        tPosGain_bkstp(1,1) = options.tPosGainY_bkstp->getValue();
        tPosGain_bkstp(2,2) = options.tPosGainZ_bkstp->getValue();
        tVelGain_bkstp(0,0) = options.tVelGainX_bkstp->getValue();
        tVelGain_bkstp(1,1) = options.tVelGainY_bkstp->getValue();
        tVelGain_bkstp(2,2) = options.tVelGainZ_bkstp->getValue();
        tEstGain_disturbance(0,0) = options.tEstGainX_disturbance->getValue();
        tEstGain_disturbance(1,1) = options.tEstGainY_disturbance->getValue();
        tEstGain_disturbance(2,2) = options.tEstGainZ_disturbance->getValue();
        tOmegaGain_OR(0,0) = options.tOmegaGainX_OR->getValue();
        tOmegaGain_OR(1,1) = options.tOmegaGainY_OR->getValue();
        tOmegaGain_OR(2,2) = options.tOmegaGainZ_OR->getValue();
        tRotGain_OR(0,0) = options.tRotGainX_OR->getValue();
        tRotGain_OR(1,1) = options.tRotGainY_OR->getValue();
        tRotGain_OR(2,2) = options.tRotGainZ_OR->getValue();

	// Compute body frame rotation matrix
	Matrix3D RotMatrix;
    RotMatrix << cos(curOrientation(2))*cos(curOrientation(1)), cos(curOrientation(2))*sin(curOrientation(0))*sin(curOrientation(1)) - cos(curOrientation(0))*sin(curOrientation(2)), sin(curOrientation(0))*sin(curOrientation(2)) + cos(curOrientation(0))*cos(curOrientation(2))*sin(curOrientation(1)),
                 cos(curOrientation(1))*sin(curOrientation(2)), cos(curOrientation(0))*cos(curOrientation(2)) + sin(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)), cos(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)) - cos(curOrientation(2))*sin(curOrientation(0)),
                 -sin(curOrientation(1)), cos(curOrientation(1))*sin(curOrientation(0)), cos(curOrientation(0))*cos(curOrientation(1));
    
    Vector3D PositionError = (curPosition - desPosition);
    Vector3D VelocityError = (curLinVelocity - desLinVelocity);
    Vector3D AccelerationError = (curAcceleration - desAcceleration);

        // Initialize timestep for disturbance estimator
        if (timeStep < MAX_INT_TIME_STEP) {
            if (!options.tTestGRASP->getValue()){
                if (desPosition == StartPoint){
                dEstDisturbance = tEstGain_disturbance * VelocityError + 2.0 * tEstGain_disturbance * tPosGain_bkstp * PositionError;
                EstDisturbance = EstDisturbance + dEstDisturbance * timeStep;
                }
            }
        }
//        std::cout << " timeStep: " << timeStep << " , dEstDisturbance: " << dEstDisturbance << " , EstDisturbance: " << EstDisturbance << std::endl;
//        std::cout << " timeStep: " << timeStep << std::endl;
        // Compute desired thrust vector (3x1) based on back-stepping method
 //        Vector3D desThrust = -mass * ExtForce + mass * desAcceleration - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError - mass * (2.0 * tPosGain_bkstp * tVelGain_bkstp + 2.0 * tPosGain_bkstp) * PositionError;
//        Vector3D desThrust = -1.0 * ExtForce -1.0 * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * curLinVelocity -
//                2.0 * tPosGain_bkstp * (tVelGain_bkstp + 1.0) * (curPosition - desPosition);


//        Vector3D desThrust = -mass * ExtForce - mass * EstDisturbance + mass * desAcceleration - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError - mass * (2.0 * tVelGain_bkstp * tPosGain_bkstp + 2.0 * tPosGain_bkstp) * PositionError;

        Vector3D force_disturbance(0.0,0.0,0.0);
        if (options.tObserver->getValue() && curPosition(2)>=0.35){
        force_disturbance(0) = force_estimate.vector.x;
        force_disturbance(1) = force_estimate.vector.y;
        force_disturbance(2) = force_estimate.vector.z;
        }

        Vector3D desThrust =  -force_disturbance -mass * ExtForce - mass * EstDisturbance + mass * desAcceleration - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError - mass * (2.0 * tVelGain_bkstp * tPosGain_bkstp + 2.0 * tPosGain_bkstp) * PositionError;



        // Compute nominal thrust value by f = norm(F,2)
        nominalThrust = desThrust.norm();

        Vector3D desAttitude2 = desThrust / nominalThrust;
        Vector3D desAttitude1 = RotMatrix.col(1) - RotMatrix.col(1).dot(desAttitude2)/desAttitude2.dot(desAttitude2) * desAttitude2;
        desAttitude1 = desAttitude1 / desAttitude1.norm();
        Vector3D desAttitude0 = RotMatrix.col(0) - RotMatrix.col(0).dot(desAttitude2)/desAttitude2.dot(desAttitude2) * desAttitude2 - RotMatrix.col(0).dot(desAttitude1)/desAttitude1.dot(desAttitude1) * desAttitude1;
        desAttitude0 = desAttitude0 / desAttitude0.norm();

	// Compute reference rotation matrix
	Matrix3D desMatrix;

        desMatrix.col(0) = desAttitude0;
        desMatrix.col(1) = desAttitude1;
        desMatrix.col(2) = desAttitude2;
        RefMatrix = desMatrix;
		
        // Compute thrust command by U1 = m * f;
    comThrust = nominalThrust;
	iniThrust = comThrust;
//std::cout << "Reference Matrix!!!!: <<  "<< RefMatrix << std::endl;
//    }
    // Set thrust constraints, from 0 to 40 N
    comThrust = std::max( comThrust, options.tMinThrust->getValue() );
    comThrust = std::min( comThrust, options.tMaxThrust->getValue() );
//    std::cout << "comThrust!!!!!!!!!! " << comThrust << std::endl;






    /****************************************************/
    // Low-level Output Regulator for Attitude

        // Initialize reference angular velocity
//    std::cout << "nominalThrust " << nominalThrust << std::endl;
    Velocity3D desAngVelocity = Vector3D::Zero();
    Vector3D desJerk = Vector3D::Zero();
    if (!options.tTestGRASP->getValue()){
     if (nominalThrust > 0.0){
        Vector3D ddesThrust = -mass * dEstDisturbance + mass * desJerk - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * AccelerationError -
                            mass * (2.0 * tPosGain_bkstp * tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError;
//        Vector3D ddesThrust = -mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * AccelerationError -
//                            mass * (2.0 * tPosGain_bkstp * tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError;
        Vector3D R1dF = (RefMatrix.transpose()* ddesThrust) / nominalThrust;
        desAngVelocity(0) = -R1dF(1);
        desAngVelocity(1) = R1dF(0);
        desAngVelocity(2) = (desOrientation(2) - curOrientation(2));
//         desAngVelocity(2) = 0;
     }
    }else{
//        desAngVelocity(0) = (desOrientation(0) - curOrientation(0));
//        desAngVelocity(1) = (desOrientation(1) - curOrientation(1));
        desAngVelocity(2) = (desOrientation(2) - curOrientation(2));
    }
 
        // Generate skew-symmetric matrix for desired and current angular velocity
        Matrix3D Qomega_ref;
        Qomega_ref << 0, -desAngVelocity(2), desAngVelocity(1),
                      desAngVelocity(2), 0, -desAngVelocity(0),
                      -desAngVelocity(1), desAngVelocity(0), 0;
        Matrix3D dRefMatrix = RefMatrix * Qomega_ref;

        Matrix3D Qomega_cur;
        Qomega_cur << 0, -curAngVelocity(2), curAngVelocity(1),
                      curAngVelocity(2), 0, -curAngVelocity(0),
                      -curAngVelocity(1), curAngVelocity(0), 0;
        Matrix3D dRotMatrix = RotMatrix * Qomega_cur;

  
        // Compute inertial matrix * current angular velocity
//        Vector3D InertialcurAngVelocity = InertialMatrix * curAngVelocity;
        // Compute body frame angular momentum of current state omega2 x Jb*omega2
        Vector3D curAngMomentum = curAngVelocity.cross(InertialMatrix * curAngVelocity);

//        //Compute attitude error matrix tracking reference and current rotation matrice
//        Matrix3D ErrRefRot = RefMatrix * RotMatrix.transpose();
//        Matrix3D ErrRotRef = RotMatrix * RefMatrix.transpose();
//        Matrix3D ErrorMatrix = ErrRefRot - ErrRotRef + ErrRotRef * Qomega_ref * ErrRefRot;
//        // Compute the inverse mapping of error matrix
//        Vector3D ErrorAngVelocity(ErrorMatrix(2,1),ErrorMatrix(0,2),ErrorMatrix(1,0));
//        // Compute motor torque command along three axis
//        comTorque = curAngMomentum - tRotGain_OR * InertialcurAngVelocity + tOmegaGain_OR * InertialMatrix * ErrorAngVelocity;

     if (!options.tTestGRASP->getValue()){
//      \nabla P(E)=1/2*(E-E')E
//        Matrix3D ErrOmega = Qomega_cur - Qomega_ref + 0.5*tRotGain_OR * (RefMatrix.transpose()*RotMatrix - RotMatrix.transpose()*RefMatrix);
//        Matrix3D ErrorMatrix = - tOmegaGain_OR*ErrOmega - 0.5*tRotGain_OR*(dRefMatrix.transpose()*RotMatrix + RefMatrix.transpose()*dRotMatrix - dRotMatrix.transpose()*RefMatrix - RotMatrix.transpose()*dRefMatrix) - dRefMatrix.transpose()*dRefMatrix - 2*RefMatrix.transpose()*RotMatrix;
//        Vector3D ErrorAngVelocity(ErrorMatrix(2,1),ErrorMatrix(0,2),ErrorMatrix(1,0));

        // \nabla P(E)=(E'-E)E
//        Matrix3D ErrOmega = Qomega_cur + dRefMatrix.transpose()*RefMatrix + tRotGain_OR * (RotMatrix.transpose()*RefMatrix - RefMatrix.transpose()*RotMatrix);
//        Matrix3D ErrorMatrix = - tOmegaGain_OR*ErrOmega - tRotGain_OR*(-dRefMatrix.transpose()*RotMatrix - RefMatrix.transpose()*dRotMatrix + dRotMatrix.transpose()*RefMatrix + RotMatrix.transpose()*dRefMatrix) - dRefMatrix.transpose()*dRefMatrix - 2*RefMatrix.transpose()*RotMatrix;
//        Vector3D ErrorAngVelocity(ErrorMatrix(2,1),ErrorMatrix(0,2),ErrorMatrix(1,0));

        // Compute motor torque command along three axis
//        comTorque = curAngMomentum + InertialMatrix * ErrorAngVelocity;

        Vector3D ErrOmega = curAngVelocity - RotMatrix.transpose() * RefMatrix * desAngVelocity;
        Matrix3D ErrorMatrix = 0.5*(RefMatrix.transpose()*RotMatrix - RotMatrix.transpose()*RefMatrix);
        Vector3D ErrorRotation(ErrorMatrix(2,1),ErrorMatrix(0,2),0);
//        comTorque = curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);


        Vector3D torque_disturbance(0.0,0.0,0.0);
        if (options.tObserver->getValue() && curPosition(2)>=0.35){
        torque_disturbance(0) = torque_estimate.vector.x;
	torque_disturbance(1) = torque_estimate.vector.y;
        torque_disturbance(2) = torque_estimate.vector.z;
        }
        comTorque = -torque_disturbance + curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);


    }else{
         Vector3D ErrOmega = curAngVelocity - RotMatrix.transpose() * RefMatrix * desAngVelocity;
         Matrix3D ErrorMatrix = 0.5*(RefMatrix.transpose()*RotMatrix - RotMatrix.transpose()*RefMatrix);
         Vector3D ErrorRotation(ErrorMatrix(2,1),ErrorMatrix(0,2),0);

//         comTorque = curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);
         comTorque = curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation;
//         std::cout << "Using Geometric Tracking !!!------2" << std::endl;
     }
	//std::cout << "forces00||: " << comThrust << ", " << comTorque(0) << ", " << comTorque(1) << ", " << comTorque(2) << std::endl;

	// Set torque constraints: abs(tau_x)<=0.25*d*MaxThrust;abs(tau_y)<=0.25*d*MaxThrust;abs(tau_z)<=Cq/Ct*2*0.25*MaxThrust;
//    comTorque(0) = std::max( comTorque(0), -1 * MaxTorqueX );
//    comTorque(0) = std::min( comTorque(0), MaxTorqueX );
//    comTorque(1) = std::max( comTorque(1), -1 * MaxTorqueY );
//    comTorque(1) = std::min( comTorque(1), MaxTorqueY );
//    comTorque(2) = std::max( comTorque(2), -1 * MaxTorqueZ );
//    comTorque(2) = std::min( comTorque(2), MaxTorqueZ );

	// Publish desired and current position and attitude data
	if (options.tPubPosition->getValue()) {
		geometry_msgs::Vector3 CurPos;
		CurPos.x = curPosition(0);
		CurPos.y = curPosition(1);
		CurPos.z = curPosition(2);
		CurPosPublisher.publish(CurPos);
		geometry_msgs::Vector3 DesPos;
		DesPos.x = desPosition(0);
		DesPos.y = desPosition(1);
		DesPos.z = desPosition(2);
		DesPosPublisher.publish(DesPos);

        geometry_msgs::Twist CurVel;
        CurVel.linear.x = curLinVelocity(0);
        CurVel.linear.y = curLinVelocity(1);
        CurVel.linear.z = curLinVelocity(2);
        CurVel.angular.x = curAngVelocity(0);
        CurVel.angular.y = curAngVelocity(1);
        CurVel.angular.z = curAngVelocity(2);
        CurVelPublisher.publish(CurVel);
	
	
	geometry_msgs::Vector3 Goal;
	Goal.x = input.position(0);
	Goal.y = -input.position(1);
	Goal.z = -input.position(2);
	WaypointPublisher.publish(Goal);
	}

	if (options.tPubAttitude->getValue()) {

        geometry_msgs::Vector3 CurAtt;
        CurAtt.x = curOrientation(0);
        CurAtt.y = curOrientation(1);
        CurAtt.z = curOrientation(2);
	CurAttPublisher.publish(CurAtt);

	}

        timer_traj = timer_traj + 1;
        if(timer_traj == 2){
           timer_traj = 0;
        }


    // ----------------Motor commands ------------------//

    // Initialize motor speed command vector and force vector
    Vector4D motorBuf = Vector4D::Zero();
    Vector4D Force(comThrust, comTorque(0), comTorque(1), comTorque(2));
    motorBuf = PropellerGain * inputTransMatrix * Force;

    output.motorBuf(0) = motorBuf(0);
    output.motorBuf(1) = motorBuf(1);
    output.motorBuf(2) = motorBuf(2);
    output.motorBuf(3) = motorBuf(3);


    Eigen::Matrix<double,6,1> motorBuf_Hex;
    motorBuf_Hex << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    if (options.tSimulationEuRoC->getValue()){
        Matrix4D CoeffMatrix;
        CoeffMatrix  << 1 / (motorConst * armLength), 0, 0, 0,
                        0, 1 / (motorConst * armLength), 0, 0,
                        0, 0, 1 /(motorConst*momentConst) , 0,
                        0, 0, 0, 1 / motorConst;
        Vector4D Force(comTorque(0), comTorque(1), comTorque(2), comThrust);
    // Calculate motor speed from transmission matrix MotorSpeed^2 = G * Force
    // G is normalized with Thrust Coefficient Ct, thus very likely to add extra gain P = C
	motorBuf_Hex = inputTransMatrix_Hex.transpose()*(inputTransMatrix_Hex* inputTransMatrix_Hex.transpose()).inverse() * CoeffMatrix * Force;

	for (int count = 0; count < nrMotors_Hex; count++)
        {
            motorBuf_Hex(count) = std::max(motorBuf_Hex(count),0.0);
            motorBuf_Hex(count) = sqrt(motorBuf_Hex(count));
        }
	output.motorBuf_Hex(0) = motorBuf_Hex(0);
        output.motorBuf_Hex(1) = motorBuf_Hex(1);
        output.motorBuf_Hex(2) = motorBuf_Hex(2);
        output.motorBuf_Hex(3) = motorBuf_Hex(3);
        output.motorBuf_Hex(4) = motorBuf_Hex(4);
        output.motorBuf_Hex(5) = motorBuf_Hex(5);
//        output.NominalForce(0) = comThrust;
//        output.NominalForce(1) = comTorque(0);
//        output.NominalForce(2) = comTorque(1);
//        output.NominalForce(3) = comTorque(2);    
    }


}


}
