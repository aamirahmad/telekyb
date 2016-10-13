/*
 * OutputControl.cpp
 *
 * A complete controller for MAV flight, including a real-time trajectory planner using model predictive control method,
 * a high-level position controller based on robust (against unknown disturbance) back-stepping method, and a low-level attitude controller based on output
 * regulation (geometric tracking) method.
 * NEED customized CVXGEN package to solve optimal control problem for trajectory planning.
 * The output of the controller is motor speed command.
 * Written by Yuyi Liu
 */

#include <tk_ctrlalgo/OutputControl.hpp>
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <telekyb_defines/physic_defines.hpp>  // Gravity Value 9.81
#include "TrajPlanning/trajplanner.h"    // MPC trajectory planner using CVXGEN to solve convex optimization problem

#define MAX_INT_TIME_STEP 0.02

namespace TELEKYB_NAMESPACE {

// Options
OutputControlOptions::OutputControlOptions()
    : OptionContainer("OutputControl")
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
    tEstGainX_disturbance = addOption<double>("tEstGainX_disturbance", "Disturbance Est Gain on x-axis", 1.0, false, false);
    tEstGainY_disturbance = addOption<double>("tEstGainY_disturbance", "Disturbance Est Gain on x-axis", 1.0, false, false);
    tEstGainZ_disturbance = addOption<double>("tEstGainZ_disturbance", "Disturbance Est Gain on x-axis", 1.6, false, false);

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
    tCostWeight_P = addOption<double>("tCostWeight_P", "Weight value for position state stage cost, L_stage(0,0)", 0.0, false, false);
    tCostWeight_V = addOption<double>("tCostWeight_V", "Weight value for velocity state stage cost, L_stage(1,0)", 0.0, false, false);
    tCostWeight_A = addOption<double>("tCostWeight_A", "Weight value for acceleration state stage cost, L_stage(2,0)", 0.0, false, false);
    tCostWeight_U = addOption<double>("tCostWeight_U", "Weight value for input stage cost, L_stage(0,1)", 0.0, false, false);
    tCostWeight_Pt = addOption<double>("tCostWeight_Pt", "Weight value for position term cost, L_stage(0,2)", 1.0, false, false);
    tCostWeight_Vt = addOption<double>("tCostWeight_Vt", "Weight value for velocity term cost, L_stage(1,2)", 1.0, false, false);
    tCostWeight_At = addOption<double>("tCostWeight_At", "Weight value for acceleration term cost, L_stage(2,2)", 1.0, false, false);

    //Switch "true" to using Geometric Tracking method @ GRASP for comparison
    tTestGRASP = addOption<bool>("tTestGRASP","Switch to Geometric Tracking method @ GRASP for comparison with true", false, false, false);
}

OutputControl::OutputControl()
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
    DesAttPublisher = mainNodehandle.advertise<geometry_msgs::Vector3>("/TeleKyb/TeleKybCore_240/DesAtt", 1);
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
    iniYawAngle = 0.0;
    // last LinVelocity
    lastPosition = Vector3D::Zero();
}

OutputControl::~OutputControl()
{

}

void OutputControl::run(const TKTrajectory& input, const TKState& currentState, const double mass, OutputCtrlOutput& output, const sensor_msgs::Imu& unbiasedImu)
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
    Acceleration3D desJerk = input.jerk;
    RotAngle3D desOrientation = input.rotangle;
    // initialize desired yaw angle into original      
//    if(options.tInitialYawAngle->getValue()){
//    	if (timer_pos == 0){
//		std::cout << "yaw angle initialize!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
//		iniYawAngle = curOrientation(2);
//		timer_pos = timer_pos + 1;
//    	}
//	desOrientation(2) = desOrientation(2) + iniYawAngle;
//    }

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

//std::cout << "if 0||curAtt: " << curOrientation(0) << ", "<< curOrientation(1) << ", "<< curOrientation(2) << std::endl;
//std::cout << "if 0||desAtt: " << desOrientation(0) << ", "<< desOrientation(1) << ", "<< desOrientation(2) << std::endl;
//std::cout << "if 0||curPos: " << curPosition(0) << ", "<< curPosition(1) << ", "<< curPosition(2) << std::endl;
//std::cout << "if 0||desPos: " << desPosition(0) << ", "<< desPosition(1) << ", "<< desPosition(2) << std::endl;

//	double DeltaTime = difTimer.getElapsed().toDSec();
//	difTimer.reset();
//	Acceleration3D curAcceleration = (curLinVelocity - lastLinVelocity) / DeltaTime;
//	lastLinVelocity = curLinVelocity;
//std::cout << "DeltaT: " << DeltaTime << "lastLinVelocity: " << lastLinVelocity << std::endl;
//std::cout << "curAcceleration: " << curAcceleration(0) << ", "<< curAcceleration(1) << ", "<< curAcceleration(2) << std::endl;
//    Acceleration3D curAcceleration = Vector3D::Zero();
    Acceleration3D curAcceleration( unbiasedImu.linear_acceleration.x, unbiasedImu.linear_acceleration.y, unbiasedImu.linear_acceleration.z - gravity);
//std::cout << "curAcceleration: " << curAcceleration(0) << ", "<< curAcceleration(1) << ", "<< curAcceleration(2) << std::endl;
    /****************************************************/
    // Back-stepping Controller for Position
    // Initialize gain parameters
    Matrix3D tPosGain_bkstp = Matrix3D::Zero(), tVelGain_bkstp = Matrix3D::Zero(), tEstGain_disturbance = Matrix3D::Zero();
    Vector3D dEstDisturbance(0.0, 0.0, 0.0);
    Matrix3D tOmegaGain_OR  = Matrix3D::Zero(), tRotGain_OR  = Matrix3D::Zero();
    double comThrust = iniThrust; double nominalThrust = iniThrust;
    Vector3D comTorque(0.0, 0.0, 0.0), StartPoint(0.0, 0.0, 1.00001);
    Matrix3D costWeight = Matrix3D::Zero();

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

        // Weight function of state stage cost, L_stage = diag([costWeight(0,0),costWeight(1,0),costWeight(2,0)])
        costWeight(0,0) = options.tCostWeight_P->getValue();
        costWeight(1,0) = options.tCostWeight_V->getValue();
        costWeight(2,0) = options.tCostWeight_A->getValue();
        // Weight function of input stage cost, R[0] =  costWeight(0,1)
        costWeight(0,1) = options.tCostWeight_U->getValue();
        //  Weight function of state terminal cost, L_term = diag([costWeight(0,2),costWeight(1,2),costWeight(2,2)])
        costWeight(0,2) = options.tCostWeight_Pt->getValue();
        costWeight(1,2) = options.tCostWeight_Vt->getValue();
        costWeight(2,2) = options.tCostWeight_At->getValue();

	// Compute body frame rotation matrix
	Matrix3D RotMatrix;
    RotMatrix << cos(curOrientation(2))*cos(curOrientation(1)), cos(curOrientation(2))*sin(curOrientation(0))*sin(curOrientation(1)) - cos(curOrientation(0))*sin(curOrientation(2)), sin(curOrientation(0))*sin(curOrientation(2)) + cos(curOrientation(0))*cos(curOrientation(2))*sin(curOrientation(1)),
                 cos(curOrientation(1))*sin(curOrientation(2)), cos(curOrientation(0))*cos(curOrientation(2)) + sin(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)), cos(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)) - cos(curOrientation(2))*sin(curOrientation(0)),
                 -sin(curOrientation(1)), cos(curOrientation(1))*sin(curOrientation(0)), cos(curOrientation(0))*cos(curOrientation(1));

    // Compute reference rotation matrix
    Matrix3D desMatrix;

    double timeStep = integTimer.getElapsed().toDSec();
    integTimer.reset();
    if (timeStep < MAX_INT_TIME_STEP){
        if (options.tOnlineTrajectoryPlanning->getValue()){
//            if (desPosition != StartPoint && timer_traj == 0){
            if (timer_traj == 0){
        // generate a set of desired states via a MPC trajectory planner
                TrajPlanner TrajPlan;
                double deltaT = 0.02;
                Eigen::Matrix<double,3,4> desState = Eigen::Matrix<double,3,4>::Zero();
//        std::cout << " *****Original desPosition: " << desPosition(0) << " , " << desPosition(1) << " , " << desPosition(2) << std::endl;
                desState = TrajPlan.TrajPlanDesigner(deltaT, curPosition, curLinVelocity, curAcceleration, desPosition, costWeight);
                desPosition = desState.col(0);
                desLinVelocity = desState.col(1);
                desAcceleration = desState.col(2);
//    std::cout << " MPC!!!!!! " << std::endl;
//    desJerk = desState.col(3);
//    std::cout << " curPosition: " << curPosition(0) << " , " << curPosition(1) << " , " << curPosition(2) << std::endl;
//    std::cout << " desPosition: " << desPosition(0) << " , " << desPosition(1) << " , " << desPosition(2) << std::endl;
//    std::cout << " desVelocity: " << desLinVelocity(0) << " , " << desLinVelocity(1) << " , " << desLinVelocity(2) << std::endl;
//    std::cout << " desAcceleration: " << desAcceleration(0) << " , " << desAcceleration(1) << " , " << desAcceleration(2) << std::endl;
//     std::cout << " desJerk: " <<  desState(0,3) << " , " <<  desState(1,3) << " , " <<  desState(2,3) << std::endl;
            }
        }
    }

//    integTimer.reset();
//    if (timer_pos == 2) {
//    if (timeStep < MAX_INT_TIME_STEP) {
//std::cout << " execute position control!!!!!!!!!! " << std::endl;
//	timer_pos = 0;
       // integTimer.reset();

        // Offset Estimator
//    Vector3D drift = (curPosition - lastPosition);
//    if ((curPosition - desPosition).norm() <= 0.05 && curLinVelocity.norm()<= 0.1){
//        if (drift(0)<= 0.01 && drift(1)<= 0.01 && drift(2)<= 0.01 ){
//            timer_pos = timer_pos + 1;
//            if (timer_pos==120 || timer_pos==180 || timer_pos==240 || timer_pos==300){
//                iniOffset << iniOffset(0) + curPosition(0) - desPosition(0), iniOffset(1) + curPosition(1) - desPosition(1), 0;
//                iniOffset = iniOffset/2;
//                offset = iniOffset;
//                std::cout << " Offset: " << offset(0) << " , " << offset(1) << " , " << offset(2) << std::endl;
//            }
//            else if (timer_pos >= 350 && timer_pos%60 == 0) {
//                offset(0) = iniOffset(0) + curPosition(0) - desPosition(0);
//                offset(1) = iniOffset(1) + curPosition(1) - desPosition(1);
//                offset(2) = iniOffset(2) + curPosition(2) - desPosition(2);
//            }
////            if(timer_pos==120 || (timer_pos >= 300 && timer_pos%40 == 0)){
////            offset(0) = 0.02 + curPosition(0) - desPosition(0);
////            offset(1) = -0.04 + curPosition(1) - desPosition(1);
////            offset(2) = 0.0 + curPosition(2) - desPosition(2);
////            }
//         }
//    }
//    lastPosition = curPosition;

    Vector3D PositionError = (curPosition - desPosition);
    Vector3D VelocityError = (curLinVelocity - desLinVelocity);
    Vector3D AccelerationError = (curAcceleration - desAcceleration);
//    if (PositionError.norm() >= 1.1){
//        PositionError = 0.7 * PositionError / PositionError.norm();
//    }
//    if (VelocityError.norm() >= 0.35){
//        VelocityError = 0.2 * VelocityError / VelocityError.norm();
//    }
//    if (AccelerationError.norm() >= 5.3){
//        AccelerationError = 3.0 * AccelerationError / AccelerationError.norm();
//    }
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
        Vector3D desThrust = -mass * ExtForce - mass * EstDisturbance + mass * desAcceleration - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError - mass * (2.0 * tVelGain_bkstp * tPosGain_bkstp + 2.0 * tPosGain_bkstp) * PositionError;
//        Vector3D desThrust = -mass * ExtForce + mass * desAcceleration - mass * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * VelocityError - mass * (2.0 * tPosGain_bkstp * tVelGain_bkstp + 2.0 * tPosGain_bkstp) * PositionError;
//        Vector3D desThrust = -1.0 * ExtForce -1.0 * (tVelGain_bkstp + 2.0 * tPosGain_bkstp) * curLinVelocity -
//                2.0 * tPosGain_bkstp * (tVelGain_bkstp + 1.0) * (curPosition - desPosition);

        // Compute nominal thrust value by f = norm(F,2)
        nominalThrust = desThrust.norm();

        Vector3D desAttitude2 = desThrust / nominalThrust;
        Vector3D desAttitude1 = RotMatrix.col(1) - RotMatrix.col(1).dot(desAttitude2)/desAttitude2.dot(desAttitude2) * desAttitude2;
        desAttitude1 = desAttitude1 / desAttitude1.norm();
        Vector3D desAttitude0 = RotMatrix.col(0) - RotMatrix.col(0).dot(desAttitude2)/desAttitude2.dot(desAttitude2) * desAttitude2 - RotMatrix.col(0).dot(desAttitude1)/desAttitude1.dot(desAttitude1) * desAttitude1;
        desAttitude0 = desAttitude0 / desAttitude0.norm();

        desMatrix.col(0) = desAttitude0;
        desMatrix.col(1) = desAttitude1;
        desMatrix.col(2) = desAttitude2;
        RefMatrix = desMatrix;
		
	// Compile nonlinear equation solver to compute desired orientation
           // Vector2D desAngle(0.0, 0.0);
          //  desAngle = AngleSolver.DesAngleSolver(curOrientation, desAttitude2D);
        // Reset desired orientation
         //   desOrientation(0) = desAngle(0);
         //   desOrientation(1) = desAngle(1);
//std::cout << "if 2.11||" << desAttitude2D(0) << ", " << desAttitude2D(1) << std::endl;
//std::cout << "if 2.12||" << desOrientation(0) << ", " << desOrientation(1) << ", " << desOrientation(2) << std::endl;
//        }
        // Aggressive maneuver mode. Inherit desired roll and pitch from reference.
//        else {
////std::cout << "if 2.2" << std::endl;
//            nominalThrust = 0.25 * mass * gravity;
//            RefMatrix << cos(desOrientation(2))*cos(desOrientation(1)), cos(desOrientation(2))*sin(desOrientation(0))*sin(desOrientation(1)) - cos(desOrientation(0))*sin(desOrientation(2)), sin(desOrientation(0))*sin(desOrientation(2)) + cos(desOrientation(0))*cos(desOrientation(2))*sin(desOrientation(1)),
//                     cos(desOrientation(1))*sin(desOrientation(2)), cos(desOrientation(0))*cos(desOrientation(2)) + sin(desOrientation(0))*sin(desOrientation(2))*sin(desOrientation(1)), cos(desOrientation(0))*sin(desOrientation(2))*sin(desOrientation(1)) - cos(desOrientation(2))*sin(desOrientation(0)),
//                     -sin(desOrientation(1)), cos(desOrientation(1))*sin(desOrientation(0)), cos(desOrientation(0))*cos(desOrientation(1));
//        }

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

//    if (timeStep < MAX_INT_TIME_STEP) {
//	timer_pos = timer_pos + 1;
//std::cout << "execute attitude control!!!!!!!!!! " << std::endl;
        // Initialize reference angular velocity
//    std::cout << "nominalThrust " << nominalThrust << std::endl;
    Velocity3D desAngVelocity = Vector3D::Zero();
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
        // Compute reference rotation matrix
 //       Matrix3D RefMatrix;
    //    RefMatrix << cos(desOrientation(2))*cos(desOrientation(1)), cos(desOrientation(2))*sin(desOrientation(0))*sin(desOrientation(1)) - cos(desOrientation(0))*sin(desOrientation(2)), sin(desOrientation(0))*sin(desOrientation(2)) + cos(desOrientation(0))*cos(desOrientation(2))*sin(desOrientation(1)),
    //                 cos(desOrientation(1))*sin(desOrientation(2)), cos(desOrientation(0))*cos(desOrientation(2)) + sin(desOrientation(0))*sin(desOrientation(2))*sin(desOrientation(1)), cos(desOrientation(0))*sin(desOrientation(2))*sin(desOrientation(1)) - cos(desOrientation(2))*sin(desOrientation(0)),
    //                 -sin(desOrientation(1)), cos(desOrientation(1))*sin(desOrientation(0)), cos(desOrientation(0))*cos(desOrientation(1));

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

        // Compute body frame rotation matrix
    //    Matrix3D RotMatrix;
      //  RotMatrix << cos(curOrientation(2))*cos(curOrientation(1)), cos(curOrientation(2))*sin(curOrientation(0))*sin(curOrientation(1)) - cos(curOrientation(0))*sin(curOrientation(2)), sin(curOrientation(0))*sin(curOrientation(2)) + cos(curOrientation(0))*cos(curOrientation(2))*sin(curOrientation(1)),
        //             cos(curOrientation(1))*sin(curOrientation(2)), cos(curOrientation(0))*cos(curOrientation(2)) + sin(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)), cos(curOrientation(0))*sin(curOrientation(2))*sin(curOrientation(1)) - cos(curOrientation(2))*sin(curOrientation(0)),
          //           -sin(curOrientation(1)), cos(curOrientation(1))*sin(curOrientation(0)), cos(curOrientation(0))*cos(curOrientation(1));

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
        comTorque = curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);

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
	}

	if (options.tPubAttitude->getValue()) {

        geometry_msgs::Vector3 CurAtt;
        CurAtt.x = curOrientation(0);
        CurAtt.y = curOrientation(1);
        CurAtt.z = curOrientation(2);
		CurAttPublisher.publish(CurAtt);

		geometry_msgs::Vector3 DesAtt;
		DesAtt.x = desOrientation(0);
		DesAtt.y = desOrientation(1);
		DesAtt.z = desOrientation(2);
		DesAttPublisher.publish(DesAtt);
	}

        timer_traj = timer_traj + 1;
        if(timer_traj == 2){
           timer_traj = 0;
        }


    // ----------------Motor commands ------------------//

    // Initialize motor speed command vector and force vector
    Vector4D motorBuf = Vector4D::Zero();
    Vector4D Force(comThrust, comTorque(0), comTorque(1), comTorque(2));

    Eigen::Matrix<double,6,1> motorBuf_Hex;
    motorBuf_Hex << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    if (options.tSimulationEuRoC->getValue()){
        Matrix4D CoeffMatrix;
        CoeffMatrix  << 1 / (motorConst * armLength), 0, 0, 0,
                        0, 1 / (motorConst * armLength), 0, 0,
                        0, 0, 1 /(motorConst*momentConst) , 0,
                        0, 0, 0, 1 / motorConst;
        Vector4D Force(comTorque(0), comTorque(1), comTorque(2), comThrust);
//          std::cout << "Nominal Force: " << Force(0) << " " << Force(1) << " " << Force(2) << " " << Force(3) << std::endl;
        motorBuf_Hex = inputTransMatrix_Hex.transpose()*(inputTransMatrix_Hex*inputTransMatrix_Hex.transpose()).inverse() * CoeffMatrix * Force;
//                std::cout << "00000Motor Speeds: " << motorBuf_Hex(0) << " " << motorBuf_Hex(1) << " " << motorBuf_Hex(2) << " " << motorBuf_Hex(3) << " " << motorBuf_Hex(4)<< " " << motorBuf_Hex(5) << std::endl;
        for (int count = 0; count < nrMotors_Hex; count++)
        {
            motorBuf_Hex(count) = std::max(motorBuf_Hex(count),0.0);
            motorBuf_Hex(count) = sqrt(motorBuf_Hex(count));
        }
//        std::cout << "Motor Speeds: " << motorBuf_Hex(0) << " " << motorBuf_Hex(1) << " " << motorBuf_Hex(2) << " " << motorBuf_Hex(3) << " " << motorBuf_Hex(4)<< " " << motorBuf_Hex(5) << std::endl;
    }

//std::cout << "forces: " << comThrust << ", " << comTorque(0) << ", " << comTorque(1) << ", " << comTorque(2) << std::endl;

    // If Plan B: [F,Tx,Ty,Tz]' becomes [-F,-Ty,-Tx,-Tz]
//    Vector4D Force(-comThrust, -comTorque(1), -comTorque(0), -comTorque(2));

    // Calculate motor speed from transmission matrix MotorSpeed^2 = G * Force
    // G is normalized with Thrust Coefficient Ct, thus very likely to add extra gain P = C
    motorBuf = PropellerGain * inputTransMatrix * Force;
//std::cout << "MotorForce: " << motorBuf(0) << ", " << motorBuf(1) << ", " << motorBuf(2) << ", " << motorBuf(3) << std::endl;

    // set the lower boundary of the nominal thrust on each motor as 0.1 N
    //motorBuf(0) = std::min(std::max(motorBuf(0), options.tMinMotorCMD->getValue()), options.tMaxMotorCMD->getValue());
    //motorBuf(1) = std::min(std::max(motorBuf(1), options.tMinMotorCMD->getValue()), options.tMaxMotorCMD->getValue());
    //motorBuf(2) = std::min(std::max(motorBuf(2), options.tMinMotorCMD->getValue()), options.tMaxMotorCMD->getValue());
    //motorBuf(3) = std::min(std::max(motorBuf(3), options.tMinMotorCMD->getValue()), options.tMaxMotorCMD->getValue());

//std::cout << "!!!Motor Forces: " << motorBuf(0) << " " << motorBuf(1) << " " << motorBuf(2) << " " << motorBuf(3) << std::endl;

    output.motorBuf(0) = motorBuf(0);
    output.motorBuf(1) = motorBuf(1);
    output.motorBuf(2) = motorBuf(2);
    output.motorBuf(3) = motorBuf(3);

    if (options.tSimulationEuRoC->getValue()){
        output.motorBuf_Hex(0) = motorBuf_Hex(0);
        output.motorBuf_Hex(1) = motorBuf_Hex(1);
        output.motorBuf_Hex(2) = motorBuf_Hex(2);
        output.motorBuf_Hex(3) = motorBuf_Hex(3);
        output.motorBuf_Hex(4) = motorBuf_Hex(4);
        output.motorBuf_Hex(5) = motorBuf_Hex(5);

//        std::cout << "!!!Motor Speed: " << output.motorBuf_Hex(0) << ", " << output.motorBuf_Hex(1) << ", " << output.motorBuf_Hex(2) << ", " << output.motorBuf_Hex(3) << ", " << output.motorBuf_Hex(4) << ", " << output.motorBuf_Hex(5) << std::endl;

//        output.NominalForce(0) = comThrust;
//        output.NominalForce(1) = comTorque(0);
//        output.NominalForce(2) = comTorque(1);
//        output.NominalForce(3) = comTorque(2);
    }

}


}
