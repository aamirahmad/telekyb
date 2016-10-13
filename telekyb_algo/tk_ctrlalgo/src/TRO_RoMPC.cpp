/*
 * TRO_RoMPC.cpp
 *
 * A tube-based robust MPC controller for nontrivial MAV maneuvers and obstacle avoidance,
 * including a real-time sub-target obstacle avoidance algorithm,
 * a high-level position controller using a tube-based model predictive control method,
 * and a low-level attitude controller based on global output regulation method.
 * NEED customized CVXGEN package to solve 3D optimal control problem for model predictive control.
 * The output of the controller is motor speed command.
 * Written by Yuyi Liu, August 2015.
 */

#include <tk_ctrlalgo/TRO_RoMPC.hpp>
#include <algorithm>
#include <geometry_msgs/Vector3.h>
#include <telekyb_defines/physic_defines.hpp>  // Gravity Value 9.81
//#include "MPC/OCPsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC
#include "RoMPC_20step/OCPsolver.h"    // To import CVXGEN to solve Convex OCP for NOMINAL MPC

#define FIXED_TIME_STEP 0.02

namespace TELEKYB_NAMESPACE {

// Options
RoMPCOptions::RoMPCOptions()
    : OptionContainer("RoMPC")
{
    // Physical constraints
    tMinThrust = addOption<double>( "tMinThrust", "Minimum thrust", 1.0, false, true);
    tMaxThrust = addOption<double>( "tMaxthrust", "Maximum thrust", 40.0, false, true);

    // Weight values for cost function of the NOMINAL MPC position controller
    tCostWeight_pLstagexy = addOption<double>("tCostWeight_pLstagexy", "Weight value for position state stage cost 6x6, L_stage(0,0;2,2)", 5.0, false, false);
    tCostWeight_pLstagez = addOption<double>("tCostWeight_pLstagez", "Weight value for position state stage cost 6x6, L_stage(4,4)", 8.0, false, false);
    tCostWeight_vLstage = addOption<double>("tCostWeight_vLstage", "Weight value for velocity state stage cost 6x6, L_stage(1,1;3,3;5,5)", 0.0, false, false);
    tCostWeight_Rxy = addOption<double>("tCostWeight_Rxy", "Weight value (horizontal) for input stage cost 3x3, L_stage(0,0;1,1)", 0.2, false, false);
    tCostWeight_Rz = addOption<double>("tCostWeight_Rz", "Weight value (vertical) for input stage cost 3x3, L_stage(2,2)", 0.1, false, false);
    tCostWeight_pLtermxy = addOption<double>("tCostWeight_pLtermxy", "Weight value for position terminal cost 6x6, L_term(0,0;2,2;4,4)", 10.0, false, false);
    tCostWeight_pLtermz = addOption<double>("tCostWeight_pLtermz", "Weight value for position terminal cost 6x6, L_term(4,4)", 15.0, false, false);
    tCostWeight_vLterm = addOption<double>("tCostWeight_vLterm", "Weight value for velocity terminal cost 6x6, L_term(1,1;3,3;5,5)", 2.0, false, false);

    // Parameters of the LQR stablizing gain for tube-MPC
    tLqrGain_pxy = addOption<double>( "tLqrGain_pxy", "Horizontal position gain for LQR Gain Matrix K_LQR 3x6, K_LQR(0,0;1,2)", -7.0711, false, false);
    tLqrGain_pz = addOption<double>( "tLqrGain_pz", "Vertical position gain for LQR Gain Matrix K_LQR 3x6, K_LQR(2,4)", -7.0711, false, false);
    tLqrGain_vxy = addOption<double>( "tLqrGain_vxy", "Horizontal velocity gain for LQR Gain Matrix K_LQR 3x6, K_LQR(0,1;1,3)", -4.9135, false, false);
    tLqrGain_vz = addOption<double>( "tLqrGain_vz", "Vertical velocity gain for LQR Gain Matrix K_LQR 3x6, K_LQR(2,5)", -4.9135, false, false);

    // Control gains in output regulator for attitude
    tOmegaGainX_OR = addOption<double>("tOmegaGainX_OR", "Gain of angular velocity in output regulator for attitude along x axis", 0.5, false, false);
    tOmegaGainY_OR = addOption<double>("tOmegaGainY_OR", "Gain of angular velocity in output regulator for attitude along y axis", 0.5, false, false);
    tOmegaGainZ_OR = addOption<double>("tOmegaGainZ_OR", "Gain of angular velocity in output regulator for attitude along z axis", 0.4, false, false);
    tRotGainX_OR = addOption<double>("tRotGainX_OR", "Gain of tracking rotation matrix in output regulator for attitude along x axis", 3.5, false, false);
    tRotGainY_OR = addOption<double>("tRotGainY_OR", "Gain of tracking rotation matrix in output regulator for attitude along y axis", 3.5, false, false);
    tRotGainZ_OR = addOption<double>("tRotGainZ_OR", "Gain of tracking rotation matrix in output regulator for attitude along z axis", 1.0, false, false);

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

    //Switch "true" to import known obstacle position
    tObstacleAvoidance = addOption<bool>("tObstacleAvoidance","Switch to import known obstacle position with true", false, false, false);
//    ObstaclePosition = addOption<Eigen::Matrix<double,2,3>>("ObstaclePosition", "Obstacle Positions, (x1,y1,z1;x2,y2,z2;x3,y3,z3;...)", Eigen::Matrix<double,2,3>(0.2,1.2,1.0,-0.5,1.5,1.0), false, false);
    tRadiusObstacle = addOption<double>( "tRadiusObstacle", "Assumed radius of obstacle [m]", 0.3, false, false);
    tRadiusSafety = addOption<double>( "tRadiusSafety", "Assumed safe radius of multi-rotor [m]", 0.5, false, false);

    //Switch "true" to turn on the force estimator
    tObserver = addOption<bool>("tObserver","Switch to turn on the force estimator with true", false, false, false);

}

RoMPC::RoMPC()
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

    MinMotorCMD = options.tMinMotorCMD->getValue();
    MaxMotorCMD = options.tMaxMotorCMD->getValue();

    // time step recorder for position controller
    timer_subtarget = 0;
    // initialize comThrust
    iniThrust = 0.0;
    // initialize Yaw Angle
    iniYawAngle = 0.0;
    timer_initialYaw = 0;
    // nominal state at t+1 step computed at the last timestep
    NextState_last = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>::Zero();
    // read obstacle positions and array as Matrix (3 x nObstacle)
    ObstaclePosition << 0.8,0.8,1.6,
                        1.4,0.8,1.6,
                        0.8,1.6,1.6,
                        1.4,1.6,1.6;
    Obstacle = ObstaclePosition.transpose();
    // record the sub-target position computed at the last timestep
    sub_terminal_last = Vector3D::Zero();

}

RoMPC::~RoMPC()
{

}

void RoMPC::run_pos(const TKTrajectory& input, const TKState& currentState, const double mass, RoMPCOutput& output, const sensor_msgs::Imu& unbiasedImu, const geometry_msgs::Vector3Stamped& force_estimate)
{

    time = ros::Time::now().toSec();

    // Current State
    Position3D curPosition = currentState.position;
    Velocity3D curLinVelocity = currentState.linVelocity;
    Acceleration3D curAcceleration( unbiasedImu.linear_acceleration.x, unbiasedImu.linear_acceleration.y, unbiasedImu.linear_acceleration.z - gravity);

    // Desired State
    Position3D desPosition = input.position;
    Velocity3D desLinVelocity = input.velocity;
    Acceleration3D desAcceleration = input.acceleration;


    // from NED to NWU
    curPosition(1)=-curPosition(1);
    curPosition(2)=-curPosition(2);
    curLinVelocity(1)=-curLinVelocity(1);
    curLinVelocity(2)=-curLinVelocity(2);


    // from NED to NWU
    desPosition(1)=-desPosition(1);
    desPosition(2)=-desPosition(2);
    desLinVelocity(1)=-desLinVelocity(1);
    desLinVelocity(2)=-desLinVelocity(2);
    desAcceleration(1)=-desAcceleration(1);
    desAcceleration(2)=-desAcceleration(2);

//        std::cout << " Desired Height:  "<< desPosition(2) << std::endl;



    /****************************************************/
    // A tube-based MPC Controller for Position and Obstacle Avoidance


    // Sub-target algorithm for obstacle avoidance
    Position3D sub_terminal =  desPosition;
    if (options.tObstacleAvoidance->getValue() && desPosition(2)!=1.00001){
    if (timer_subtarget == 2){
        // TO DO!! 2D sub-target algorithm
        int nObstacle = Obstacle.cols();
        int nCollision = 0;
        int nBlockObj = 0;
        double RadiusObstacle = options.tRadiusObstacle->getValue();
        double RadiusSafety = options.tRadiusSafety->getValue();
        Eigen::Matrix<double, 3, Eigen::Dynamic> Collision(3,1);
        Collision << 1000, 0, -1;
        Vector3D TotalDistance;
        TotalDistance = desPosition - curPosition;
        TotalDistance(2) = 0.0; //set as 2D, with no distance on z axis
        double TotalDistanceNorm = std::max(0.000001,TotalDistance.norm());
        Eigen::Matrix<double, 1, Eigen::Dynamic> aa(nObstacle);
        Eigen::Matrix<double, 3, Eigen::Dynamic> db(3,nObstacle);
        Eigen::Matrix<double, 1, Eigen::Dynamic> bb(nObstacle);
        Vector3D db_countB;

        Eigen::Matrix<double, 2, Eigen::Dynamic> collDistance(2,1);
        collDistance << 0.0, 0.0;
        double ObsHeight = Obstacle.row(2).maxCoeff();

//        std::cout << " **** Debug 1.1 **** DisNorm =  "<< TotalDistanceNorm << std::endl;


        // determine the objects in the blocking group B
        for (int countB = 0; countB < nObstacle; countB++)
        {
            // Compute projected distance aa and offset bb for each obstacle
            Vector3D ObsDistance;
            ObsDistance = Obstacle.col(countB) - curPosition;
            ObsDistance(2) = 0.0; //set as 2D, with no distance on z axis
            aa(countB) = TotalDistance.dot(ObsDistance) / TotalDistanceNorm;
            db.col(countB) = TotalDistance.cross(ObsDistance) / TotalDistanceNorm;
            db_countB = db.col(countB);
            bb(countB) = db_countB(2);

//            std::cout << " **** Debug 1.2 **** db =  "<< db << std::endl;

            // Determine collision and record projected distance and offset
            if ( (aa(countB) < TotalDistanceNorm) && (aa(countB) > 0) && (abs(bb(countB)) < (RadiusObstacle + RadiusSafety)) )
            {
                nCollision = nCollision + 1;
                Collision.conservativeResize(3, nCollision);
                Collision.col(nCollision - 1) = Obstacle.col(countB);
                collDistance.conservativeResize(2, nCollision);
                collDistance(0, nCollision - 1) = aa(countB);
                collDistance(1, nCollision - 1) = bb(countB);

//             std::cout << "**** Debug 2.2 ****  collDistance = "<< "\n" << collDistance << std::endl;
//             std::cout << "**** Debug 2.2 ****  Collision = "<< "\n" << Collision << std::endl;
            }
        }

        if (nCollision > 0){
        // determine the first obstructing object f and the side to pass
        Eigen::RowVectorXd::Index nf;
        Eigen::Matrix<double, 1, Eigen::Dynamic> ABScollDist_a(nCollision);
        ABScollDist_a = collDistance.row(0).cwiseAbs();
        double mincollDist_a = ABScollDist_a.minCoeff(&nf);
        Eigen::Matrix<double, 3, Eigen::Dynamic> BlockObject(3,1);
        BlockObject.col(0) = Collision.col(nf);
        Eigen::Matrix<double, 2, Eigen::Dynamic>  Distance_BlockObj(2,1);
        Distance_BlockObj << collDistance(0,nf), collDistance(1,nf);
        Eigen::Matrix<double, 2, Eigen::Dynamic> side(2,nObstacle);
        side = Eigen::Matrix<double, 2, Eigen::Dynamic>::Zero(2,nObstacle);
//        side.resize(Eigen::NoChange, nObstacle);
        side(0,0) = (Distance_BlockObj(1) >= 0) ? 1.0 : -1.0;

//        std::cout << "**** Debug 3 ****  nf = "<< nf << std::endl;

        // determine the collision cluster G
        for (int countG = 0; countG < nObstacle; countG++)
        {
            Vector3D BlockObjGap;
            BlockObjGap = Obstacle.col(countG) - BlockObject.col(0);
            BlockObjGap(2) = 0.0; //set as 2D, with no distance on z axis
            if ((BlockObjGap.norm() > 0) && (BlockObjGap.norm() < 2*(RadiusObstacle + RadiusSafety)) )
            {
                nBlockObj = nBlockObj + 1;
                BlockObject.conservativeResize(3, nBlockObj + 1);
                BlockObject.col(nBlockObj) = Obstacle.col(countG);
                Distance_BlockObj.conservativeResize(2, nBlockObj + 1);
                Distance_BlockObj(0, nBlockObj) = aa(countG);
                Distance_BlockObj(1, nBlockObj) = bb(countG);
                side.conservativeResize(2, nBlockObj + 1);
                side(0,nBlockObj) = (Distance_BlockObj(1,nBlockObj) >= 0) ? 1.0 : -1.0;
            }

//            std::cout << "**** Debug 4 ****  side = "<< side << std::endl;
        }
        // determine the side to pass cluster G
        Eigen::RowVectorXd::Index nG;
        Eigen::Matrix<double, 1, Eigen::Dynamic> ABSblockDist_b(nBlockObj + 1);
        ABSblockDist_b = Distance_BlockObj.row(1).cwiseAbs();
        double maxblockDist_b = ABSblockDist_b.maxCoeff(&nG);
        double SideToPass = side(0,nG);

//        std::cout << "**** Debug 4 ****  SideToPass = "<< SideToPass << std::endl;

        // determine the angle to pass each blocking object in group G
        int nBlkObj = BlockObject.cols();      
        Eigen::Matrix<double, 1, Eigen::Dynamic> Alpha_G(nBlkObj);
        for (int countA = 0; countA < nBlkObj; countA++)
        {
            Vector3D BlockObjDistance;
            BlockObjDistance = BlockObject.col(countA) - curPosition;
            BlockObjDistance(2) = 0.0; //set as 2D, with no distance on z axis
            double BlockObjDistanceNorm = std::max(0.000001,BlockObjDistance.norm());
            double sinAlpha_G = std::min(1.0, (2 * RadiusSafety + RadiusObstacle) / BlockObjDistanceNorm );
            Alpha_G(countA) = atan(Distance_BlockObj(1,countA) / Distance_BlockObj(0,countA)) - SideToPass * asin( sinAlpha_G );
        }

//        std::cout << "**** Debug 5 ****  " << std::endl;

        // determine the largest absolute angle to pass
        Eigen::RowVectorXd::Index nAlpha;
        Eigen::Matrix<double, 1, Eigen::Dynamic> ABSAlpha_G(nBlkObj);
        ABSAlpha_G = Alpha_G.cwiseAbs();
        double maxAlpha_G = ABSAlpha_G.maxCoeff(&nAlpha);
        double Alpha_a = Alpha_G(nAlpha);
        Vector3D Object_Alpha = BlockObject.col(nAlpha);
        Vector3D Distance_Alpha = Object_Alpha - curPosition;
        Distance_Alpha(2) = 0.0; //set as 2D, with no distance on z axis
        double Distance_AlphaNorm = std::max(0.000001,Distance_Alpha.norm());

//        std::cout << "**** Debug 6 ****  " << std::endl;

        // determine the subtarget position to pass
        Matrix3D matAlpha;
        matAlpha << cos(Alpha_a), -sin(Alpha_a), 0,
                    sin(Alpha_a),  cos(Alpha_a), 0,
                        0       ,       0      , 1;

        sub_terminal = curPosition + matAlpha * TotalDistance / TotalDistanceNorm * Distance_AlphaNorm;

        // fly over obstacles
        if(ObsHeight <= desPosition(2) - 0.1){
             sub_terminal(0) = 0.7 * (desPosition(0) + curPosition(0));
             sub_terminal(1) = 0.7 * (desPosition(1) + curPosition(1));
             sub_terminal(2) = ObsHeight + RadiusSafety;

   //          std::cout << "**** fly over at ****  " << sub_terminal(2) << " m------" << std::endl;

            if(sub_terminal(2)< desPosition(2)){
                sub_terminal(2) = desPosition(2);
            }
         }
        }

        // record sub-target position
        sub_terminal_last = sub_terminal;
    }
    else {
        sub_terminal = sub_terminal_last;
    }
   }


    // NOMINAL MPC for the 3D translational motion control
        OCPSolver solveMPC;
        const double deltaT = 0.02;
        // initialize gain values
        Matrix3D StateInput = Matrix3D::Zero();

        Matrix3D costWeight = Matrix3D::Zero();
        Eigen::Matrix<double, 3, 6, Eigen::DontAlign> LqrGain = Eigen::Matrix<double, 3, 6, Eigen::DontAlign>::Zero();

        Eigen::Matrix<double, 3, 21> ref_path = Eigen::Matrix<double, 3, 21>::Zero();

       if(ref_path.col(0).norm()!= 0 && ref_path.col(1).norm()!= 0){
        // Weight function of stage state cost, L_stage 6x6 = diag([costWeight(0,0),costWeight(1,0),costWeight(0,0),costWeight(1,0),costWeight(0,0),costWeight(1,0)])
        costWeight(0,0) = options.tCostWeight_pLstagexy->getValue();
        costWeight(2,0) = options.tCostWeight_pLstagez->getValue();
        costWeight(1,0) = options.tCostWeight_vLstage->getValue();
       }else{
        //  Weight function of terminal state cost, L_term 6x6 = diag([costWeight(0,2),costWeight(1,2),costWeight(0,2),costWeight(1,2),costWeight(0,2),costWeight(1,2)])
        costWeight(0,2) = options.tCostWeight_pLtermxy->getValue();
        costWeight(2,2) = options.tCostWeight_pLtermz->getValue();
        costWeight(1,2) = options.tCostWeight_vLterm->getValue();
       }
        // Weight function of stage input cost, R 3x3 =  diag([costWeight(0,1),costWeight(0,1),costWeight(1,1)])
        costWeight(0,1) = options.tCostWeight_Rxy->getValue();
        costWeight(1,1) = options.tCostWeight_Rz->getValue();


        // Linear stabilizing gain for LQR compensational control, K_lqr 3x6
        LqrGain(0,0) = options.tLqrGain_pxy->getValue();
        LqrGain(1,2) = LqrGain(0,0); LqrGain(2,4) = options.tLqrGain_pz->getValue();
        LqrGain(0,1) = options.tLqrGain_vxy->getValue();
        LqrGain(1,3) = LqrGain(0,1); LqrGain(2,5) = options.tLqrGain_vz->getValue();


//      std::cout << " *****Original desPosition: " << desPosition(0) << " , " << desPosition(1) << " , " << desPosition(2) << std::endl;
        StateInput = solveMPC.OCPsolDesigner(deltaT, ExtForce, curPosition, curLinVelocity, ref_path, sub_terminal, costWeight);

        // obtain the nominal state at next predictive step
        NextState_last(0) = StateInput(0,0);
        NextState_last(1) = StateInput(0,1);
        NextState_last(2) = StateInput(0,2);
        NextState_last(3) = StateInput(1,0);
        NextState_last(4) = StateInput(1,1);
        NextState_last(5) = StateInput(1,2);

        // Compute desired thrust vector (3x1) based on the nominal-model based MPC
        Vector3D nominalInput;
        nominalInput(0) = StateInput(2,0);
        nominalInput(1) = StateInput(2,1);
        nominalInput(2) = StateInput(2,2);

        Eigen::Matrix<double, 6, 1> StateError;
        StateError(0) = curPosition(0) - NextState_last(0);
        StateError(1) = curLinVelocity(0) - NextState_last(1);
        StateError(2) = curPosition(1) - NextState_last(2);
        StateError(3) = curLinVelocity(1) - NextState_last(3);
        StateError(4) = curPosition(2) - NextState_last(4);
        StateError(5) = curLinVelocity(2) - NextState_last(5);

//        std::cout << " StateError: " << StateError(0) << " , " << StateError(1) << " , " << StateError(2) << " , " << StateError(3) << " , " << StateError(4) << " , " << StateError(5)  << std::endl;

        Vector3D force_disturbance(0.0,0.0,0.0);
        if (options.tObserver->getValue() && curPosition(2)>=0.35){
        force_disturbance(0) = force_estimate.vector.x;
        force_disturbance(1) = force_estimate.vector.y;
        force_disturbance(2) = force_estimate.vector.z;
        }
       
//         Vector3D desThrust = mass * nominalInput + LqrGain * StateError;
        Vector3D desThrust = mass * (nominalInput + LqrGain * StateError) - force_disturbance;

	
	
        timer_subtarget = timer_subtarget + 1;
        if(timer_subtarget == 3){
           timer_subtarget = 0;
        }

        // Publish desired and current position data
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
            CurVelPublisher.publish(CurVel);
        }

        output.desired_force(0) = desThrust(0);
        output.desired_force(1) = desThrust(1);
        output.desired_force(2) = desThrust(2);

}





// -----------------ATTITUDE CONTROLLER-----------------------


void RoMPC::run_att(const Vector3D desired_force, const TKTrajectory& input, const TKState& currentState, RoMPCOutput& output, const sensor_msgs::Imu& unbiasedImu, const geometry_msgs::Vector3Stamped& torque_estimate)
{

//    double Interval = integTimer.getElapsed().toDSec();
//    integTimer.reset();
//    std::cout << "Attitude Control, time interval:    " << Interval << std::endl;

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

    // Current State
    Position3D curPosition = currentState.position;
    curPosition(1) = -curPosition(1);
    curPosition(2) = -curPosition(2);
    RotAngle3D curOrientation = currentState.getEulerRPY();
    AngVelocity3D curAngVelocity = currentState.angVelocity;

    RotAngle3D desOrientation = input.rotangle;
    // initialize desired yaw angle into original
    if(options.tInitialYawAngle->getValue()){
        if (timer_initialYaw == 0){
        std::cout << "yaw angle initialize!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        iniYawAngle = curOrientation(2);
        timer_initialYaw = timer_initialYaw + 1;
        }
    desOrientation(2) = desOrientation(2) + iniYawAngle;
    }


    curOrientation(1)=-curOrientation(1);
    curOrientation(2)=-curOrientation(2);
    curAngVelocity(1)=-curAngVelocity(1);
    curAngVelocity(2)=-curAngVelocity(2);

    // Initialize gain parameters
    Matrix3D tOmegaGain_OR  = Matrix3D::Zero(), tRotGain_OR  = Matrix3D::Zero();
    double comThrust = iniThrust; double nominalThrust = iniThrust;
    Vector3D comTorque(0.0, 0.0, 0.0);
    Vector3D desThrust = desired_force;

    // Gain matrice for Attitude Control 3x3
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

    // Compute reference rotation matrix
    Matrix3D desMatrix;


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

    comThrust = nominalThrust;
    iniThrust = comThrust;
//std::cout << "Reference Matrix!!!!: <<  "<< RefMatrix << std::endl;
//    }
    // Set thrust constraints, from 1 to 40 N
    comThrust = std::max( comThrust, options.tMinThrust->getValue() );
    comThrust = std::min( comThrust, options.tMaxThrust->getValue() );
//    std::cout << "comThrust!!!!!!!!!! " << comThrust << std::endl;






    /****************************************************/
    // Low-level Global Output Regulator for Attitude

    // Initialize reference angular velocity
    Velocity3D desAngVelocity = Vector3D::Zero();
     if (nominalThrust > 0.0){
        desAngVelocity(2) = (desOrientation(2) - curOrientation(2))/2;
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


        // Compute body frame angular momentum of current state omega2 x Jb*omega2
        Vector3D curAngMomentum = curAngVelocity.cross(InertialMatrix * curAngVelocity);

        Vector3D ErrOmega = curAngVelocity - RotMatrix.transpose() * RefMatrix * desAngVelocity;
        Matrix3D ErrorMatrix = 0.5*(RefMatrix.transpose()*RotMatrix - RotMatrix.transpose()*RefMatrix);
        Vector3D ErrorRotation(ErrorMatrix(2,1),ErrorMatrix(0,2),0);

        // Compute desired torque vector (3x1)
       // comTorque = curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);


        Vector3D torque_disturbance(0.0,0.0,0.0);
        if (options.tObserver->getValue() && curPosition(2)>=0.35){
        torque_disturbance(0) = torque_estimate.vector.x;
        torque_disturbance(1) = torque_estimate.vector.y;
        torque_disturbance(2) = torque_estimate.vector.z;
        }

        comTorque = -torque_disturbance + curAngMomentum - tOmegaGain_OR*ErrOmega - tRotGain_OR*ErrorRotation - InertialMatrix * (Qomega_cur * RotMatrix.transpose() * RefMatrix * desAngVelocity);

    //std::cout << "forces00||: " << comThrust << ", " << comTorque(0) << ", " << comTorque(1) << ", " << comTorque(2) << std::endl;

    // Publish desired and current attitude data
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

//        geometry_msgs::Twist CurAngVel;
//        CurAngVel.angular.x = curAngVelocity(0);
//        CurAngVel.angular.y = curAngVelocity(1);
//        CurAngVel.angular.z = curAngVelocity(2);
//        CurAngVelPublisher.publish(CurAngVel);
    }





    /****************************************************/
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
        motorBuf_Hex = inputTransMatrix_Hex.transpose()*(inputTransMatrix_Hex*inputTransMatrix_Hex.transpose()).inverse() * CoeffMatrix * Force;
        for (int count = 0; count < nrMotors_Hex; count++)
        {
            motorBuf_Hex(count) = std::max(motorBuf_Hex(count),0.0);
            motorBuf_Hex(count) = sqrt(motorBuf_Hex(count));
        }
//        std::cout << "Motor Speeds: " << motorBuf_Hex(0) << " " << motorBuf_Hex(1) << " " << motorBuf_Hex(2) << " " << motorBuf_Hex(3) << " " << motorBuf_Hex(4)<< " " << motorBuf_Hex(5) << std::endl;
    }

//std::cout << "forces: " << comThrust << ", " << comTorque(0) << ", " << comTorque(1) << ", " << comTorque(2) << std::endl;


    // Calculate motor speed from transmission matrix MotorSpeed^2 = G * Force
    // G is normalized with Thrust Coefficient Ct, thus very likely to add extra gain P = C

    motorBuf = PropellerGain * inputTransMatrix * Force;
//std::cout << "MotorForce: " << motorBuf(0) << ", " << motorBuf(1) << ", " << motorBuf(2) << ", " << motorBuf(3) << std::endl;


//    double sleepTime = FIXED_TIME_STEP - computeTime;
//    ros::Duration(sleepTime).sleep();

//    std::cout << " Control || Computation Time:  "<< computeTime << std::endl;

//    while (true)
//    {
//        if (timeStep >= FIXED_TIME_STEP)
//        {break;}
//    }
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
    }

    output.Thrust = comThrust;

}


}

