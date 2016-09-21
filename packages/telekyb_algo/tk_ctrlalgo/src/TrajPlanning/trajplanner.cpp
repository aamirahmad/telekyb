#include "trajplanner.h"
#include <iostream>

Vars vars;
Params params;
Workspace work;
Settings settings;

TrajPlanner::TrajPlanner()
{
}

Eigen::Matrix<double,3,4> TrajPlanner::TrajPlanDesigner(double deltaT, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Vector3d CurAcceleration, Eigen::Vector3d Waypoint, Eigen::Matrix3d CostFunction)
{

    // initialize desired state reference
    Eigen::Matrix<double,4,3> desState = Eigen::Matrix<double,4,3>::Zero();
//    int num_iters_x, num_iters_y, num_iters_z;

    // initialize defaults parameters and variables
    set_defaults();
    setup_indexing();
    // disable output of solver progress
    settings.verbose = 0;


    // set up initial state and constraints for optimal control problem
    Eigen::Vector3d curState_x(CurPosition(0), CurVelocity(0), CurAcceleration(0));
    Eigen::Vector3d termState_x(Waypoint(0), 0, 0);
    double ddx_min = -3, ddx_max = 3, jx_min = -30, jx_max = 30;
    Eigen::Vector4d constraint_x(ddx_min,ddx_max,jx_min,jx_max);
    // generate an optimal control problem for x-axis motion
    load_data(deltaT, curState_x, termState_x, constraint_x, CostFunction);
    // solve x-axis OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along x-axis) as reference
    desState.col(0) = use_solution(vars);

    // set up initial state and constraints for optimal control problem
    Eigen::Vector3d curState_y(CurPosition(1), CurVelocity(1), CurAcceleration(1));
    Eigen::Vector3d termState_y(Waypoint(1), 0, 0);
    double ddy_min = -3, ddy_max = 3, jy_min = -30, jy_max = 30;
    Eigen::Vector4d constraint_y(ddy_min,ddy_max,jy_min,jy_max);
    // generate an optimal control problem for y-axis motion
    load_data(deltaT, curState_y, termState_y, constraint_y, CostFunction);
    // solve y-axis OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along y-axis) as reference
    desState.col(1) = use_solution(vars);

    // set up initial state and constraints for optimal control problem
    Eigen::Vector3d curState_z(CurPosition(2), CurVelocity(2), CurAcceleration(2));
    Eigen::Vector3d termState_z(Waypoint(2), 0, 0);
    double ddz_min = -7, ddz_max = 12, jz_min = -100, jz_max = 100;
    Eigen::Vector4d constraint_z(ddz_min,ddz_max,jz_min,jz_max);
    // generate an optimal control problem for z-axis motion
    load_data(deltaT, curState_z, termState_z, constraint_z, CostFunction);
    // solve z-axis OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along z-axis) as reference
    desState.col(2) = use_solution(vars);

    return desState.transpose();
}

void TrajPlanner::load_data(double deltaT, Eigen::Vector3d InitialState, Eigen::Vector3d TerminalState, Eigen::Vector4d Constraints, Eigen::Matrix3d CostFunction)
{
    params.x_0[0] = InitialState(0);
    params.x_0[1] = InitialState(1);
    params.x_0[2] = InitialState(2);
    params.xN[0] = TerminalState(0);
    params.xN[1] = TerminalState(1);
    params.xN[2] = TerminalState(2);
    params.s_acc[0] = -Constraints(0);
    params.s_acc[1] = Constraints(1);
    params.u_bound[0] = -Constraints(2);
    params.u_bound[1] = Constraints(3);
    params.A[0] = 1;
    params.A[1] = 0;
    params.A[2] = 0;
    params.A[3] = deltaT;
    params.A[4] = 1;
    params.A[5] = 0;
    params.A[6] = 0.5*deltaT*deltaT;
    params.A[7] = deltaT;
    params.A[8] = 1;
    params.B[0] = deltaT*deltaT*deltaT/6;
    params.B[1] = 0.5*deltaT*deltaT;
    params.B[2] = deltaT;
    params.C[0] = 0;
    params.C[1] = 0;
    params.C[2] = 0;
    params.C[3] = 0;
    params.C[4] = -1;
    params.C[5] = 1;
    params.D[0] = -1;
    params.D[1] = 1;
//    params.L_stage[0] = CostFunction(0,0);
//    params.L_stage[3] = 0;
//    params.L_stage[6] = 0;
//    params.L_stage[1] = 0;
//    params.L_stage[4] = CostFunction(1,0);
//    params.L_stage[7] = 0;
//    params.L_stage[2] = 0;
//    params.L_stage[5] = 0;
//    params.L_stage[8] = CostFunction(2,0);
    params.R[0] = CostFunction(0,1);
    params.L_term[0] = CostFunction(0,2);
    params.L_term[3] = 0;
    params.L_term[6] = 0;
    params.L_term[1] = 0;
    params.L_term[4] = CostFunction(1,2);
    params.L_term[7] = 0;
    params.L_term[2] = 0;
    params.L_term[5] = 0;
    params.L_term[8] = CostFunction(2,2);
}


Eigen::Vector4d TrajPlanner::use_solution(Vars vars)
{
    Eigen::Vector4d desState_axis(vars.x_1[0], vars.x_1[1], vars.x_1[2], vars.u_0[0]);
    return desState_axis;
}
