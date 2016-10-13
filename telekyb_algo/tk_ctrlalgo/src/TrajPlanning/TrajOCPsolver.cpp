#include "TrajOCPsolver.h"
#include <iostream>

Vars vars;
Params params;
Workspace work;
Settings settings;

TrajOCPSolver::TrajOCPSolver()
{
}

Eigen::Matrix3d TrajOCPSolver::OCPsolDesigner(double deltaT, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Vector3d CurAcceleration, Eigen::Matrix<double, 3, 21> ReferencePath, Eigen::Vector3d Waypoint, Eigen::Matrix<double, 3, 5> CostFunction)
{

    // initialize 1st predictive state and control input
    Eigen::Matrix3d StateInput = Eigen::Matrix3d::Zero();
//    int num_iters_x, num_iters_y, num_iters_z;

    // initialize defaults parameters and variables
    set_defaults();
    setup_indexing();
    // disable output of solver progress
    settings.verbose = 0;


    // set up initial state and constraints for optimal control problem
    Eigen::Matrix<double, 9, 1> curState;
    curState << CurPosition(0), CurVelocity(0), CurAcceleration(0), CurPosition(1), CurVelocity(1), CurAcceleration(1), CurPosition(2), CurVelocity(2), CurAcceleration(2);
    Eigen::Matrix<double, 9, 1> termState;
    termState << Waypoint(0), 0, 0, Waypoint(1), 0, 0, Waypoint(2), 0, 0;

    Eigen::Matrix<double, 9, 2> state_constraint;
    state_constraint << -6, 6,
                        -2.5, 2.5,
			-5, 5,
                        -6, 6,
                        -2.5, 2.5,
			-5, 5,
                         0, 3,
                        -3, 3,
			-6, 6;

    Eigen::Matrix<double, 3, 2> input_constraint;
    input_constraint << -300, 300,
                        -300, 300,
                        -300, 300;


    // generate an optimal control problem for x-axis motion
    load_data(deltaT, curState, termState, ReferencePath, state_constraint, input_constraint, CostFunction);
    // solve 3d OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along x-axis) as reference
    StateInput = use_solution(vars);

    return StateInput;
}

void TrajOCPSolver::load_data(double deltaT, Eigen::Matrix<double, 9, 1> InitialState, Eigen::Matrix<double, 9, 1> TerminalState, Eigen::Matrix<double, 3, 21> ReferencePath, Eigen::Matrix<double, 9, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix<double, 3, 5> CostFunction)
{
    params.x_0[0] = InitialState(0);
    params.x_0[1] = InitialState(1);
    params.x_0[2] = InitialState(2);
    params.x_0[3] = InitialState(3);
    params.x_0[4] = InitialState(4);
    params.x_0[5] = InitialState(5);
    params.x_0[6] = InitialState(6);
    params.x_0[7] = InitialState(7);
    params.x_0[8] = InitialState(8);

    params.xN[0] = TerminalState(0);
    params.xN[1] = TerminalState(1);
    params.xN[2] = TerminalState(2);
    params.xN[3] = TerminalState(3);
    params.xN[4] = TerminalState(4);
    params.xN[5] = TerminalState(5);
    params.xN[6] = TerminalState(6);
    params.xN[7] = TerminalState(7);
    params.xN[8] = TerminalState(8);

    params.x_min[0] = StateConstraint(0,0);
    params.x_min[1] = StateConstraint(1,0);
    params.x_min[2] = StateConstraint(2,0);
    params.x_min[3] = StateConstraint(3,0);
    params.x_min[4] = StateConstraint(4,0);
    params.x_min[5] = StateConstraint(5,0);
    params.x_min[6] = StateConstraint(6,0);
    params.x_min[7] = StateConstraint(7,0);
    params.x_min[8] = StateConstraint(8,0);


    params.x_max[0] = StateConstraint(0,1);
    params.x_max[1] = StateConstraint(1,1);
    params.x_max[2] = StateConstraint(2,1);
    params.x_max[3] = StateConstraint(3,1);
    params.x_max[4] = StateConstraint(4,1);
    params.x_max[5] = StateConstraint(5,1);
    params.x_max[6] = StateConstraint(6,1);
    params.x_max[7] = StateConstraint(7,1);
    params.x_max[8] = StateConstraint(8,1);

    params.u_min[0] = InputConstraint(0,0);
    params.u_min[1] = InputConstraint(1,0);
    params.u_min[2] = InputConstraint(2,0);

    params.u_max[0] = InputConstraint(0,1);
    params.u_max[1] = InputConstraint(1,1);
    params.u_max[2] = InputConstraint(2,1);

    params.A[0] = 1;
    params.A[1] = deltaT;
    params.A[2] = 0.5*deltaT*deltaT;
    params.A[3] = 1;
    params.A[4] = deltaT;
    params.A[5] = 1;
    params.A[6] = 1;
    params.A[7] = deltaT;
    params.A[8] = 0.5*deltaT*deltaT;
    params.A[9] = 1;
    params.A[10] = deltaT;
    params.A[11] = 1;
    params.A[12] = 1;
    params.A[13] = deltaT;
    params.A[14] = 0.5*deltaT*deltaT;
    params.A[15] = 1;
    params.A[16] = deltaT;
    params.A[17] = 1;

    params.B[0] = 0.16667*deltaT*deltaT*deltaT;
    params.B[1] = 0.5*deltaT*deltaT;
    params.B[2] = deltaT;
    params.B[3] = 0.16667*deltaT*deltaT*deltaT;
    params.B[4] = 0.5*deltaT*deltaT;
    params.B[5] = deltaT;
    params.B[6] = 0.16667*deltaT*deltaT*deltaT;
    params.B[7] = 0.5*deltaT*deltaT;
    params.B[8] = deltaT;

    params.L_stage[0] = CostFunction(0,0);
    params.L_stage[1] = CostFunction(1,0);
    params.L_stage[2] = CostFunction(2,0);
    params.L_stage[3] = CostFunction(0,0);
    params.L_stage[4] = CostFunction(1,0);
    params.L_stage[5] = CostFunction(2,0);
    params.L_stage[6] = CostFunction(0,1);
    params.L_stage[7] = CostFunction(1,1);
    params.L_stage[8] = CostFunction(2,1);
    
    params.R[0] = CostFunction(0,2);
    params.R[1] = CostFunction(1,2);
    params.R[2] = CostFunction(2,2);
    
    params.L_term[0] = CostFunction(0,3);
    params.L_term[1] = CostFunction(1,3);
    params.L_term[2] = CostFunction(2,3);
    params.L_term[3] = CostFunction(0,3);
    params.L_term[4] = CostFunction(1,3);
    params.L_term[5] = CostFunction(2,3);
    params.L_term[6] = CostFunction(0,4);
    params.L_term[7] = CostFunction(1,4);
    params.L_term[8] = CostFunction(2,4);

    params.xr_0[0] = ReferencePath(0,0);
    params.xr_0[1] = 0;
    params.xr_0[2] = 0;
    params.xr_0[3] = ReferencePath(1,0);
    params.xr_0[4] = 0;
    params.xr_0[5] = 0;
    params.xr_0[6] = ReferencePath(2,0);
    params.xr_0[7] = 0;
    params.xr_0[8] = 0;

    params.xr_1[0] = ReferencePath(0,1);
    params.xr_1[1] = 0;
    params.xr_1[2] = 0;
    params.xr_1[3] = ReferencePath(1,1);
    params.xr_1[4] = 0;
    params.xr_1[5] = 0;
    params.xr_1[6] = ReferencePath(2,1);
    params.xr_1[7] = 0;
    params.xr_1[8] = 0;

    params.xr_2[0] = ReferencePath(0,2);
    params.xr_2[1] = 0;
    params.xr_2[2] = 0;
    params.xr_2[3] = ReferencePath(1,2);
    params.xr_2[4] = 0;
    params.xr_2[5] = 0;
    params.xr_2[6] = ReferencePath(2,2);
    params.xr_2[7] = 0;
    params.xr_2[8] = 0;

    params.xr_3[0] = ReferencePath(0,3);
    params.xr_3[1] = 0;
    params.xr_3[2] = 0;
    params.xr_3[3] = ReferencePath(1,3);
    params.xr_3[4] = 0;
    params.xr_3[5] = 0;
    params.xr_3[6] = ReferencePath(2,3);
    params.xr_3[7] = 0;
    params.xr_3[8] = 0;

    params.xr_4[0] = ReferencePath(0,4);
    params.xr_4[1] = 0;
    params.xr_4[2] = 0;
    params.xr_4[3] = ReferencePath(1,4);
    params.xr_4[4] = 0;
    params.xr_4[5] = 0;
    params.xr_4[6] = ReferencePath(2,4);
    params.xr_4[7] = 0;
    params.xr_4[8] = 0;

    params.xr_5[0] = ReferencePath(0,5);
    params.xr_5[1] = 0;
    params.xr_5[2] = 0;
    params.xr_5[3] = ReferencePath(1,5);
    params.xr_5[4] = 0;
    params.xr_5[5] = 0;
    params.xr_5[6] = ReferencePath(2,5);
    params.xr_5[7] = 0;
    params.xr_5[8] = 0;

    params.xr_6[0] = ReferencePath(0,6);
    params.xr_6[1] = 0;
    params.xr_6[2] = 0;
    params.xr_6[3] = ReferencePath(1,6);
    params.xr_6[4] = 0;
    params.xr_6[5] = 0;
    params.xr_6[6] = ReferencePath(2,6);
    params.xr_6[7] = 0;
    params.xr_6[8] = 0;

    params.xr_7[0] = ReferencePath(0,7);
    params.xr_7[1] = 0;
    params.xr_7[2] = 0;
    params.xr_7[3] = ReferencePath(1,7);
    params.xr_7[4] = 0;
    params.xr_7[5] = 0;
    params.xr_7[6] = ReferencePath(2,7);
    params.xr_7[7] = 0;
    params.xr_7[8] = 0;

    params.xr_8[0] = ReferencePath(0,8);
    params.xr_8[1] = 0;
    params.xr_8[2] = 0;
    params.xr_8[3] = ReferencePath(1,8);
    params.xr_8[4] = 0;
    params.xr_8[5] = 0;
    params.xr_8[6] = ReferencePath(2,8);
    params.xr_8[7] = 0;
    params.xr_8[8] = 0;

    params.xr_9[0] = ReferencePath(0,9);
    params.xr_9[1] = 0;
    params.xr_9[2] = 0;
    params.xr_9[3] = ReferencePath(1,9);
    params.xr_9[4] = 0;
    params.xr_9[5] = 0;
    params.xr_9[6] = ReferencePath(2,9);
    params.xr_9[7] = 0;
    params.xr_9[8] = 0;

    params.xr_10[0] = ReferencePath(0,10);
    params.xr_10[1] = 0;
    params.xr_10[2] = 0;
    params.xr_10[3] = ReferencePath(1,10);
    params.xr_10[4] = 0;
    params.xr_10[5] = 0;
    params.xr_10[6] = ReferencePath(2,10);
    params.xr_10[7] = 0;
    params.xr_10[8] = 0;

    params.xr_11[0] = ReferencePath(0,11);
    params.xr_11[1] = 0;
    params.xr_11[2] = 0;
    params.xr_11[3] = ReferencePath(1,11);
    params.xr_11[4] = 0;
    params.xr_11[5] = 0;
    params.xr_11[6] = ReferencePath(2,11);
    params.xr_11[7] = 0;
    params.xr_11[8] = 0;

    params.xr_12[0] = ReferencePath(0,12);
    params.xr_12[1] = 0;
    params.xr_12[2] = 0;
    params.xr_12[3] = ReferencePath(1,12);
    params.xr_12[4] = 0;
    params.xr_12[5] = 0;
    params.xr_12[6] = ReferencePath(2,12);
    params.xr_12[7] = 0;
    params.xr_12[8] = 0;

    params.xr_13[0] = ReferencePath(0,13);
    params.xr_13[1] = 0;
    params.xr_13[2] = 0;
    params.xr_13[3] = ReferencePath(1,13);
    params.xr_13[4] = 0;
    params.xr_13[5] = 0;
    params.xr_13[6] = ReferencePath(2,13);
    params.xr_13[7] = 0;
    params.xr_13[8] = 0;

    params.xr_14[0] = ReferencePath(0,14);
    params.xr_14[1] = 0;
    params.xr_14[2] = 0;
    params.xr_14[3] = ReferencePath(1,14);
    params.xr_14[4] = 0;
    params.xr_14[5] = 0;
    params.xr_14[6] = ReferencePath(2,14);
    params.xr_14[7] = 0;
    params.xr_14[8] = 0;

    params.xr_15[0] = ReferencePath(0,15);
    params.xr_15[1] = 0;
    params.xr_15[2] = 0;
    params.xr_15[3] = ReferencePath(1,15);
    params.xr_15[4] = 0;
    params.xr_15[5] = 0;
    params.xr_15[6] = ReferencePath(2,15);
    params.xr_15[7] = 0;
    params.xr_15[8] = 0;

    params.xr_16[0] = ReferencePath(0,16);
    params.xr_16[1] = 0;
    params.xr_16[2] = 0;
    params.xr_16[3] = ReferencePath(1,16);
    params.xr_16[4] = 0;
    params.xr_16[5] = 0;
    params.xr_16[6] = ReferencePath(2,16);
    params.xr_16[7] = 0;
    params.xr_16[8] = 0;

    params.xr_17[0] = ReferencePath(0,17);
    params.xr_17[1] = 0;
    params.xr_17[2] = 0;
    params.xr_17[3] = ReferencePath(1,17);
    params.xr_17[4] = 0;
    params.xr_17[5] = 0;
    params.xr_17[6] = ReferencePath(2,17);
    params.xr_17[7] = 0;
    params.xr_17[8] = 0;

    params.xr_18[0] = ReferencePath(0,18);
    params.xr_18[1] = 0;
    params.xr_18[2] = 0;
    params.xr_18[3] = ReferencePath(1,18);
    params.xr_18[4] = 0;
    params.xr_18[5] = 0;
    params.xr_18[6] = ReferencePath(2,18);
    params.xr_18[7] = 0;
    params.xr_18[8] = 0;

    params.xr_19[0] = ReferencePath(0,19);
    params.xr_19[1] = 0;
    params.xr_19[2] = 0;
    params.xr_19[3] = ReferencePath(1,19);
    params.xr_19[4] = 0;
    params.xr_19[5] = 0;
    params.xr_19[6] = ReferencePath(2,19);
    params.xr_19[7] = 0;
    params.xr_19[8] = 0;

    params.xr_20[0] = ReferencePath(0,20);
    params.xr_20[1] = 0;
    params.xr_20[2] = 0;
    params.xr_20[3] = ReferencePath(1,20);
    params.xr_20[4] = 0;
    params.xr_20[5] = 0;
    params.xr_20[6] = ReferencePath(2,20);
    params.xr_20[7] = 0;
    params.xr_20[8] = 0;
    
}


Eigen::Matrix3d TrajOCPSolver::use_solution(Vars vars)
{
    Eigen::Matrix3d StateInput_output;
   StateInput_output << vars.x_1[0], vars.x_1[1], vars.x_1[2],
                        vars.x_1[3], vars.x_1[4], vars.x_1[5],
                        vars.x_1[6], vars.x_1[7], vars.x_1[8];
    return StateInput_output;
}

