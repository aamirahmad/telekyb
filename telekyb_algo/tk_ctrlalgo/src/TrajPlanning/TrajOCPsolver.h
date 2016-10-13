#ifndef TRAJOCPSOLVER_H
#define TRAJOCPSOLVER_H

#include <Eigen/Dense>

extern "C"{
#include "solver.h"
}



class TrajOCPSolver
{

public:
    TrajOCPSolver();

    // Design a trajectory containing desired position, velocity, acceleration, jerk via the solution of the convex optimization problem
    Eigen::Matrix3d OCPsolDesigner(double deltaT, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Vector3d CurAcceleration, Eigen::Matrix<double, 3, 21> ReferencePath, Eigen::Vector3d Waypoint, Eigen::Matrix<double, 3, 5> CostFunction);
    // Load and generate required parameters for the optimal control problem
    void load_data(double deltaT, Eigen::Matrix<double, 9, 1> InitialState, Eigen::Matrix<double, 9, 1> TerminalState, Eigen::Matrix<double, 3, 21> ReferencePath, Eigen::Matrix<double, 9, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix<double, 3, 5> CostFunction);
    // output the solution of decoupled optimal control problems
    Eigen::Matrix3d use_solution(Vars vars);

};

#endif // TRAJOCPSOLVER_H
