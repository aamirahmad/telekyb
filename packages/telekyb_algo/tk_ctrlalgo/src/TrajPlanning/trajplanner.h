#ifndef TRAJPLANNER_H
#define TRAJPLANNER_H

#include <Eigen/Dense>

extern "C"{
#include "solver.h"
}



class TrajPlanner
{

public:
    TrajPlanner();

    // Design a trajectory containing desired position, velocity, acceleration, jerk via the solution of the convex optimization problem
    Eigen::Matrix<double,3,4> TrajPlanDesigner(double deltaT, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Vector3d CurAcceleration,  Eigen::Vector3d Waypoint, Eigen::Matrix3d CostFunction);
    // Load and generate required parameters for the optimal control problem
    void load_data(double delta, Eigen::Vector3d InitialState, Eigen::Vector3d TerminalState, Eigen::Vector4d Constraints, Eigen::Matrix3d CostFunction);
    // output the solution of decoupled optimal control problems
    Eigen::Vector4d use_solution(Vars vars);

};

#endif // TRAJPLANNER_H
