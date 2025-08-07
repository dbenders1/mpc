#ifndef MPC_TOOLS_TRAJ_GEN_H
#define MPC_TOOLS_TRAJ_GEN_H

#include <Eigen/Dense>

Eigen::MatrixXd circleTrajectory(double radius, double frequency, double dt,
                                 bool clockwise = true);
Eigen::MatrixXd lemniscateTrajectory(double radius, double frequency, double dt,
                                     bool clockwise = true);

#endif  // MPC_TOOLS_TRAJ_GEN_H
