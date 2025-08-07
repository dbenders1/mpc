#include "mpc_tools/traj_gen.h"

#include <cmath>

Eigen::MatrixXd circleTrajectory(double radius, double frequency, double dt,
                                 bool clockwise) {
  int n_points = static_cast<int>(1 / (frequency * dt)) + 1;
  Eigen::MatrixXd trajectory(3, n_points);
  int y_sign = clockwise ? -1 : 1;
  for (int i = 0; i < n_points; ++i) {
    double angle = 2 * M_PI * frequency * i * dt;
    trajectory(0, i) = -radius + radius * cos(angle);
    trajectory(1, i) = y_sign * radius * sin(angle);
    trajectory(2, i) = 1;
  }
  return trajectory;
}

Eigen::MatrixXd lemniscateTrajectory(double radius, double frequency, double dt,
                                     bool clockwise) {
  int n_points = static_cast<int>(2 / (frequency * dt)) + 1;
  Eigen::MatrixXd trajectory(3, n_points);
  int sign = clockwise ? 1 : -1;
  for (int i = 0; i < n_points; ++i) {
    double y_angle = M_PI * frequency * i * dt;
    double x_angle = 2 * y_angle;
    trajectory(0, i) = sign * radius * sin(x_angle);
    trajectory(1, i) = -sign * radius * sin(y_angle);
    trajectory(2, i) = 1;
  }
  return trajectory;
}
