#ifndef MPC_TOOLS_MATH_H
#define MPC_TOOLS_MATH_H

#include <Eigen/Dense>

inline double euclideanDistance(const Eigen::Vector3d& a,
                                const Eigen::Vector3d& b) {
  return (a - b).norm();
}

template <typename Derived>
inline Eigen::Quaternion<typename Derived::Scalar> eulerToQuaternion(
    const Eigen::Block<Derived, 3, 1>& eulerAngles) {
  using Scalar = typename Derived::Scalar;
  Scalar psi = eulerAngles(2);    // Rotation around z-axis
  Scalar theta = eulerAngles(1);  // Rotation around rotated y-axis
  Scalar phi = eulerAngles(0);    // Rotation around twice rotated x-axis

  Eigen::AngleAxis<Scalar> rollAngle(phi, Eigen::Matrix<Scalar, 3, 1>::UnitX());
  Eigen::AngleAxis<Scalar> pitchAngle(theta,
                                      Eigen::Matrix<Scalar, 3, 1>::UnitY());
  Eigen::AngleAxis<Scalar> yawAngle(psi, Eigen::Matrix<Scalar, 3, 1>::UnitZ());

  Eigen::Quaternion<Scalar> q = yawAngle * pitchAngle * rollAngle;
  return q;
}

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q) {
  // Convert quaternion to rotation matrix
  Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

  // Extract ZYX Euler angles
  double psi = atan2(rotationMatrix(1, 0),
                     rotationMatrix(0, 0));    // Rotation around z-axis
  double theta = asin(-rotationMatrix(2, 0));  // Rotation around rotated y-axis
  double phi =
      atan2(rotationMatrix(2, 1),
            rotationMatrix(2, 2));  // Rotation around twice rotated x-axis

  return Eigen::Vector3d(phi, theta, psi);
}

Eigen::VectorXd interp_cubic_hermite(const Eigen::VectorXd x_prev,
                                     const Eigen::VectorXd x_next,
                                     const Eigen::VectorXd dx_prev,
                                     const Eigen::VectorXd dx_next,
                                     const double t_prev, const double t_next,
                                     const double t) {
  double dt = t_next - t_prev;
  double t_norm = (t - t_prev) / dt;
  double h00 = 2 * t_norm * t_norm * t_norm - 3 * t_norm * t_norm + 1;
  double h10 = t_norm * t_norm * t_norm - 2 * t_norm * t_norm + t_norm;
  double h01 = -2 * t_norm * t_norm * t_norm + 3 * t_norm * t_norm;
  double h11 = t_norm * t_norm * t_norm - t_norm * t_norm;
  int nx = x_prev.size();
  Eigen::VectorXd x_interp(nx);
  for (int i = 0; i < nx; ++i) {
    x_interp(i) = h00 * x_prev(i) + h10 * dt * dx_prev(i) + h01 * x_next(i) +
                  h11 * dt * dx_next(i);
  }
  return x_interp;
}

#endif  // MPC_TOOLS_MATH_H
