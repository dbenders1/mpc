#ifndef MPC_TOOLS_EIGEN_STD_H
#define MPC_TOOLS_EIGEN_STD_H

#include <Eigen/Core>
#include <vector>

void eigenMatrixToStdVector(const Eigen::MatrixXd &matrix,
                            std::vector<std::vector<double>> &result);

template <typename Derived>
inline std::vector<typename Derived::Scalar> eigenToStdVector(
    const Eigen::MatrixBase<Derived> &block) {
  using Scalar = typename Derived::Scalar;
  std::vector<Scalar> stdVector(block.size());
  for (int i = 0; i < block.size(); ++i) {
    stdVector[i] = block(i);
  }

  return stdVector;
}

template <typename Derived>
inline Eigen::Matrix<Derived, Eigen::Dynamic, 1> stdToEigenVector(
    const std::vector<Derived> &std_vector) {
  Eigen::Matrix<Derived, Eigen::Dynamic, 1> eigen_vector(std_vector.size());
  for (size_t i = 0; i < std_vector.size(); ++i) {
    eigen_vector(i) = std_vector[i];
  }
  return eigen_vector;
}

#endif  // MPC_TOOLS_EIGEN_STD_H
