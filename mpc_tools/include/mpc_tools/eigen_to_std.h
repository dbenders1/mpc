#ifndef MPC_TOOLS_EIGEN_TO_STD_H
#define MPC_TOOLS_EIGEN_TO_STD_H

#include <vector>

#include <Eigen/Core>

namespace Helpers {
namespace EigenToStd {

void eigenToStdVector(const Eigen::MatrixXd &matrix, std::vector<std::vector<double>> &result);

}; // EigenToStd
}; // namespace Helpers

#endif // MPC_TOOLS_EIGEN_TO_STD_H
