#include <mpc_tools/eigen_to_std.h>

namespace Helpers {
namespace EigenToStd {

void eigenToStdVector(const Eigen::MatrixXd &matrix, std::vector<std::vector<double>> &result)
{
    int n_rows = matrix.rows(), n_cols = matrix.cols();
    result.resize(n_rows);
    for (int i = 0; i < n_rows; ++i)
    {
        result[i].resize(n_cols);
        for (int j = 0; j < n_cols; ++j)
        {
            result[i][j] = matrix(i, j);
        }
    }
}

}; // EigenToStd
}; // namespace Helpers
