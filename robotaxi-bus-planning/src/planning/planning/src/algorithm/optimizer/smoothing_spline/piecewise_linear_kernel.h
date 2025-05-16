/**
 * @file piecewise_linear_kernel.h
 * @brief: Definition of PiecewiseLinearKernel class
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

namespace acu {
namespace planning {
namespace math {
class PiecewiseLinearKernel {
 public:
  PiecewiseLinearKernel(const uint32_t dimension, const double unit_segment);

  void AddRegularization(const double param);

  const Eigen::MatrixXd& kernel_matrix() const;
  const Eigen::MatrixXd& offset_matrix() const;

  void AddSecondOrderDerivativeMatrix(const double init_derivative,
                                      const double weight);
  void AddThirdOrderDerivativeMatrix(const double init_derivative,
                                     const double init_second_derivative,
                                     const double weight);

  // reference line kernel
  bool AddReferenceLineKernelMatrix(const std::vector<uint32_t>& index_list,
                                    const std::vector<double>& pos_list,
                                    const double weight);

 private:
  const uint32_t dimension_;
  const double unit_segment_;
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_matrix_;
};

}
}  // namespace planning
}  // namespace acu
