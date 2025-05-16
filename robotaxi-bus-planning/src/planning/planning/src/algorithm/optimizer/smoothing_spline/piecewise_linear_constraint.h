/**
 * @file : piecewise_linear_constraint.h
 * @brief: Definition of PiecewiseLinearConstraint class.
 **/

#pragma once

#include <vector>

#include "Eigen/Core"

namespace acu {
namespace planning {
namespace math {

class PiecewiseLinearConstraint {
 public:
  PiecewiseLinearConstraint(const uint32_t dimension,
                            const double unit_segment);
  virtual ~PiecewiseLinearConstraint() = default;

  Eigen::MatrixXd inequality_constraint_matrix() const;
  Eigen::MatrixXd inequality_constraint_boundary() const;
  Eigen::MatrixXd equality_constraint_matrix() const;
  Eigen::MatrixXd equality_constraint_boundary() const;

  /**
   * @brief: inequality boundary constraints
   **/
  bool AddBoundary(const std::vector<uint32_t>& index_list,
                   const std::vector<double>& lower_bound,
                   const std::vector<double>& upper_bound);
  bool AddDerivativeBoundary(const std::vector<uint32_t>& index_list,
                             const std::vector<double>& lower_bound,
                             const std::vector<double>& upper_bound);
  bool AddSecondDerivativeBoundary(const double init_derivative,
                                   const std::vector<uint32_t>& index_list,
                                   const std::vector<double>& lower_bound,
                                   const std::vector<double>& upper_bound);

  /**
   * @brief: equality constraints
   **/
  bool AddPointConstraint(const uint32_t index, const double val);
  bool AddPointDerivativeConstraint(const uint32_t index, const double val);

  /**
   * @brief: Add monotone constraint inequality at all indices
   **/
  bool AddMonotoneInequalityConstraint();

 private:
  const uint32_t dimension_;
  const double unit_segment_;
  std::vector<Eigen::MatrixXd> inequality_matrices_;
  std::vector<Eigen::MatrixXd> inequality_boundaries_;

  std::vector<Eigen::MatrixXd> equality_matrices_;
  std::vector<Eigen::MatrixXd> equality_boundaries_;
};

}
}  // namespace planning
}  // namespace acu
