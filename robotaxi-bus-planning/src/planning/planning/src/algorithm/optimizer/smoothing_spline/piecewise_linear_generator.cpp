/**
 * @file piecewise_linear_generator.cpp
 * @brief: piecewise_smoothing_spline (pss) generator class solve pss by qp
 *  algorithm, include adding constraint, adding kernel, and solver solve
 **/

#include "piecewise_linear_generator.h"
#include "common/base/log/include/log.h"
#include "common/math/qp_solver/active_set_qp_solver.h"

namespace acu {
namespace planning {
namespace math {

// NOTICE: the first point is kept at 0.0
PiecewiseLinearGenerator::PiecewiseLinearGenerator(
    const uint32_t num_of_segments, const double unit_segment)
    : num_of_segments_(num_of_segments),
      unit_segment_(unit_segment),
      total_t_(num_of_segments * unit_segment),
      constraint_(num_of_segments, unit_segment),
      kernel_(num_of_segments, unit_segment) {
  CHECK_GE(num_of_segments, 3);
}

PiecewiseLinearConstraint* PiecewiseLinearGenerator::mutable_constraint() {
  return &constraint_;
}

PiecewiseLinearKernel* PiecewiseLinearGenerator::mutable_kernel() {
  return &kernel_;
}

bool PiecewiseLinearGenerator::Solve() {
  const Eigen::MatrixXd& kernel_matrix = kernel_.kernel_matrix();
  const Eigen::MatrixXd& offset = kernel_.offset_matrix();

  const Eigen::MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint_matrix();
  const Eigen::MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint_boundary();

  const Eigen::MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint_matrix();
  const Eigen::MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint_boundary();

  qp_solver_.reset(new acu::common::math::ActiveSetQpSolver(
      kernel_matrix, offset, inequality_constraint_matrix,
      inequality_constraint_boundary, equality_constraint_matrix,
      equality_constraint_boundary));

  qp_solver_->EnableCholeskyRefactorisation(1);
  qp_solver_->set_pos_definite_hessian();

  if (!qp_solver_->Solve()) {
    return false;
  }
  const Eigen::MatrixXd solved_params = qp_solver_->params();
  return true;
}

}
}  // namespace planning
}  // namespace acu
