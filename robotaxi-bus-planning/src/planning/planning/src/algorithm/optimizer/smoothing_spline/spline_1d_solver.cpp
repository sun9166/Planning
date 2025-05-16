/**
 * @file spline_1d_solver.cpp
 **/
#include "spline_1d_solver.h"

namespace acu {
namespace planning {
namespace math {

using Eigen::MatrixXd;

// converte qp problem to proto
void Spline1dSolver::GenerateProblemProto(
    acu::planning::QuadraticProgrammingProblem* const qp_proto) const {
  const MatrixXd& kernel_matrix = kernel_.kernel_matrix();
  const MatrixXd& offset = kernel_.offset();
  const MatrixXd& inequality_constraint_matrix =
      constraint_.inequality_constraint().constraint_matrix();
  const MatrixXd& inequality_constraint_boundary =
      constraint_.inequality_constraint().constraint_boundary();
  const MatrixXd& equality_constraint_matrix =
      constraint_.equality_constraint().constraint_matrix();
  const MatrixXd& equality_constraint_boundary =
      constraint_.equality_constraint().constraint_boundary();

  qp_proto->set_param_size(static_cast<int32_t>(kernel_matrix.rows()));
  ConvertMatrixXdToProto(kernel_matrix, qp_proto->mutable_quadratic_matrix());
  int offset_rows_num = offset.rows();
  for (int i = 0; i < offset_rows_num; ++i) {
    qp_proto->add_bias(offset(i, 0));
  }

  ConvertMatrixXdToProto(inequality_constraint_matrix,
                         qp_proto->mutable_inequality_matrix());
  int inequality_constraint_boundary_rows_size = inequality_constraint_boundary.rows();
  for (int i = 0; i < inequality_constraint_boundary_rows_size; ++i) {
    qp_proto->add_inequality_value(inequality_constraint_boundary(i, 0));
  }

  ConvertMatrixXdToProto(equality_constraint_matrix,
                         qp_proto->mutable_equality_matrix());
  int equality_constraint_boundary_rows_num = equality_constraint_boundary.rows();
  for (int i = 0; i < equality_constraint_boundary_rows_num; ++i) {
    qp_proto->add_equality_value(equality_constraint_boundary(i, 0));
  }

  // add the optimal solution
  const auto& splines = spline_.splines();
  for (const auto& spline_seg : splines) {
    const auto& param_seg = spline_seg.spline_func().params();
    for (const auto value : param_seg) {
      qp_proto->add_optimal_param(value);
    }
  }

  return;
}

void Spline1dSolver::ConvertMatrixXdToProto(const Eigen::MatrixXd& matrix,
                                            QPMatrix* const proto) const {
  int row_size = static_cast<int>(matrix.rows());
  int col_size = static_cast<int>(matrix.cols());

  proto->set_row_size(row_size);
  proto->set_col_size(col_size);

  for (int r = 0; r < row_size; ++r) {
    for (int c = 0; c < col_size; ++c) {
      proto->add_element(matrix(r, c));
    }
  }
}

}
}  // namespace planning
}  // namespace acu
