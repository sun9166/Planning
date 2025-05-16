/**
 * @file active_set_spline_1d_solver.cpp
 **/

#include <algorithm>
//#include "ros/ros.h"
#include "common/base/log/include/log.h"

#include "active_set_spline_1d_solver.h"
#include "src/execution/motionplan/common/datatype.h"
#include "common/math/qp_solver/qp_solver_gflags.h"
#include "common/math/qp_solver/active_set_qp_solver.h"
#include "src/execution/motionplan/common/planning_gflags.h"


namespace acu {
namespace planning {
namespace math {
namespace {

constexpr double kMaxBound = 1e3;

}

using Eigen::MatrixXd;

bool ActiveSetSpline1dSolver::Solve() {
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

  if (kernel_matrix.rows() != kernel_matrix.cols()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "kernel_matrix.rows() [" << kernel_matrix.rows()
           << "] and kernel_matrix.cols() [" << kernel_matrix.cols()
           << "] should be identical.";
    return false;
  }

  int num_param = static_cast<int>(kernel_matrix.rows());
  int num_constraint = static_cast<int>(equality_constraint_matrix.rows() +
                                        inequality_constraint_matrix.rows());
  bool use_hotstart =
      last_problem_success_ &&
      (FLAGS_enable_sqp_solver && sqp_solver_ != nullptr &&
       num_param == last_num_param_ && num_constraint == last_num_constraint_);

  if (!use_hotstart) {
    sqp_solver_.reset(new ::qpOASES::SQProblem(num_param, num_constraint,
                                               ::qpOASES::HST_UNKNOWN));
    ::qpOASES::Options my_options;
    my_options.enableCholeskyRefactorisation = 1;
    my_options.epsNum = -1e-7;//FLAGS_default_active_set_eps_num;
    my_options.epsDen = 1e-7;//FLAGS_default_active_set_eps_den;
    my_options.epsIterRef = 1e-7;//FLAGS_default_active_set_eps_iter_ref;
    sqp_solver_->setOptions(my_options);

    bool default_enable_active_set_debug_info = false; 
    if (!default_enable_active_set_debug_info) {
      sqp_solver_->setPrintLevel(qpOASES::PL_NONE);
    }
  }

  // definition of qpOASESproblem
  const auto kNumOfMatrixElements = kernel_matrix.rows() * kernel_matrix.cols();
  double h_matrix[kNumOfMatrixElements];  // NOLINT

  const auto kNumOfOffsetRows = offset.rows();
  double g_matrix[kNumOfOffsetRows];  // NOLINT
  int index = 0;
  int kernel_matrix_rows_size = kernel_matrix.rows();
  for (int r = 0; r < kernel_matrix_rows_size; ++r) {
    g_matrix[r] = offset(r, 0);
    int kernel_matrix_cols_size = kernel_matrix.cols();
    for (int c = 0; c < kernel_matrix_cols_size; ++c) {
      h_matrix[index++] = kernel_matrix(r, c);
    }
  }
  DCHECK_EQ(index, kernel_matrix.rows() * kernel_matrix.cols());

  // search space lower bound and uppper bound
  double lower_bound[num_param];  // NOLINT
  double upper_bound[num_param];  // NOLINT

  const double l_lower_bound_ = -kMaxBound;
  const double l_upper_bound_ = kMaxBound;
  for (int i = 0; i < num_param; ++i) {
    lower_bound[i] = l_lower_bound_;
    upper_bound[i] = l_upper_bound_;
  }

  // constraint matrix construction
  double affine_constraint_matrix[num_param * num_constraint];  // NOLINT
  double constraint_lower_bound[num_constraint];                // NOLINT
  double constraint_upper_bound[num_constraint];                // NOLINT

  index = 0;
  int equality_constraint_matrix_rows_size = equality_constraint_matrix.rows();
  for (int r = 0; r < equality_constraint_matrix_rows_size; ++r) {
    constraint_lower_bound[r] = equality_constraint_boundary(r, 0);
    constraint_upper_bound[r] = equality_constraint_boundary(r, 0);

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = equality_constraint_matrix(r, c);
    }
  }

  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param);

  const double constraint_upper_bound_ = kMaxBound;
  int inequality_constraint_matrix_rows_size = inequality_constraint_matrix.rows();
  for (int r = 0; r < inequality_constraint_matrix_rows_size; ++r) {
    constraint_lower_bound[r + equality_constraint_boundary.rows()] =
        inequality_constraint_boundary(r, 0);
    constraint_upper_bound[r + equality_constraint_boundary.rows()] =
        constraint_upper_bound_;

    for (int c = 0; c < num_param; ++c) {
      affine_constraint_matrix[index++] = inequality_constraint_matrix(r, c);
    }
  }
  DCHECK_EQ(index, equality_constraint_matrix.rows() * num_param +
                       inequality_constraint_boundary.rows() * num_param);

  // initialize problem
  int max_iteration_ = 2000;//1000;
  int max_iter = std::max(max_iteration_, num_constraint);
  AWARN_IF(FLAGS_enable_debug_motion) << "max_iter: " <<max_iter << " , num_constraint: "<<  num_constraint;
  double cpu_time = FLAGS_st_spline_solver_time_limit;
  AWARN_IF(FLAGS_enable_debug_motion) << "cpu_time: " <<cpu_time;
  ::qpOASES::returnValue ret;
  const double start_timestamp = acu::common::NodeTime::Now().ToSecond() ;//Clock::NowInSeconds();
  if (use_hotstart) {
    AWARN_IF(FLAGS_enable_debug_motion) << "using SQP hotstart.";
    ret = sqp_solver_->hotstart(
        h_matrix, g_matrix, affine_constraint_matrix, lower_bound, upper_bound,
        constraint_lower_bound, constraint_upper_bound, max_iter, &cpu_time);
    AWARN_IF(FLAGS_enable_debug_motion) << "max_iter: " <<max_iter;
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Fail to hotstart spline 1d, will use re-init instead.";
      ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                              lower_bound, upper_bound, constraint_lower_bound,
                              constraint_upper_bound, max_iter, &cpu_time);
      AWARN_IF(FLAGS_enable_debug_motion) << "max_iter: " <<max_iter;
    }
  } else {
    AWARN_IF(FLAGS_enable_debug_motion) << "no using SQP hotstart.";
    AWARN_IF(FLAGS_enable_debug_motion) << "before max_iter: " <<max_iter;
    ret = sqp_solver_->init(h_matrix, g_matrix, affine_constraint_matrix,
                            lower_bound, upper_bound, constraint_lower_bound,
                            constraint_upper_bound, max_iter, &cpu_time);
    AWARN_IF(FLAGS_enable_debug_motion) << "max_iter: " <<max_iter;
  }
  const double end_timestamp = acu::common::NodeTime::Now().ToSecond() ;//Clock::NowInSeconds();
  // ADEBUG << "ActiveSetSpline1dSolver QP solve time: "
         // << (end_timestamp - start_timestamp) * 1000 << " ms.";

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      AWARN_IF(FLAGS_enable_debug_motion) << "qpOASES solver failed due to reached max iteration <<"<<max_iter;
    } else {
      AWARN_IF(FLAGS_enable_debug_motion) << "qpOASES solver failed due to infeasibility or other internal "
                "reasons:"<< ret;
    }
    last_problem_success_ = false;
    return false;
  }

  last_problem_success_ = true;
  double result[num_param];  // NOLINT
  memset(result, 0, sizeof result);

  sqp_solver_->getPrimalSolution(result);

  MatrixXd solved_params = MatrixXd::Zero(num_param, 1);
  for (int i = 0; i < num_param; ++i) {
    solved_params(i, 0) = result[i];
    // ADEBUG << "spline 1d solved param[" << i << "]: " << result[i];
  }

  last_num_param_ = num_param;
  last_num_constraint_ = num_constraint;

  return spline_.SetSplineSegs(solved_params, spline_.spline_order());
}

}
}  // namespace planning
}  // namespace acu
