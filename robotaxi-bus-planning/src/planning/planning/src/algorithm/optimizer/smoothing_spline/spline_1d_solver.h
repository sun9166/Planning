/**
 * @file spline_1d_solver.h
 **/

#pragma once

#include <vector>
#include "Eigen/Core"
#include "spline_1d.h"
#include "qp_problem.pb.h"
#include "spline_1d_kernel.h"
#include "spline_1d_constraint.h"
#include "common/math/qp_solver/qp_solver.h"

namespace acu {
namespace planning {
namespace math {

class Spline1dSolver {
 public:
  Spline1dSolver(const std::vector<double>& x_knots, const uint32_t order)
      : spline_(x_knots, order),
        constraint_(x_knots, order),
        kernel_(x_knots, order) {}

  virtual void Reset(const std::vector<double>& x_knots, const uint32_t order) {
    spline_ = Spline1d(x_knots, order);
    constraint_ = Spline1dConstraint(x_knots, order);
    kernel_ = Spline1dKernel(x_knots, order);
  }

  virtual Spline1dConstraint* mutable_spline_constraint() {
    return &constraint_;
  }

  virtual Spline1dKernel* mutable_spline_kernel() { return &kernel_; }

  virtual bool Solve() = 0;

  // output
  virtual const Spline1d& spline() const { return spline_; }

  // convert qp problem to proto
  void GenerateProblemProto(acu::planning::QuadraticProgrammingProblem* const qp_proto) const;

 protected:
  void ConvertMatrixXdToProto(const Eigen::MatrixXd& matrix,
                              QPMatrix* const proto) const;

 protected:
  Spline1d spline_;
  Spline1dConstraint constraint_;
  Spline1dKernel kernel_;

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
};

}
}  // namespace planning
}  // namespace acu
