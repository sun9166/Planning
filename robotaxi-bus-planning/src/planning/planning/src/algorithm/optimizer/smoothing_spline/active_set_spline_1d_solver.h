/**
 * @file active_set_spline_1d_solver.h
 **/

#pragma once

#include <memory>
#include <vector>

#include "spline_1d_solver.h"
#include "common/common_lib/qpOASES.hpp"
#include "common/math/qp_solver/qp_solver.h"

namespace acu {
namespace planning {
namespace math {

class ActiveSetSpline1dSolver : public Spline1dSolver {
 public:
  ActiveSetSpline1dSolver(const std::vector<double>& x_knots,
                          const uint32_t order)
      : Spline1dSolver(x_knots, order) {}

  bool Solve() override;

 private:
  std::unique_ptr<::qpOASES::SQProblem> sqp_solver_;
};

}
}  // namespace planning
}  // namespace acu
