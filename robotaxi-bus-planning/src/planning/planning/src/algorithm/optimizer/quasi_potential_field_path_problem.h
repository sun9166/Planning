/**
 * @file piecewise_jerk_path_problem.h
 **/

#pragma once

#include <utility>
#include <vector>
#include <limits>

#include "src/algorithm/optimizer/piecewise_jerk/quasi_potential_field_piecewise_jerk_problem.h"

namespace acu {
namespace planning {

/*
 * @brief:
 * FEM stands for finite element method.
 * This class solve an optimization problem:
 * x
 * |
 * |                       P(s1, x1)  P(s2, x2)
 * |            P(s0, x0)                       ... P(s(k-1), x(k-1))
 * |P(start)
 * |
 * |________________________________________________________ s
 *
 * we suppose s(k+1) - s(k) == s(k) - s(k-1)
 *
 * Given the x, x', x'' at P(start),  The goal is to find x0, x1, ... x(k-1)
 * which makes the line P(start), P0, P(1) ... P(k-1) "smooth".
 */

class QuasiPotentialFieldPathProblem : public QuasiPotentialFieldPiecewiseJerkProblem {
 public:
  QuasiPotentialFieldPathProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 5>& x_init);

  virtual ~QuasiPotentialFieldPathProblem() = default;

  void set_x_bound_types(std::vector<std::pair<int, int>> x_bound_types) {
    x_bound_types_ = x_bound_types;
  }




 protected:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;


  protected:
  std::vector<std::pair<int, int>> x_bound_types_;



};

}  // namespace planning
}  // namespace acu
