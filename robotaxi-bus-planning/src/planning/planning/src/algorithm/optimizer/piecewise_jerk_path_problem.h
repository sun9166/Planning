/**
 * @file piecewise_jerk_path_problem.h
 **/

#pragma once

#include <utility>
#include <vector>
#include <limits>

#include "src/algorithm/optimizer/piecewise_jerk/piecewise_jerk_problem.h"

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

class PiecewiseJerkPathProblem : public PiecewiseJerkProblem {
 public:
  PiecewiseJerkPathProblem(const size_t num_of_knots, const double delta_s,
                           const std::array<double, 3>& x_init);

  virtual ~PiecewiseJerkPathProblem() = default;
  void set_x_bound_types(std::vector<std::pair<int, int>> x_bound_types) {
  	x_bound_types_ = x_bound_types;
  }
  void set_dynamic_obstacle_bounds(std::vector<std::pair<double, double>> obs_bounds) {
    dynamic_obs_bounds_ = obs_bounds;
  }
  void set_lane_change_index(int index) {
    lane_change_index_ = index;
  }

  void set_weight_obs(const double weight_obs) { weight_obs_ = weight_obs; }

  void set_weight_dynamic_obs(const double weight_obs) { weight_dynamic_obs_ = weight_obs; }

  void set_weight_bound_type(const std::tuple<double, double, double> bound_type_weights) { 
       bound_type_weights_ = bound_type_weights; }

  void set_vehicle_constraint(const double vehicle_constraint) {vehicle_constraint_ = vehicle_constraint;}

 protected:
  void CalculateKernel(std::vector<c_float>* P_data,
                       std::vector<c_int>* P_indices,
                       std::vector<c_int>* P_indptr) override;

  void CalculateOffset(std::vector<c_float>* q) override;

  void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                         std::vector<c_int>* A_indices,
                                         std::vector<c_int>* A_indptr,
                                         std::vector<c_float>* lower_bounds,
                                         std::vector<c_float>* upper_bounds) override;
 protected:
   std::vector<std::pair<int, int>> x_bound_types_;
   int lane_change_index_ = std::numeric_limits<int>::max(); 
   double weight_obs_ = 0.0;
   double weight_dynamic_obs_ = 500.0;
   double vehicle_constraint_ = 0.0;
   // contains: (dotted_line weight, solid_line weight, curb weight).
   std::tuple<double, double, double> bound_type_weights_ = std::make_tuple(0.0,0.0,0.0);//std::make_tuple(0.0,3.0,5.0);
   std::vector<std::pair<double, double>> dynamic_obs_bounds_;
};

}  // namespace planning
}  // namespace acu
