/**
 * @file piecewise_jerk_path_problem.cpp
 **/
#include "src/execution/motionplan/common/planning_gflags.h"
#include "piecewise_jerk_path_problem.h"
#include "common/base/log/include/log_impl.h"
#include <iostream>

namespace acu {
namespace planning {

PiecewiseJerkPathProblem::PiecewiseJerkPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 3>& x_init)
    : PiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

void PiecewiseJerkPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_nonzeros = num_of_variables + (n - 1);//非0元素
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;
  // 1) i ∈[0,n-1] : x(i)
  // x(i)^2 * (w_x + w_x_ref)  
  CHECK_EQ(x_bound_types_.size(),x_bounds_.size());
  for (int i = 0; i < n - 1; ++i) {
    double bound_weight = 0.0;
    double dynamic_obstacle_weight = 0.0;
    int max_bound_type = std::max(x_bound_types_.at(i).first, x_bound_types_.at(i).second);
    double dynamic_obstacle_bound_min = 
      std::fmin(fabs(dynamic_obs_bounds_.at(i).first),fabs(dynamic_obs_bounds_.at(i).second));
    // 添加的边界类型权重
    if (max_bound_type > 2 ) {
      if (max_bound_type < 6) {
        bound_weight = std::get<1>(bound_type_weights_);
      } else if (max_bound_type < 7) {
        bound_weight = std::get<2>(bound_type_weights_);
      } else {
        bound_weight = weight_obs_;
      }
    } else {
      bound_weight = std::get<0>(bound_type_weights_);
    }
    // 动态障碍物的影响
    if (dynamic_obstacle_bound_min < 10.0) {
      dynamic_obstacle_weight = weight_dynamic_obs_;
    } else {
      dynamic_obstacle_weight = 0.0;
    }
    // 变道的影响
    double weight_lane_change = 1000 * weight_x_;
    if (i < lane_change_index_) {
      weight_lane_change = 0.0;
    }

    columns[i].emplace_back(
        i, (weight_x_ + weight_x_ref_ +  bound_weight + weight_lane_change + dynamic_obstacle_weight) / (scale_factor_[0] * scale_factor_[0]));
    
    ++value_index;
  }

  // x(n-1)^2 * (w_x + w_x_ref + w_end_x)
  double weight_lane_change = 30 * weight_x_;
  if ( n - 1 < lane_change_index_) {
    weight_lane_change = 0.0;
  } 
  double bound_weight = 0.0;
  double dynamic_obstacle_weight = 0.0;
  int max_bound_type = std::max(x_bound_types_.at(n-1).first, x_bound_types_.at(n-1).second);
  double dynamic_obstacle_bound_min = 
      std::fmin(fabs(dynamic_obs_bounds_.at(n-1).first),fabs(dynamic_obs_bounds_.at(n-1).second));
  if (max_bound_type > 2 ) {
    if (max_bound_type < 6) {
      bound_weight = std::get<1>(bound_type_weights_);
    } else if (max_bound_type < 7) {
      bound_weight = std::get<2>(bound_type_weights_);
    } else {
      bound_weight = weight_obs_;
    }
  } else {
    bound_weight = std::get<0>(bound_type_weights_);
  }
  if (dynamic_obstacle_bound_min < 10.0) {
    dynamic_obstacle_weight = weight_dynamic_obs_;
  } else {
    dynamic_obstacle_weight = 0.0;
  }
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_ + weight_end_state_[0] +  bound_weight + weight_lane_change + dynamic_obstacle_weight) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // 2) i ∈[n,2n-1] : x(i)'
  // x(i)'^2 * w_dx 
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  ( weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

  // 3) i ∈[2n,3n-1] : x(i)''
  auto delta_s_square = delta_s_ * delta_s_;
  // x(i)''^2 * (w_ddx + 2 * w_dddx / delta_s^2) 
  columns[2 * n].emplace_back(2 * n,
                              (weight_ddx_ + weight_dddx_ / delta_s_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  ++value_index;
  for (int i = 1; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_ddx_ + 2.0 * weight_dddx_ / delta_s_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }
  columns[3 * n - 1].emplace_back(
      3 * n - 1,
      (weight_ddx_ + weight_dddx_ / delta_s_square + weight_end_state_[2]) /
          (scale_factor_[2] * scale_factor_[2]));
  ++value_index;

  // -2 * w_dddx / delta_s^2 * x(i)'' * x(i + 1)''
  for (int i = 0; i < n - 1; ++i) {
    columns[2 * n + i].emplace_back(2 * n + i + 1,
                                    (-2.0 * weight_dddx_ / delta_s_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    ++value_index;
  }

  CHECK_EQ(value_index, num_of_nonzeros);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      // P_data来记录 hession矩阵的元素
      P_data->push_back(row_data_pair.second * 2.0);// *2.0是因为要构造1/2xTPx + qTx这样的二次型
      // P_indices来记录各个元素所在列的行号
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void PiecewiseJerkPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 3 * n;
  q->resize(kNumParam, 0.0);


  //@pqg add dynamic obstacle weight
  for (int i = 0; i < n; ++i) {
    double dynamic_bound_weight = 0.0; 
    double dynamic_obstacle_bound_min = 
      std::fmin(fabs(dynamic_obs_bounds_.at(i).first),fabs(dynamic_obs_bounds_.at(i).second));
    if (dynamic_obstacle_bound_min < 10.0) {
      dynamic_bound_weight = weight_dynamic_obs_;
    } else {
      dynamic_bound_weight = 0.0;
    }
    

    if (fabs(dynamic_obs_bounds_.at(i).first) < 10.0 && fabs(dynamic_obs_bounds_.at(i).second) < 10.0) {//no dynamicobstacle
      q->at(i) += - 2.0 * dynamic_bound_weight * 0.5 * (dynamic_obs_bounds_[i].first + dynamic_obs_bounds_[i].second) / scale_factor_[0];
    } else if (fabs(dynamic_obs_bounds_.at(i).first) < 10.0) {//right side is dynaimc obstacle
      double x_ref = x_bounds_[i].second;
      // if (x_ref > 0 ) {
      //   x_ref = x_ref - 0.2;
      // } else {
      //   x_ref = x_ref + 0.2;
      // }
      x_ref -=  0.2;
      q->at(i) += - 2.0 * dynamic_bound_weight * x_ref / scale_factor_[0];
      // q->at(i) += - 2.0 * dynamic_bound_weight * x_bounds_[i].second / scale_factor_[0];//origin
    } else if (fabs(dynamic_obs_bounds_.at(i).second) < 10.0) {//left side is dynamic obstacle
      double x_ref = x_bounds_[i].first;
      // if (x_ref <= 0 ) {
      //   x_ref = x_ref + 0.2;
      // } else {
      //   x_ref = x_ref - 0.2;
      // }
      x_ref += 0.2;
      q->at(i) += - 2.0 * dynamic_bound_weight * x_ref / scale_factor_[0];
      // q->at(i) += - 2.0 * dynamic_bound_weight * x_bounds_[i].first / scale_factor_[0];//origin
    } else {
      q->at(i) += 0.0;
    }
  }

  //@pqg 
  CHECK_EQ(x_bound_types_.size(),x_bounds_.size());
  for (int i = 0; i < n; ++i) {
    double bound_weight = 0.0; 
    int max_bound_type = std::max(x_bound_types_.at(i).first, x_bound_types_.at(i).second);
    if (max_bound_type > 2 ) {
      if (max_bound_type < 6) {
        bound_weight = std::get<1>(bound_type_weights_);
      } else if (max_bound_type < 7) {
        bound_weight = std::get<2>(bound_type_weights_);
      } else {
        bound_weight = weight_obs_;
      }
    } else {
      bound_weight = std::get<0>(bound_type_weights_);
    }
    if (x_bound_types_.at(i).first > 2 && x_bound_types_.at(i).second > 2) {//left and right side are obstacle
      q->at(i) += - 2.0 * bound_weight * 0.5 * (x_bounds_[i].first + x_bounds_[i].second) / scale_factor_[0];
    } else if (x_bound_types_.at(i).first > 2) {//right side is obstacle
      q->at(i) += - 2.0 * bound_weight * x_bounds_[i].second / scale_factor_[0];
    } else if (x_bound_types_.at(i).second > 2) {//left side are obstacle
      q->at(i) += - 2.0 * bound_weight * x_bounds_[i].first / scale_factor_[0];
    } else {
      q->at(i) += 0.0;
    }
  }

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      q->at(i) += -2.0 * weight_x_ref_ * x_ref_[i] / scale_factor_[0];
    }
  }

  if (has_end_state_ref_) {
    q->at(n - 1) +=
        -2.0 * weight_end_state_[0] * end_state_ref_[0] / scale_factor_[0];
    q->at(2 * n - 1) +=
        -2.0 * weight_end_state_[1] * end_state_ref_[1] / scale_factor_[1];
    q->at(3 * n - 1) +=
        -2.0 * weight_end_state_[2] * end_state_ref_[2] / scale_factor_[2];
  }
}

void PiecewiseJerkPathProblem::CalculateAffineConstraint(std::vector<c_float>* A_data,
                  std::vector<c_int>* A_indices, std::vector<c_int>* A_indptr, 
                  std::vector<c_float>* lower_bounds, std::vector<c_float>* upper_bounds) {
                                         
  // 3N params bounds on x, x', x''
  // 3(N-1) constraints on x, x', x''
  // 3 constraints on x_init_
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);

  int constraint_index = 0;
  // set x, x', x'' bounds
  for (int i = 0; i < num_of_variables; ++i) {
    if (i < n) {
      variables[i].emplace_back(constraint_index, 1.0);
      //添加约束 20230506  dzx
      variables[n+i].emplace_back(constraint_index, vehicle_constraint_);
      //end
      lower_bounds->at(constraint_index) =
          x_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(constraint_index) =
          x_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      variables[i].emplace_back(constraint_index, 1.0);

      lower_bounds->at(constraint_index) =
          dx_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(constraint_index) =
          dx_bounds_[i - n].second * scale_factor_[1];
    } else {
      variables[i].emplace_back(constraint_index, 1.0);
      lower_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(constraint_index) =
          ddx_bounds_[i - 2 * n].second * scale_factor_[2];
    }
    ++constraint_index;
  }
  CHECK_EQ(constraint_index, num_of_variables);

  // x" 加速度约束
  // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
  for (int i = 0; i + 1 < n; ++i) {
    variables[2 * n + i].emplace_back(constraint_index, -1.0);
    variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) =
        dddx_bound_.first * delta_s_ * scale_factor_[2];
    upper_bounds->at(constraint_index) =
        dddx_bound_.second * delta_s_ * scale_factor_[2];
    ++constraint_index;
  }

  // x' 速度连续性约束
  // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
  for (int i = 0; i + 1 < n; ++i) {
    variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
    variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
    variables[2 * n + i].emplace_back(constraint_index,
                                      -0.5 * delta_s_ * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // x 位置连续性约束
  // x(i+1) =  x(i) + delta_s * x(i)' + 1/3* delta_s^2 * x(i)'' + 1/6* delta_s^2 * x(i+1)''
  auto delta_s_sq_ = delta_s_ * delta_s_;
  for (int i = 0; i + 1 < n; ++i) {
    variables[i].emplace_back(constraint_index,
                              -1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[i + 1].emplace_back(constraint_index,
                                  1.0 * scale_factor_[1] * scale_factor_[2]);
    variables[n + i].emplace_back(
        constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
    variables[2 * n + i].emplace_back(
        constraint_index,
        -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
    variables[2 * n + i + 1].emplace_back(
        constraint_index,
        -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

    lower_bounds->at(constraint_index) = 0.0;
    upper_bounds->at(constraint_index) = 0.0;
    ++constraint_index;
  }

  // 初始状态约束
  // constrain on x_init
  variables[0].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
  ++constraint_index;

  variables[n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
  ++constraint_index;

  variables[2 * n].emplace_back(constraint_index, 1.0);
  lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
  ++constraint_index;

  CHECK_EQ(constraint_index, num_of_constraints);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      // coefficient
      A_data->push_back(variable_nz.second);

      // constraint index
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  // We indeed need this line because of
  // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
  A_indptr->push_back(ind_p);
}

}  // namespace planning
}  // namespace acu
