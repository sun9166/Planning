/**
 * @file piecewise_jerk_path_problem.cpp
 **/
#include "src/execution/motionplan/common/planning_gflags.h"
#include "quasi_potential_field_path_problem.h"
#include "common/base/log/include/log_impl.h"
#include "common/base/log/include/log.h"
#include <iostream>

namespace acu {
namespace planning {

QuasiPotentialFieldPathProblem::QuasiPotentialFieldPathProblem(
    const size_t num_of_knots, const double delta_s,
    const std::array<double, 5>& x_init)
    : QuasiPotentialFieldPiecewiseJerkProblem(num_of_knots, delta_s, x_init) {}

void QuasiPotentialFieldPathProblem::CalculateKernel(std::vector<c_float>* P_data,
                                               std::vector<c_int>* P_indices,
                                               std::vector<c_int>* P_indptr) {
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 3 * n;
  const int num_of_softlimits = 2 * n;
  const int num_of_nonzeros = num_of_variables + num_of_softlimits + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables + num_of_softlimits);
  int value_index = 0;
  //https://www.cnblogs.com/icathianrain/p/14407626.html
  // 从以下角度出发来规划和评价path：
  // 1 无碰撞
  // 2 最小横向偏差 - 沿中心线行驶 wobs⋅∫(l(s)−0.5∗(lB(s)min+lB(s)max))2ds  这里没有加这一部分
  // 3 最小横向位移 - 降低横向位移和位移频率
  // 4 （可选）远离障碍物  
  // 上述设置了目标函数，主要包括：l^2+l'^2+l''^2，以及l'''^2，其中l'''通过l''前后两帧之差与delta_s之比替代

  // x(i)^2 * (w_x + w_x_ref[i]), w_x_ref might be a uniform value for all x(i)
  // or piecewise values for different x(i)
  for (int i = 0; i < n - 1; ++i) {
    columns[i].emplace_back(i, (weight_x_ + weight_x_ref_vec_[i]) /
                                   (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // x(n-1)^2 * (w_x + w_x_ref[n-1] + w_end_x)
  columns[n - 1].emplace_back(
      n - 1, (weight_x_ + weight_x_ref_vec_[n - 1] + weight_end_state_[0]) /
                 (scale_factor_[0] * scale_factor_[0]));
  ++value_index;

  // x(i)'^2 * w_dx
  for (int i = 0; i < n - 1; ++i) {
    columns[n + i].emplace_back(
        n + i, weight_dx_ / (scale_factor_[1] * scale_factor_[1]));
    ++value_index;
  }
  // x(n-1)'^2 * (w_dx + w_end_dx)
  columns[2 * n - 1].emplace_back(2 * n - 1,
                                  (weight_dx_ + weight_end_state_[1]) /
                                      (scale_factor_[1] * scale_factor_[1]));
  ++value_index;

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

  double left_delt_l_weight = 0;
  // left_delt_l(i)^2 * w_left_delt_l
  for (int i = 0; i < n - 1; ++i) {
    // AERROR << "x_bound_types_.at(i).second = " << x_bound_types_.at(i).second;
    if(x_bound_types_.at(i).second < 3){
      left_delt_l_weight = weight_dotted_line_;
    }else if(x_bound_types_.at(i).second < 6){
      left_delt_l_weight = weight_solid_line_;
    }else if(x_bound_types_.at(i).second < 7){
      left_delt_l_weight = weight_curb_;
    }else if(x_bound_types_.at(i).second < 9){
      left_delt_l_weight = weight_obstacle_;
    }
    columns[3 * n + i].emplace_back(
        3 * n + i, left_delt_l_weight / (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // left_delt_l(n-i)^2 * w_left_delt_l
  if(x_bound_types_.at(n - 1).second < 3){
    left_delt_l_weight = weight_dotted_line_;
  }else if(x_bound_types_.at(n - 1).second < 6){
    left_delt_l_weight = weight_solid_line_;
  }else if(x_bound_types_.at(n - 1).second < 7){
    left_delt_l_weight = weight_curb_;
  }else if(x_bound_types_.at(n - 1).second < 9){
    left_delt_l_weight = weight_obstacle_;
  }
  columns[4 * n - 1].emplace_back(4 * n - 1,
                                  (left_delt_l_weight + weight_end_state_[3]) /
                                      (scale_factor_[0] * scale_factor_[0]));
  ++value_index;
  double right_delt_l_weight = 0;
  // right_delt_l(i)^2 * w_right_delt_l
  for (int i = 0; i < n - 1; ++i) {
    // AERROR << "x_bound_types_.at(i).first = " << x_bound_types_.at(i).first;
    if(x_bound_types_.at(i).first < 3){
      right_delt_l_weight = weight_dotted_line_;
    }else if(x_bound_types_.at(i).first < 6){
      right_delt_l_weight = weight_solid_line_;
    }else if(x_bound_types_.at(i).first < 7){
      right_delt_l_weight = weight_curb_;
    }else if(x_bound_types_.at(i).first < 9){
      right_delt_l_weight = weight_obstacle_;
    }
    columns[4 * n + i].emplace_back(
        4 * n + i, right_delt_l_weight / (scale_factor_[0] * scale_factor_[0]));
    ++value_index;
  }
  // right_delt_l(n-i)^2 * w_right_delt_l
  if(x_bound_types_.at(n - 1).first < 3){
    right_delt_l_weight = weight_dotted_line_;
  }else if(x_bound_types_.at(n - 1).first < 6){
    right_delt_l_weight = weight_solid_line_;
  }else if(x_bound_types_.at(n - 1).first < 7){
    right_delt_l_weight = weight_curb_;
  }else if(x_bound_types_.at(n - 1).first < 9){
    right_delt_l_weight = weight_obstacle_;
  }
  columns[5 * n - 1].emplace_back(5 * n - 1,
                                  (right_delt_l_weight + weight_end_state_[4]) /
                                      (scale_factor_[0] * scale_factor_[0]));
  ++value_index;
  CHECK_EQ(value_index, num_of_nonzeros);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables + num_of_softlimits; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second * 2.0);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

//计算代价
void QuasiPotentialFieldPathProblem::CalculateOffset(std::vector<c_float>* q) {
  CHECK_NOTNULL(q);
  const int n = static_cast<int>(num_of_knots_);
  const int kNumParam = 5 * n;
  q->resize(kNumParam, 0.0);

  if (has_x_ref_) {
    for (int i = 0; i < n; ++i) {
      //目标函数在横向位移上有两项： l^2+(l-ref)^2,因此可以看到为什么在目标函数里，l^2的系数乘以2，
      //在这里将第二项进行了拆解，于是有了offset。 即 -2ref*i，这个就对应了。各位细品。
      //至于为什么不考虑ref^2，因为它是个非负实数，并不包含任何变量，因此不影响梯度下降，
      //从而不影响整个函数的求解。因此在此处省略
      q->at(i) += -2.0 * weight_x_ref_vec_.at(i) * x_ref_[i] / scale_factor_[0];
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


}  // namespace planning
}  // namespace acu
