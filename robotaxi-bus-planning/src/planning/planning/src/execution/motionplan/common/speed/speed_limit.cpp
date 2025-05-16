/**
 * @file speed_limit.cpp
 **/

#include "speed_limit.h"

#include <algorithm>

#include "common/base/log/include/log.h"

namespace acu {
namespace planning {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    DCHECK_GE(s, speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(s, v);
}

const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  CHECK_GE(speed_limit_points_.size(), 2);
  DCHECK_GE(s, speed_limit_points_.front().first);

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

double SpeedLimit::MinValidS() const {
  CHECK_GE(speed_limit_points_.size(), 2);
  return speed_limit_points_.front().first;
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }

//根据设定加减速度修剪速度增幅上限
void SpeedLimit::TrimSpeedLimit(const double cruise_speed, const double init_v) {
  const double deta_v = cruise_speed - init_v;
  double comfort_accel = std::min(1.5, 1.5*deta_v/cruise_speed);
  comfort_accel = std::max(comfort_accel, 0.5);
  const double comfort_decel = -0.6;
  // 变道场景下，减小加速度
  // if (BehaviorParser::instance()->target_reference_line_id() 
  //       != BehaviorParser::instance()->current_reference_line_id()) {
  //       comfort_accel = 0.5;
  // };
  const size_t speed_limit_size = speed_limit_points_.size();
  if (speed_limit_size <= 1) return;
  const double kSpeedBuffer = 0.1;
  speed_limit_points_[0].second = std::fmax(init_v + kSpeedBuffer, 1.0);
  for (int i = 1; i< speed_limit_size; i++) {
    //数组正方向约束递增幅度
    double deta_accel = (std::pow(speed_limit_points_[i].second, 2) - std::pow(speed_limit_points_[i-1].second, 2))
                    /(2*(speed_limit_points_[i].first - speed_limit_points_[i-1].first));
    if (deta_accel > comfort_accel ) {
        // 大于增速幅度的，更新增幅限制
        double new_speed_limit_acc = std::sqrt(2*comfort_accel*(speed_limit_points_[i].first - speed_limit_points_[i-1].first)
                    + std::pow(speed_limit_points_[i-1].second, 2));
        speed_limit_points_[i].second = std::fmin(speed_limit_points_[i].second, new_speed_limit_acc);
    }
    //数组反方向约束递减幅度
    double deta_decel = (std::pow(speed_limit_points_[speed_limit_size-i].second, 2) 
                          - std::pow(speed_limit_points_[speed_limit_size-i-1].second, 2))
                    /(2*(speed_limit_points_[speed_limit_size-i].first - speed_limit_points_[speed_limit_size-i-1].first));
    if (deta_decel < comfort_decel) {
      double new_speed_limit_dec = std::sqrt(-2*comfort_decel*(speed_limit_points_[speed_limit_size-i].first 
                        - speed_limit_points_[speed_limit_size-i-1].first)
                        + std::pow(speed_limit_points_[speed_limit_size-i].second, 2));
      speed_limit_points_[speed_limit_size-i-1].second = 
            std::fmin(speed_limit_points_[speed_limit_size-i-1].second, new_speed_limit_dec);
    }
  }
  //上面的流程可能会把首元素更改，重新赋值后再进下进行面的校验
  speed_limit_points_[0].second = std::fmax(init_v + kSpeedBuffer, 1.0);
  // 以起点速度，正向再校验减速度幅度,减少猛刹的情况
  for (int i = 1; i< speed_limit_size; i++) {
    double deta_decel =  (std::pow(speed_limit_points_[i].second, 2) - std::pow(speed_limit_points_[i-1].second, 2))
                    /(2*(speed_limit_points_[i].first - speed_limit_points_[i-1].first));
    if (deta_decel >= 0) break;
    if (deta_decel < comfort_decel) {
      double new_speed_limit = std::sqrt(2*comfort_decel*(speed_limit_points_[i].first - speed_limit_points_[i-1].first)
                    + std::pow(speed_limit_points_[i-1].second, 2));
      speed_limit_points_[i].second = std::fmax(speed_limit_points_[i].second, new_speed_limit);
    }
  }
  return;
}

}  // namespace planning
}  // namespace acu
