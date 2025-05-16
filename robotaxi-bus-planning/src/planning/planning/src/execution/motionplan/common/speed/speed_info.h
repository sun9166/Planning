/**
 * @file speed_info.h
 **/
#pragma once

#include <string>
#include <vector>

#include "pnc_point.pb.h"

namespace acu {
namespace planning {

class SpeedInfo : public std::vector<common::SpeedPoint> {//
 public:
  SpeedInfo() = default;

  explicit SpeedInfo(std::vector<common::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time,
                      common::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  virtual std::string DebugString() const;

  bool IsReverse() const {
    return reverse_;
  }

  void SetReverse(bool reverse_flag) {
    reverse_ = reverse_flag;
    return;
  }
 private:
  bool reverse_ = false;
};

}  // namespace planning
}  // namespace acu
