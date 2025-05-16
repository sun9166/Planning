/**
 * @file speed_info.cpp
 **/


#include <algorithm>
#include <utility>
#include "speed_info.h"
#include "common/util/util.h"
#include "common/util/string_util.h"
#include "common/math/linear_interpolation.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/algorithm/math_util/interpolation/linear_interpolation.h"

namespace acu {
namespace planning {

using acu::common::SpeedPoint;

SpeedInfo::SpeedInfo(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {//该函数实现的功能是按时间进行排序
  std::sort(begin(), end(), [](const SpeedPoint& p1, const SpeedPoint& p2) {
    return p1.t() < p2.t();
  });
}

void SpeedInfo::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!empty()) {
    // CHECK(back().t() < time);
  }
  push_back(common::util::MakeSpeedPoint(s, time, v, a, da));
}

bool SpeedInfo::EvaluateByTime(const double t,
                               common::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (  !( front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t() )  ) {
    return false;
  }

  auto comp = [](const common::SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);//找到与输入参数t更近的一个点。
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);//该点的前一个点，
    const auto& p1 = *it_lower;//当前点
    double t0 = p0.t();
    double t1 = p1.t();

    common::SpeedPoint res;
    res.set_t(t);

    double s = common::math::lerp(p0.s(), t0, p1.s(), t1, t);//线性插值得到s
    res.set_s(s);

    if (p0.has_v() && p1.has_v()) {
      double v = common::math::lerp(p0.v(), t0, p1.v(), t1, t);//线性插值得到v
      res.set_v(v);
    }

    if (p0.has_a() && p1.has_a()) {//线性插值得到a
      double a = common::math::lerp(p0.a(), t0, p1.a(), t1, t);
      res.set_a(a);
    }

    if (p0.has_da() && p1.has_da()) {//线性插值得到da
      double da = common::math::lerp(p0.da(), t0, p1.da(), t1, t);
      res.set_da(da);
    }

    *speed_point = res;//进行赋值，输出speed_point
  }
  return true;
}

double SpeedInfo::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

std::string SpeedInfo::DebugString() const {
  const auto limit = std::min(
      size(), static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return acu::common::util::StrCat(
      "[\n", acu::common::util::PrintDebugStringIter(begin(),
                                                        begin() + limit, ",\n"),
      "]\n");
}

}  // namespace planning
}  // namespace acu
