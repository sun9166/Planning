/**
 * @file : linear_interpolation.cpp
 * @brief: 
 **/

#include <cmath>
#include "linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "common/math/linear_interpolation.h"

namespace acu {
namespace planning {
namespace math {

common::SLPoint InterpolateUsingLinearApproximation(const common::SLPoint &p0,
                                            const common::SLPoint &p1, const double w) {
  CHECK_GE(w, 0.0);

  common::SLPoint p;
  p.set_s((1 - w) * p0.s() + w * p1.s());
  p.set_l((1 - w) * p0.l() + w * p1.l());
  return p;
}

common::PathPoint InterpolateUsingLinearApproximation(const common::PathPoint &p0,
                                              const common::PathPoint &p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();
  CHECK_LE(s0, s1);

  common::PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  double theta = common::math::slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double dr_x = (1 - weight) * p0.dr_x() + weight * p1.dr_x();
  double dr_y = (1 - weight) * p0.dr_y() + weight * p1.dr_y();
  double dr_theta = common::math::slerp(p0.dr_theta(), p0.s(), p1.dr_theta(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_dr_x(dr_x);
  path_point.set_dr_y(dr_y);
  path_point.set_dr_theta(dr_theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
}

common::TrajectoryPoint InterpolateUsingLinearApproximation(const common::TrajectoryPoint &tp0,
                                                    const common::TrajectoryPoint &tp1,
                                                    const double t) {
  if (!tp0.has_path_point() || !tp1.has_path_point()) {
    common::TrajectoryPoint p;
    p.mutable_path_point()->CopyFrom(common::PathPoint());
    return p;
  }
  const common::PathPoint pp0 = tp0.path_point();
  const common::PathPoint pp1 = tp1.path_point();
  double t0 = tp0.relative_time();
  double t1 = tp1.relative_time();

  common::TrajectoryPoint tp;
  tp.set_v(common::math::lerp(tp0.v(), t0, tp1.v(), t1, t));
  tp.set_a(common::math::lerp(tp0.a(), t0, tp1.a(), t1, t));
  tp.set_relative_time(t);
  // tp.set_steer(common::math::slerp(tp0.steer(), t0, tp0.steer(), t1, t));

  common::PathPoint *path_point = tp.mutable_path_point();
  path_point->set_x(common::math::lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(common::math::lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_theta(common::math::slerp(pp0.theta(), t0, pp1.theta(), t1, t));
  path_point->set_dr_x(common::math::lerp(pp0.dr_x(), t0, pp1.dr_x(), t1, t));
  path_point->set_dr_y(common::math::lerp(pp0.dr_y(), t0, pp1.dr_y(), t1, t));
  path_point->set_dr_theta(common::math::slerp(pp0.dr_theta(), t0, pp1.dr_theta(), t1, t));
  path_point->set_kappa(common::math::lerp(pp0.kappa(), t0, pp1.kappa(), t1, t));
  path_point->set_dkappa(common::math::lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(common::math::lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));
  path_point->set_s(common::math::lerp(pp0.s(), t0, pp1.s(), t1, t));

  return tp;
}

}  // namespace math
}  // namespace common
}  // namespace acu
