/**
 * @file st_point.h
 **/

#pragma once

#include <string>

#include "common/math/vec2d.h"

namespace acu {
namespace planning {

class STPoint : public common::math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const common::math::Vec2d& vec2d_point);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
};

}  // namespace planning
}  // namespace acu
