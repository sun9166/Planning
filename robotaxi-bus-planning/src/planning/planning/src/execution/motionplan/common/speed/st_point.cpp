/**
 * @file st_point.cpp
 **/

#include "st_point.h"
#include "common/util/string_util.h"

namespace acu {
namespace planning {


STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const common::math::Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { return set_y(s); }

void STPoint::set_t(const double t) { return set_x(t); }

}  // namespace planning
}  // namespace acu
