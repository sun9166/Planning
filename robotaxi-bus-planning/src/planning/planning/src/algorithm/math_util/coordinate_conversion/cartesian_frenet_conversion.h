/**
 * @file cartesian_frenet_conversion.h
 **/

#pragma once

#include <array>

#include "common/math/vec2d.h"

namespace acu {
namespace planning {
namespace math {
void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const double x, const double y,
                                  const double v, const double a,
                                  const double theta, const double kappa,
                                  std::array<double, 3>* const ptr_s_condition,
                                  std::array<double, 3>* const ptr_d_condition);

void cartesian_to_frenet(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double x, const double y, double* ptr_s,
                                  double* ptr_d);
void frenet_to_cartesian(const double rs, const double rx,
                                  const double ry, const double rtheta,
                                  const double rkappa, const double rdkappa,
                                  const std::array<double, 3>& s_condition,
                                  const std::array<double, 3>& d_condition,
                                  double* const ptr_x, double* const ptr_y,
                                  double* const ptr_theta,
                                  double* const ptr_kappa, double* const ptr_v,
                                  double* const ptr_a);

double CalculateCartesianTheta(const double rtheta, const double rkappa,
                               const double l, const double dl);

double CalculateCartesianKappa(const double rkappa, const double rdkappa,
                               const double l, const double dl,
                               const double ddl);

common::math::Vec2d CalculateCartesianPoint(const double rtheta, const common::math::Vec2d& rpoint,
                                       const double l);

double CalculateFrenetLateralDerivative(const double theta_ref,
                                           const double theta, const double l,
                                           const double kappa_ref);

double CalculateFrenetSecondOrderLateralDerivative(
      const double theta_ref, const double theta, const double kappa_ref,
      const double kappa, const double dkappa_ref, const double l);


}  // namespace math
}  // namespace common
}  // namespace acu
