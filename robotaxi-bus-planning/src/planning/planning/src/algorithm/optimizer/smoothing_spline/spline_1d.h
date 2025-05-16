/**
 * @file : spline_1d.h
 * @brief: piecewise smoothing spline class
 *       1. Model description: piecewise smoothing spline are made by pieces of
 *smoothing splines
 *          joint at knots;
 *       2. To guarantee smoothness, pieces value at knots should joint together
 *with
 *           same value, derivative, and etc. Higher the order, More smoothness
 *the piecewise spline;
 **/

#pragma once

#include <vector>
#include "spline_1d_seg.h"
#include "affine_constraint.h"
// #include "src/execution/motionplan/math/polynomial_xd.h"

namespace acu {
namespace planning {
namespace math {

class Spline1d {
 public:
  Spline1d(const std::vector<double>& x_knots, const uint32_t order);

  // @brief: given x return f(x) value, derivative, second order derivative and
  // the third order;
  double operator()(const double x) const;
  double Derivative(const double x) const;
  double SecondOrderDerivative(const double x) const;
  double ThirdOrderDerivative(const double x) const;

  // @brief: set spline segments
  bool SetSplineSegs(const Eigen::MatrixXd& param_matrix, const uint32_t order);

  const std::vector<double>& x_knots() const;
  uint32_t spline_order() const;

  const std::vector<Spline1dSeg>& splines() const;

 private:
  uint32_t FindIndex(const double x) const;

 private:
  std::vector<Spline1dSeg> splines_;
  std::vector<double> x_knots_;
  uint32_t spline_order_;
};

}
}  // namespace planning
}  // namespace acu
