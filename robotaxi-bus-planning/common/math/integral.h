/******************************************************************************
 * Copyright 2017 The acu Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Functions to compute integral.
 */

#ifndef MODULES_COMMON_MATH_INTEGRAL_H_
#define MODULES_COMMON_MATH_INTEGRAL_H_

#include <functional>
#include <vector>

/**
 * @namespace acu::common::math
 * @brief acu::common::math
 */
namespace acu {
namespace common {
namespace math {

double IntegrateBySimpson(const std::vector<double>& funv_vec, const double dx,
                          const std::size_t nsteps);

double IntegrateByTrapezoidal(const std::vector<double>& funv_vec,
                              const double dx, const std::size_t nsteps);
/**
 * @brief Compute the integral of a target single-variable function
 *        from a lower bound to an upper bound, by 5-th Gauss-Legendre method
 * Given a target function and integral lower and upper bound,
 * compute the integral approximation using 5th order Gauss-Legendre
 * integration.
 * The target function must be a smooth function.
 * Example:
 * target function: auto func = [](const double& x) {return x * x;};
 *                  double integral = gauss_legendre(func, -2, 3);
 * This gives you the approximated integral of function x^2 in bound [-2, 3]
 *
 * reference: https://en.wikipedia.org/wiki/Gaussian_quadrature
 *            http://www.mymathlib.com/quadrature/gauss_legendre.html
 *
 * @param func The target single-variable function
 * @param lower_bound The lower bound of the integral
 * @param upper_bound The upper bound of the integral
 * @return The integral result
 */
double IntegrateByGaussLegendre(const std::function<double(double)>& func,
                                const double lower_bound,
                                const double upper_bound);

}  // namespace math
}  // namespace common
}  // namespace acu

#endif  // MODULES_COMMON_MATH_INTEGRAL_H_
