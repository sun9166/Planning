

#include "common/toolbox/optimizer/include/curves/reeds_shepp_curve.h"
#include <boost/math/constants/constants.hpp>

// The comments, variable names, etc. use the nomenclature from the Reeds &
// Shepp paper.
namespace toolbox {
namespace rs {
const double pi = boost::math::constants::pi<double>();
const double twopi = 2. * pi;
const double RS_EPS = 1e-6;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();

inline double ReedsSheppStateSpace::mod2pi(double x) {
  double v = fmod(x, twopi);
  if (v < -pi)
    v += twopi;
  else if (v > pi)
    v -= twopi;
  return v;
}
inline void ReedsSheppStateSpace::polar(double x, double y, double &r,
                                        double &theta) {
  r = sqrt(x * x + y * y);
  theta = atan2(y, x);
}
inline void ReedsSheppStateSpace::tauOmega(double u, double v, double xi,
    double eta, double phi, double &tau,
    double &omega) {
  double delta = mod2pi(u - v), A = sin(u) - sin(delta),
         B = cos(u) - cos(delta) - 1.;
  double t1 = atan2(eta * A - xi * B, xi * A + eta * B),
         t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
  tau = (t2 < 0) ? mod2pi(t1 + pi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// formula 8.1 in Reeds-Shepp paper
inline bool ReedsSheppStateSpace::LpSpLp(double x, double y, double phi,
    double &t, double &u, double &v) {
  polar(x - sin(phi), y - 1. + cos(phi), u, t);
  if (t >= -ZERO) {
    v = mod2pi(phi - t);
    if (v >= -ZERO) {
      assert(fabs(u * cos(t) + sin(phi) - x) < RS_EPS);
      assert(fabs(u * sin(t) - cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t + v - phi)) < RS_EPS);
      return true;
    }
  }
  return false;
}
// formula 8.2
inline bool ReedsSheppStateSpace::LpSpRp(double x, double y, double phi,
    double &t, double &u, double &v) {
  double t1, u1;
  polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.) {
    double theta;
    u = sqrt(u1 - 4.);
    theta = atan2(2., u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    assert(fabs(2 * sin(t) + u * cos(t) - sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) + u * sin(t) + cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
    return t >= -ZERO && v >= -ZERO;
  }
  return false;
}
void ReedsSheppStateSpace::CSC(double x, double y, double phi,
                               ReedsSheppStateSpace::ReedsSheppPath &path) {
  // double t, u, v, Lmin = path.length(), L;
  double t, u, v, L;//fixed by zhangzhuo [0713]
  if (LpSpLp(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[14], t, u, v);
    pathqueue.emplace(path);
  }
  if (LpSpLp(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[14], -t, -u, -v);
    pathqueue.emplace(path);
  }
  if (LpSpLp(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[15], t, u, v);

    pathqueue.emplace(path);
  }
  if (LpSpLp(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[15], -t, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpSpRp(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[12], t, u, v);

    pathqueue.emplace(path);
  }
  if (LpSpRp(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[12], -t, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpSpRp(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[13], t, u, v);

    pathqueue.emplace(path);
  }
  if (LpSpRp(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[13], -t, -u, -v);

    pathqueue.emplace(path);
  }
}
// formula 8.3 / 8.4  *** TYPO IN PAPER ***
inline bool ReedsSheppStateSpace::LpRmL(double x, double y, double phi,
                                        double &t, double &u, double &v) {
  double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.) {
    u = -2. * asin(.25 * u1);
    t = mod2pi(theta + .5 * u + pi);
    v = mod2pi(phi - t + u);
    assert(fabs(2 * (sin(t) - sin(t - u)) + sin(phi) - x) < RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u)) - cos(phi) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t - u + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO;
  }
  return false;
}
void ReedsSheppStateSpace::CCC(double x, double y, double phi,
                               ReedsSheppStateSpace::ReedsSheppPath &path) {
  // double t, u, v, Lmin = path.length(), L;
  double t, u, v, L;//fixed by zhangzhuo [0713]
  if (LpRmL(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[0], t, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmL(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[0], -t, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpRmL(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[1], t, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmL(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[1], -t, -u, -v);

    pathqueue.emplace(path);
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[0], v, u, t);

    pathqueue.emplace(path);
  }
  if (LpRmL(-xb, yb, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[0], -v, -u, -t);

    pathqueue.emplace(path);
  }
  if (LpRmL(xb, -yb, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[1], v, u, t);

    pathqueue.emplace(path);
  }
  if (LpRmL(-xb, -yb, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[1], -v, -u, -t);

    pathqueue.emplace(path);
  }
}
// formula 8.7
inline bool ReedsSheppStateSpace::LpRupLumRm(double x, double y, double phi,
    double &t, double &u, double &v) {
  double xi = x + sin(phi), eta = y - 1. - cos(phi),
         rho = .25 * (2. + sqrt(xi * xi + eta * eta));
  if (rho <= 1.) {
    u = acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    assert(fabs(2 * (sin(t) - sin(t - u) + sin(t - 2 * u)) - sin(phi) - x) <
           RS_EPS);
    assert(fabs(2 * (-cos(t) + cos(t - u) - cos(t - 2 * u)) + cos(phi) + 1 -
                y) < RS_EPS);
    assert(fabs(mod2pi(t - 2 * u - v - phi)) < RS_EPS);
    return t >= -ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.8
inline bool ReedsSheppStateSpace::LpRumLumRp(double x, double y, double phi,
    double &t, double &u, double &v) {
  double xi = x + sin(phi), eta = y - 1. - cos(phi),
         rho = (20. - xi * xi - eta * eta) / 16.;
  if (rho >= 0 && rho <= 1) {
    u = -acos(rho);
    if (u >= -.5 * pi) {
      tauOmega(u, u, xi, eta, phi, t, v);
      assert(fabs(4 * sin(t) - 2 * sin(t - u) - sin(phi) - x) < RS_EPS);
      assert(fabs(-4 * cos(t) + 2 * cos(t - u) + cos(phi) + 1 - y) < RS_EPS);
      assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void ReedsSheppStateSpace::CCCC(double x, double y, double phi,
                                ReedsSheppStateSpace::ReedsSheppPath &path) {
  // double t, u, v, Lmin = path.length(), L;
  double t, u, v, L;//fixed by zhangzhuo [0713]
  if (LpRupLumRm(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[2], t, u, -u, v);

    pathqueue.emplace(path);
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, u, -v);

    pathqueue.emplace(path);
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[3], t, u, -u, v);

    pathqueue.emplace(path);
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, u, -v);

    pathqueue.emplace(path);
  }

  if (LpRumLumRp(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[2], t, u, u, v);

    pathqueue.emplace(path);
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[2], -t, -u, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[3], t, u, u, v);

    pathqueue.emplace(path);
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[3], -t, -u, -u, -v);

    pathqueue.emplace(path);
  }
}
// formula 8.9
inline bool ReedsSheppStateSpace::LpRmSmLm(double x, double y, double phi,
    double &t, double &u, double &v) {
  double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    double r = sqrt(rho * rho - 4.);
    u = 2. - r;
    t = mod2pi(theta + atan2(r, -2.));
    v = mod2pi(phi - .5 * pi - t);
    assert(fabs(2 * (sin(t) - cos(t)) - u * sin(t) + sin(phi) - x) < RS_EPS);
    assert(fabs(-2 * (sin(t) + cos(t)) + u * cos(t) - cos(phi) + 1 - y) <
           RS_EPS);
    assert(fabs(mod2pi(t + pi / 2 + v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
// formula 8.10
inline bool ReedsSheppStateSpace::LpRmSmRm(double x, double y, double phi,
    double &t, double &u, double &v) {
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.) {
    t = theta;
    u = 2. - rho;
    v = mod2pi(t + .5 * pi - phi);
    assert(fabs(2 * sin(t) - cos(t - v) - u * sin(t) - x) < RS_EPS);
    assert(fabs(-2 * cos(t) - sin(t - v) + u * cos(t) + 1 - y) < RS_EPS);
    assert(fabs(mod2pi(t + pi / 2 - v - phi)) < RS_EPS);
    return t >= -ZERO && u <= ZERO && v <= ZERO;
  }
  return false;
}
void ReedsSheppStateSpace::CCSC(double x, double y, double phi,
                                ReedsSheppStateSpace::ReedsSheppPath &path) {
  // double t, u, v, Lmin = path.length(), L;
  double t, u, v, L;//fixed by zhangzhuo [0713]
  if (LpRmSmLm(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[4], t, -.5 * pi, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[4], -t, .5 * pi, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[5], t, -.5 * pi, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[5], -t, .5 * pi, -u, -v);

    pathqueue.emplace(path);
  }

  if (LpRmSmRm(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[8], t, -.5 * pi, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[8], -t, .5 * pi, -u, -v);

    pathqueue.emplace(path);
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[9], t, -.5 * pi, u, v);

    pathqueue.emplace(path);
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[9], -t, .5 * pi, -u, -v);

    pathqueue.emplace(path);
  }

  // backwards
  double xb = x * cos(phi) + y * sin(phi), yb = x * sin(phi) - y * cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[6], v, u, -.5 * pi, t);
    pathqueue.emplace(path);
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[6], -v, -u, .5 * pi, -t);
    pathqueue.emplace(path);
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[7], v, u, -.5 * pi, t);
    pathqueue.emplace(path);
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[7], -v, -u, .5 * pi, -t);
    pathqueue.emplace(path);
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[10], v, u, -.5 * pi, t);
    pathqueue.emplace(path);
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[10], -v, -u, .5 * pi, -t);
    pathqueue.emplace(path);
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[11], v, u, -.5 * pi, t);
    pathqueue.emplace(path);
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[11], -v, -u, .5 * pi, -t);
    pathqueue.emplace(path);
  }
}
// formula 8.11 *** TYPO IN PAPER ***
inline bool ReedsSheppStateSpace::LpRmSLmRp(double x, double y, double phi,
    double &t, double &u, double &v) {
  double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.) {
    u = 4. - sqrt(rho * rho - 4.);
    if (u <= ZERO) {
      t = mod2pi(atan2((4 - u) * xi - 2 * eta, -2 * xi + (u - 4) * eta));
      v = mod2pi(t - phi);
      assert(fabs(4 * sin(t) - 2 * cos(t) - u * sin(t) - sin(phi) - x) <
             RS_EPS);
      assert(fabs(-4 * cos(t) - 2 * sin(t) + u * cos(t) + cos(phi) + 1 - y) <
             RS_EPS);
      assert(fabs(mod2pi(t - v - phi)) < RS_EPS);
      return t >= -ZERO && v >= -ZERO;
    }
  }
  return false;
}
void ReedsSheppStateSpace::CCSCC(double x, double y, double phi,
                                 ReedsSheppStateSpace::ReedsSheppPath &path) {
  // double t, u, v, Lmin = path.length(), L;
  double t, u, v, L;//fixed by zhangzhuo [0713]
  if (LpRmSLmRp(x, y, phi, t, u, v)) {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[16], t, -.5 * pi, u, -.5 * pi,
             v);

    pathqueue.emplace(path);
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v))  // timeflip
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[16], -t, .5 * pi, -u, .5 * pi,
             -v);

    pathqueue.emplace(path);
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v))  // reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[17], t, -.5 * pi, u, -.5 * pi,
             v);

    pathqueue.emplace(path);
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v))  // timeflip + reflect
  {
    path = ReedsSheppStateSpace::ReedsSheppPath(
             ReedsSheppStateSpace::reedsSheppPathType[17], -t, .5 * pi, -u, .5 * pi,
             -v);

    pathqueue.emplace(path);
  }
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(
  double x, double y, double phi) {
  ReedsSheppStateSpace::ReedsSheppPath path;
  while (!pathqueue.empty()) {
    pathqueue.pop();
  }
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  path = pathqueue.top();
  return path;
}

const ReedsSheppStateSpace::ReedsSheppPathSegmentType
ReedsSheppStateSpace::reedsSheppPathType[18][5] = {
  {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},         // 0
  {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},        // 1
  {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},       // 2
  {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},       // 3
  {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 4
  {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 5
  {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},    // 6
  {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},   // 7
  {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},   // 8
  {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},    // 9
  {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},   // 10
  {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},    // 11
  {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},     // 12
  {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},     // 13
  {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},      // 14
  {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},    // 15
  {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},  // 16
  {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}   // 17
};

ReedsSheppStateSpace::ReedsSheppPath::ReedsSheppPath(
  const ReedsSheppPathSegmentType *type, double t, double u, double v,
  double w, double x)
  : type_(type) {
  length_[0] = t;
  length_[1] = u;
  length_[2] = v;
  length_[3] = w;
  length_[4] = x;
  totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
}

double ReedsSheppStateSpace::distance(double q0[3], double q1[3]) {
  return rho_ * reedsShepp(q0, q1).length();
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::reedsShepp(
  double q0[3], double q1[3]) {
  double dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
  double c = cos(q0[2]), s = sin(q0[2]);
  double x = c * dx + s * dy, y = -s * dx + c * dy;
  return reedsShepp(x / rho_, y / rho_, dth);
}

void ReedsSheppStateSpace::type(double q0[3], double q1[3],
                                ReedsSheppPathTypeCallback cb,
                                void *user_data) {
  ReedsSheppPath path = reedsShepp(q0, q1);
  for (int i = 0; i < 5; ++i) cb(path.type_[i], user_data);
  return;
}

ReedsSheppStateSpace::ReedsSheppPath ReedsSheppStateSpace::sample(
  double q0[3], double q1[3], double step_size,
  ReedsSheppPathSamplingCallback cb, void *user_data) {
  ReedsSheppPath path = reedsShepp(q0, q1);
  double dist = rho_ * path.length();

  for (double seg = 0.0; seg <= dist; seg += step_size) {
    double qnew[3] = {};
    interpolate(q0, path, seg / rho_, qnew);
    cb(qnew, user_data);
  }
  return path;
}

std::vector<ReedsSheppStateSpace::ReedsSheppPath>
ReedsSheppStateSpace::sample_many(
  double q0[3], double q1[3], double step_size,
  std::vector<std::vector<geometry_msgs::Point32>> &data,
  std::vector<std::vector<geometry_msgs::Polygon>> &bounding_polygon) {
  float front_lon = 1.2;
  float back_lon = 0.4;
  float half_lat = 0.6;
  ReedsSheppPath path = reedsShepp(q0, q1);
  std::vector<ReedsSheppPath> patharray;
  double dist;
  double qnew[3] = {};
  geometry_msgs::Point32 temp;
  while (!pathqueue.empty()) {
    path = pathqueue.top();
    // The first path is the optimized shortest path!!!
    patharray.push_back(path);
    pathqueue.pop();
    dist = rho_ * path.length();
    std::vector<geometry_msgs::Point32> single;
    geometry_msgs::Point32 pt;
    std::vector<geometry_msgs::Polygon> polygons;
    for (double seg = 0.0; seg <= dist; seg += step_size) {
      interpolate(q0, path, seg / rho_, qnew);
      temp.x = qnew[0];
      temp.y = qnew[1];
      temp.z = qnew[2];
      single.push_back(temp);
      geometry_msgs::Polygon poly;
      float vyaw = qnew[2] + M_PI / 2.0;
      pt.x = qnew[0] + front_lon * cos(qnew[2]) - half_lat * cos(vyaw);
      pt.y = qnew[1] + front_lon * sin(qnew[2]) - half_lat * sin(vyaw);
      poly.points.push_back(pt);
      pt.x = qnew[0] + front_lon * cos(qnew[2]) + half_lat * cos(vyaw);
      pt.y = qnew[1] + front_lon * sin(qnew[2]) + half_lat * sin(vyaw);
      poly.points.push_back(pt);
      pt.x = qnew[0] - back_lon * cos(qnew[2]) + half_lat * cos(vyaw);
      pt.y = qnew[1] - back_lon * sin(qnew[2]) + half_lat * sin(vyaw);
      poly.points.push_back(pt);
      pt.x = qnew[0] - back_lon * cos(qnew[2]) - half_lat * cos(vyaw);
      pt.y = qnew[1] - back_lon * sin(qnew[2]) - half_lat * sin(vyaw);
      poly.points.push_back(pt);
      polygons.push_back(poly);
    }
    // The first path is the optimized shortest interpolated path!!!
    data.push_back(single);
    bounding_polygon.push_back(polygons);
  }
  return patharray;
}

void ReedsSheppStateSpace::interpolate(double q0[3], ReedsSheppPath &path,
                                       double seg, double s[3]) {
  if (seg < 0.0) seg = 0.0;
  if (seg > path.length()) seg = path.length();

  double phi, v;

  s[0] = s[1] = 0.0;
  s[2] = q0[2];

  for (unsigned int i = 0; i < 5 && seg > 0; ++i) {
    if (path.length_[i] < 0) {
      v = std::max(-seg, path.length_[i]);
      seg += v;
    } else {
      v = std::min(seg, path.length_[i]);
      seg -= v;
    }
    phi = s[2];
    switch (path.type_[i]) {
    case RS_LEFT:
      s[0] += (sin(phi + v) - sin(phi));
      s[1] += (-cos(phi + v) + cos(phi));
      s[2] = phi + v;
      break;
    case RS_RIGHT:
      s[0] += (-sin(phi - v) + sin(phi));
      s[1] += (cos(phi - v) - cos(phi));
      s[2] = phi - v;
      break;
    case RS_STRAIGHT:
      s[0] += (v * cos(phi));
      s[1] += (v * sin(phi));
      break;
    case RS_NOP:
      break;
    }
  }

  s[0] = s[0] * rho_ + q0[0];
  s[1] = s[1] * rho_ + q0[1];
}

}//namespce rs
} //namespace toolbox