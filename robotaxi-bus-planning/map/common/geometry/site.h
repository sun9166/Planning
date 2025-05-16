/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * NodeName: common
 * FileName: sitebase.h
 *
 * Description: declare 2-D points with (x, y), and some common methods
 *
 * History:
 * Feng younan          2018/08/20    v1.0.0        Build this model.
 ******************************************************************************/
#ifndef __COMMON_GEOMETRY_SITE_BASE_H__
#define __COMMON_GEOMETRY_SITE_BASE_H__

#include <cmath>
#include <functional>
#include <limits>
#include <string>

namespace geometry {

typedef struct Restricts
{
  float restrict_left;
  float restrict_right;
  Restricts()
  {
    restrict_left = std::numeric_limits<float>::max();
    restrict_right = std::numeric_limits<float>::max();
  }
  void operator=(const Restricts &r) 
  {
    restrict_left = r.restrict_left;
    restrict_right = r.restrict_right;
  }
}Restricts;

struct Site {
  Site() {
    x = 0;
    y = 0;
    angle = 0;
    xg = 0;
    yg = 0;
    globalangle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    a = 0;
    t = 0;
    curvature = -1.0;
    length = 0.0;
    reverse = false; //false : front true: back
    type = "W";
    property = 0; // 1: global_path 2: clean_path 3: curb_path 4:connection_path
    boundary_id = "";
  }
  Site(const Site &s) {
    x = s.x;
    y = s.y;
    angle = s.angle;
    xg = s.xg;
    yg = s.yg;
    globalangle = s.globalangle;
    velocity = s.velocity;
    index = s.index;
    origin_index = s.origin_index;
    a = s.a;
    t = s.t;
    curvature = s.curvature;
    length = s.length;
    reverse = s.reverse;
    type = s.type;
    property = s.property;
    boundary_id = s.boundary_id;
  }
  Site(double xx, double yy) {
    x = xx;
    y = yy;
    angle = 0;
    xg = 0;
    yg = 0;
    globalangle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    a = 0;
    t = 0;
    curvature = -1.0;
    length = 0.0;
    reverse = false;
    type = "W";
    property = 0;
    boundary_id = "";
  }
  Site(double xx, double yy, double aa) {
    x = xx;
    y = yy;
    angle = aa;
    xg = 0;
    yg = 0;
    globalangle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    a = 0;
    t = 0;
    curvature = -1.0;
    length = 0.0;
    reverse = false;
    type = "W";
    property = 0;
    boundary_id = "";
  }
  ~Site() = default;

  double x;
  double y;
  double angle;
  double xg;
  double yg;
  double globalangle;
  double velocity;
  int index;
  int origin_index;
  double a;
  double t;
  double curvature;
  double length;
  bool reverse;
  std::string type;
  int property;
  std::string boundary_id;

  void operator=(const Site &s) {
    x = s.x;
    y = s.y;
    angle = s.angle;
    xg = s.xg;
    yg = s.yg;
    globalangle = s.globalangle;
    velocity = s.velocity;
    index = s.index;
    origin_index = s.origin_index;
    a = s.a;
    t = s.t;
    curvature = s.curvature;
    length = s.length;
    reverse = s.reverse;
    type = s.type;
    property = s.property;
    boundary_id = s.boundary_id;
  }
  Site &operator+=(const Site &s) {
    x += s.x;
    y += s.y;
    xg += s.xg;
    yg += s.yg;
    return *this;
  }
  Site &operator-=(const Site &s) {
    x -= s.x;
    y -= s.y;
    xg -= s.xg;
    yg -= s.yg;
    return *this;
  }
  bool operator==(const Site &s) const {
    double precision = std::hypot((x - s.x), (y - s.y));
    return (precision < 1e-5) && (std::fabs(angle - s.angle) < 1e-5);
  }
  Site operator+(const Site &s) const { return Site(x + s.x, y + s.y); }
  Site operator-(const Site &s) const { return Site(x - s.x, y - s.y); }
  Site operator*(const double d) const { return Site(x * d, y * d); }
  Site operator/(const double d) const { return Site(x / d, y / d); }

  void set_g(double xx, double yy) {
    xg = xx;
    yg = yy;
  }

  void set_g(double xx, double yy, double aa) {
    xg = xx;
    yg = yy;
    angle = aa;
  }

  void set_global(double xx, double yy, double aa) {
    xg = xx;
    yg = yy;
    globalangle = aa;
  }

  void set_property(int temp_property) {
    property = temp_property;
  }

  double inerangle() const { return std::atan2(y, x) * 180 / M_PI; }
  double mold() const { return std::hypot(x, y); }
  Site direction() const { return Site(x / this->mold(), y / this->mold()); }

  struct compare {
    bool operator()(const Site &s1, const Site &s2) const {
      if (std::fabs(s1.x - s2.x) < 1e-5)
        return s1.y < s2.y;
      else
        return s1.x < s2.x;
    }
  };

  struct hash_equal {
    bool operator()(const Site &ls, const Site &rs) const {
      double precision = std::hypot((ls.x - rs.x), (ls.y - rs.y));
      return (precision < 1e-5);
    }
  };

  struct hash_key {
    std::size_t operator()(const Site &s) const {
      std::hash<double> hash_fun;
      return size_t(hash_fun(s.x + s.y));
    }
  };
};

}  // namespace geometry

#endif  // __COMMON_GEOMETRY_SITE_BASE_H__
