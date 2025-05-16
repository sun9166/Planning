
#ifndef COMMON_TOOLBOX_GEOMETRY_INCLUDE_SITE_BASE_H__
#define COMMON_TOOLBOX_GEOMETRY_INCLUDE_SITE_BASE_H__

#include <cmath>
#include <functional>
#include <limits>
#include <string>

namespace geometry {

typedef struct Restricts {
  float restrict_left;
  float restrict_right;
  Restricts() {
    restrict_left = std::numeric_limits<float>::max();
    restrict_right = std::numeric_limits<float>::max();
  }
  void operator=(const Restricts &r) {
    restrict_left = r.restrict_left;
    restrict_right = r.restrict_right;
  }
} Restricts;

struct Site {
  Site() {
    x = 0;
    y = 0;
    angle = 0;
    xg = 0;
    yg = 0;
    globalangle = 0;
    dr_x = 0;
    dr_y = 0;
    dr_angle = 0; //in degree
    velocity = 0;
    index = -999;
    origin_index = -999;
    length = 0.0;
    a = 0;
    t = 0;
    curvature = 999.0;
    speed_limit = -88.0;
    reverse = false; //false : front true: back
    type = "W";
    property = 0; // 1: global_path 2: clean_path 3: curb_path 4:connection_path
    offset_property = 0;// @pqg add 0/2
    boundary_id = "";
    pathpoint_restrict.restrict_left = std::numeric_limits<float>::max();
    pathpoint_restrict.restrict_right = std::numeric_limits<float>::max();
    direction = 0;
  }
  Site(const Site &s) {
    x = s.x;
    y = s.y;
    angle = s.angle;
    xg = s.xg;
    yg = s.yg;
    globalangle = s.globalangle;
    dr_x = s.dr_x;
    dr_y = s.dr_y;
    dr_angle = s.dr_angle;
    velocity = s.velocity;
    index = s.index;
    origin_index = s.origin_index;
    length = s.length;
    a = s.a;
    t = s.t;
    curvature = s.curvature;
    speed_limit = s.speed_limit;
    reverse = s.reverse;
    type = s.type;
    property = s.property;
    offset_property = s.offset_property;
    boundary_id = s.boundary_id;
    pathpoint_restrict = s.pathpoint_restrict;
  }
  Site(double xx, double yy) {
    x = xx;
    y = yy;
    angle = 0;
    xg = 0;
    yg = 0;
    globalangle = 0;
    dr_x = 0;
    dr_y = 0;
    dr_angle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    length = 0.0;
    a = 0;
    t = 0;
    curvature = 999.0;
    speed_limit = -88.0;
    reverse = false;
    type = "W";
    property = 0;
    offset_property = 0;
    boundary_id = "";
    pathpoint_restrict.restrict_left = std::numeric_limits<float>::max();
    pathpoint_restrict.restrict_right = std::numeric_limits<float>::max();
  }
  Site(double xx, double yy, double aa) {
    x = xx;
    y = yy;
    angle = aa;
    xg = 0;
    yg = 0;
    globalangle = 0;
    dr_x = 0;
    dr_y = 0;
    dr_angle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    length = 0.0;
    a = 0;
    t = 0;
    curvature = 999.0;
    speed_limit = -88.0;
    reverse = false;
    type = "W";
    property = 0;
    offset_property = 0;
    boundary_id = "";
    pathpoint_restrict.restrict_left = std::numeric_limits<float>::max();
    pathpoint_restrict.restrict_right = std::numeric_limits<float>::max();
  }
  //~Site() = default;

  double x;
  double y;
  double angle;
  double xg;
  double yg;
  double globalangle;
  double dr_x;
  double dr_y;
  double dr_angle;
  double velocity;
  int index;
  int origin_index;
  double length;
  double a;
  double t;
  double curvature;
  double speed_limit;
  bool reverse;
  std::string type;
  int property;
  int offset_property;
  std::string boundary_id;
  Restricts pathpoint_restrict;
  int direction;

  double DistanceTo(const Site &s) const{
    return hypot(s.xg - xg, s.yg - yg);
  }

  void operator=(const Site &s) {
    x = s.x;
    y = s.y;
    angle = s.angle;
    xg = s.xg;
    yg = s.yg;
    globalangle = s.globalangle;
    dr_x = s.dr_x;
    dr_y = s.dr_y;
    dr_angle = s.dr_angle;
    velocity = s.velocity;
    index = s.index;
    origin_index = s.origin_index;
    length = s.length;
    a = s.a;
    t = s.t;
    curvature = s.curvature;
    speed_limit = s.speed_limit;
    reverse = s.reverse;
    type = s.type;
    property = s.property;
    offset_property = s.offset_property;
    boundary_id = s.boundary_id;
    pathpoint_restrict = s.pathpoint_restrict;
  }
  Site &operator+=(const Site &s) {
    x += s.x;
    y += s.y;
    xg += s.xg;
    yg += s.yg;
    dr_x += s.dr_x;
    dr_y += s.dr_y;
    return *this;
  }
  Site &operator-=(const Site &s) {
    x -= s.x;
    y -= s.y;
    xg -= s.xg;
    yg -= s.yg;
    dr_x -= s.dr_x;
    dr_y -= s.dr_y;
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

  void Reset() {
    x = 0;
    y = 0;
    angle = 0;
    xg = 0;
    yg = 0;
    globalangle = 0;
  	dr_x = 0;
  	dr_y = 0;
    dr_angle = 0;
    velocity = 0;
    index = -999;
    origin_index = -999;
    length = 0.0;
    a = 0;
    t = 0;
    curvature = 999.0;
    speed_limit = -88.0;
    reverse = false; //false : front true: back
    type = "W";
    property = 0; // 1: global_path 2: clean_path 3: curb_path 4:connection_path
    offset_property = 0;
    boundary_id = "";
    pathpoint_restrict.restrict_left = std::numeric_limits<float>::max();
    pathpoint_restrict.restrict_right = std::numeric_limits<float>::max();
  }

  double inerangle() const { return std::atan2(y, x) * 180 / M_PI; }
  double mold() const { return std::hypot(x, y); }
  Site Direction() const { return Site(x / this->mold(), y / this->mold()); }

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

struct StructTtajectoryPoint : public Site {
  
};

}  // namespace geometry

#endif  // __COMMON_GEOMETRY_SITE_BASE_H__
