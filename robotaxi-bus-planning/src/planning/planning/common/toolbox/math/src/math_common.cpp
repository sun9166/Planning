
#include "common/toolbox/math/include/math_common.h"
#include "common/toolbox/optimizer/include/meanfilt.h"
#include "common/toolbox/geometry/include/dubins.h"
#include <iostream>

#include <cstdlib>

namespace toolbox {
namespace math    {

using geometry::Site;
using geometry::SiteVec;

double Math::rand (const double min, const double max)      const {
  // struct timespec tn;
  // clock_gettime(CLOCK_REALTIME, &tn);
  // std::srand((unsigned)tn.tv_nsec);
  double tmp = std::rand() / double(RAND_MAX);
  return (min + (max - min) * tmp);
}

Site   Math::rand (const Site &center, const double radius) const {
  double ac      = rand_ac();
  double length  = rand(0.0, radius);
  double delta_x = length * std::cos(ac);
  double delta_y = length * std::sin(ac);

  return Site(center.x + delta_x, center.y + delta_y);
}

Site Math::Circle (const Site p1, const Site p2, const Site p3, double &radius) {
  double x1 = p1.x;
  double x2 = p2.x;
  double x3 = p3.x;
  double y1 = p1.y;
  double y2 = p2.y;
  double y3 = p3.y;

  double a = x1 - x2;
  double b = y1 - y2;
  double c = x1 - x3;
  double d = y1 - y3;
  double e = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0;
  double f = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0;
  double det = b*c - a*d;

  if (std::fabs(det) < 1e-5) {
    radius = -1;
    return Site(0,0);
  }

  double x0 = -(d*e - b*f) / det;
  double y0 = -(a*f - c*e) / det;
  radius = std::hypot(x1 - x0, y1 - y0);
  return Site(x0, y0);
}


void Math::GetArc(SiteVec &list, Site &center, double &radius, Site &base, double length) {
  list.clear();

  bool clockwise = true;

  // if the tail list is a straight line
  if (radius < 0) {
    Site target(base + base.Direction() * length);
    list.push_back(base);
    list.push_back(target);
    Interpolate(list);
    return;
  }

  if (std::fabs(base.angle) > 45 && std::fabs(base.angle) < 135) {
    if (base.x < center.x) {
      if (base.angle > 0) {
        clockwise = true;
      } else {
        clockwise = false;
      }
    } else {
      if (base.angle < 0) {
        clockwise = true;
      } else {
        clockwise = false;
      }
    }
  } else {
    if (base.y > center.y) {
      if (std::fabs(base.angle) < 90) {
        clockwise = true;
      } else {
        clockwise = false;
      }
    } else {
      if (std::fabs(base.angle) < 90) {
        clockwise = false;
      } else {
        clockwise = true;
      }
    }
  }

  // if (clockwise) {
  //   std::cout << "clockwise" << std::endl;
  // } else {
  //   std::cout << "unclockwise" << std::endl;
  // }

  if(radius < 1.0) radius = 1.0;
  double base_angle = std::atan2((base.y - center.y), (base.x - center.x));
  double delta_alpha = 0.05 / radius;
  size_t count = length / 0.05;
  // std::cout << "base = " << base.x << "," << base.y << std::endl;
  // std::cout << "center = " << center.x << "," << center.y << std::endl;
  // std::cout << "base_angle = " << base_angle << std::endl;
  // std::cout << "delta_alpha = " << delta_alpha << std::endl;
  // std::cout << "count = " << count << std::endl;

  // std::cout << "==================" << std::endl;

  for (size_t i=1; i<count; ++i) {
    Site new_site;
    if (clockwise) {
      new_site = Site(center.x + radius * std::cos(base_angle - i * delta_alpha),
                      center.y + radius * std::sin(base_angle - i * delta_alpha));
    } else {
      new_site = Site(center.x + radius * std::cos(base_angle + i * delta_alpha),
                      center.y + radius * std::sin(base_angle + i * delta_alpha));
    }
    list.push_back(new_site);
    // std::cout << "new_site = " << new_site.x << "," << new_site.y << std::endl;
  }

  for (size_t i=0; i<list.size() - 2; ++i) list[i].angle = (list[i+1] - list[i]).inerangle();
  list.back().angle = list[list.size() - 2].angle;
}


int Math::Interpolate(SiteVec& list, const double dens) {
  if (list.size() < 1) return -1;

  SiteVec buffer;
  for (auto it = list.begin(); it != list.end() - 1; ++it) {
    buffer.push_back(*it);

    Site current_point(*it);
    Site next_point(*(it + 1));
    SiteVec insert_list;
    insert_list.clear();
    double tmp_dis = (next_point - current_point).mold();
    int insert_num = tmp_dis / dens;
    Site delta;
    if (insert_num < 1) {
      continue;
    } else {
      delta = (next_point - current_point) / insert_num;
    }

    for (int i = 1; i < insert_num; ++i) {
      insert_list.push_back((current_point + delta * i));
    }

    buffer.insert(buffer.end(), insert_list.begin(), insert_list.end());
  }
  buffer.push_back(list.back());
  list.swap(buffer);
  
  return 0;
}

//默认list中携带角度
int Math::InterpolateWithAngle(SiteVec& list, const double dens) {
  if (list.size() < 1) return -1;

  SiteVec buffer;
  for (auto it = list.begin(); it != list.end() - 1; ++it) {
    buffer.push_back(*it);

    Site current_point(*it);
    Site next_point(*(it + 1));
    SiteVec insert_list;
    insert_list.clear();
    double tmp_dis = (next_point - current_point).mold();
    int insert_num = tmp_dis / dens;
    Site delta;
    if (insert_num < 1) {
      continue;
    } else {
      delta = (next_point - current_point) / insert_num;
    }

    for (int i = 1; i < insert_num; ++i) {
      Site insert_pt = current_point + delta * i;
      insert_pt.angle = current_point.angle;
      insert_pt.curvature = current_point.curvature;
      insert_pt.reverse = current_point.reverse;
      insert_pt.type = current_point.type;
      insert_pt.property = current_point.property;
      insert_list.push_back(insert_pt);
    }

    buffer.insert(buffer.end(), insert_list.begin(), insert_list.end());
  }
  buffer.push_back(list.back());
  list.swap(buffer);
  
  return 0;
}


} // math
} // toolbox

