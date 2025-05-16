/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * NodeName: common
 * FileName: line.h
 *
 * Description: some line operation
 *
 * History:
 * Zhang zhuo          2019/03/14    v1.0.0        Build this model.
 * Zhang zhuo          2019/06/11    v1.0.1        update. 
 ******************************************************************************/
#ifndef __COMMON_GEOMETRY_LINE_BASE_H__
#define __COMMON_GEOMETRY_LINE_BASE_H__

#include <cmath>
#include "site.h"
#include <Polygon.h>
#include <Point32.h>
#include <tuple>
#include <set>
#include <list>      //huangchuo  20250331
// #include "common/toolbox/geometry/include/geoheader.h"
// #include "common/log/include/log.h"

namespace geometry {

struct LineBase {
  LineBase() {
    p1 = Site();
    p2 = Site();
  }

  LineBase(const Site &pp1, const Site &pp2) {
    p1 = pp1;
    p2 = pp2;
  }

  ~LineBase() = default;

  Site p1;
  Site p2;

  void operator=(const LineBase &line) {
    p1 = line.p1;
    p2 = line.p2;
  }

  // LineBase operator+(const LineBase &line) const {return LineBase(x+line.x, y+line.y);}
  // LineBase operator-(const LineBase &line) const {return LineBase(x-line.x, y-line.y);}
  // double operator*(const LineBase &line) const {return (x*line.x+y*line.y);}
  std::pair<double, double> lvector() const {
    return std::make_pair(p2.x - p1.x, p2.y - p1.y);
  }

  double linerangle() const {
    return std::atan2(p2.y - p1.y, p2.x - p1.x) * 180 / M_PI; //-180~180
  }

  double lmold() const {
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
  }

  bool IsIntersect(const LineBase &l) {
    return (std::min(p1.x, p2.x) <= std::max(l.p1.x, l.p2.x)
         && std::max(p1.x, p2.x) >= std::min(l.p1.x, l.p2.x)
         && std::min(p1.y, p2.y) <= std::max(l.p1.y, l.p2.y)
         && std::max(p1.y, p2.y) >= std::min(l.p1.y, l.p2.y));
  }

  void Line2Path(std::vector<Site> &path, const double density) const { //modify by zhanghzuo [0611]
    path.clear();
    double density_tp = density;
    if (density_tp < 0.05) {
      density_tp = 0.05;
    }
    if (std::hypot(p2.x-p1.x, p2.y-p1.y) < density_tp+1e-3) {
      path.push_back(p1);
      path.push_back(p2);
      return;
    }
    int point_num = ceil(std::hypot(p2.x-p1.x, p2.y-p1.y) / density_tp);
    double l_agnle = std::atan2(p2.y-p1.y, p2.x-p1.x) * 180 / M_PI; //-180~180
    double deta_x = (p2.x - p1.x) / point_num;
    double deta_y = (p2.y - p1.y) / point_num;
    Site temp_p;
    for (int i=0; i<=point_num; ++i) {
      temp_p.x     = p1.x + i * deta_x;
      temp_p.y     = p1.y + i * deta_y;
      temp_p.xg    = temp_p.x;
      temp_p.yg    = temp_p.y;
      temp_p.angle = l_agnle;
      path.push_back(temp_p);
    }
    path.push_back(p2);
  }

  double Point2Line(const Site &p) const {
    if(p1 == p2) return __DBL_MAX__;
    double dis = std::fabs((p1.x - p.x)*(p2.y - p.y)-
                           (p2.x - p.x)*(p1.y - p.y)) / 
                 std::hypot(p2.x - p1.x, p2.y - p1.y);
    return dis;
  }
  int JudgeRelativePos2Seg(Site site_to_check) const{
  if(p1 == p2) return -1;
    double judge = ((site_to_check.x-p1.x)*(p2.x-p1.x)+
      (site_to_check.y-p1.y)*(p2.y-p1.y)) /
      (std::pow((p2.x-p1.x),2)+std::pow((p2.y-p1.y),2));
    return judge>=1.0-1e-3? 2 : (judge<=0+1e-3? 0 : 1);
  }
};


template <class T>
static bool CheckPointInPolygon(const T & pt,
                                const geometry_msgs::Polygon &bounding_polygon) {
  if (bounding_polygon.points.empty()) return false;

  int counter = 0;
  int i;
  double xinters;
  geometry_msgs::Point32 p1;
  geometry_msgs::Point32 p2;
  int N = bounding_polygon.points.size();
  p1 = bounding_polygon.points.at(0);
  for (i = 1; i <= N; i++) {
    p2 = bounding_polygon.points.at(i % N);
    if (pt.y > std::min<float>(p1.y, p2.y)) {
      if (pt.y <= std::max<float>(p1.y, p2.y)) {
        if (pt.x <= std::max<float>(p1.x, p2.x)) {
          if (p1.y != p2.y) {
            xinters = (pt.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
            if (p1.x == p2.x || pt.x <= xinters) counter++;
          }
        }
      }
    }
    p1 = p2;
  }
  if (counter % 2 == 0)
    return false;
  else
    return true;
}


static int GetNearestDis(std::list<Site> &gccs_list, const Site &target_pos, double &mindis) {
  int minindex = 0;
  mindis = std::numeric_limits<float>::max();

  if (gccs_list.size() == 0) {
    mindis = 0;
    return -1;
  }

  auto it = gccs_list.begin();
  int count = 0;
  for (; it != gccs_list.end(); ++it ) {
    float dis = std::hypot((*it).xg - target_pos.xg,
                           (*it).yg - target_pos.yg);

    // AINFO << "GetNearestDis " << (*it).xg << "," << (*it).yg << "," << target_pos.xg << "," << target_pos.yg;
    if (mindis > dis) {
      minindex = count;
      mindis = dis;
    }
    count++;
  }

  return minindex;
}


static std::tuple<geometry_msgs::Point32, geometry_msgs::Point32> carmodel(
  geometry_msgs::Point32 ego, geometry_msgs::Polygon &polygon,
  float headCarLength, float tailCarLength, float halfWheelTrack) {
  polygon.points.clear();
  geometry_msgs::Point32 pt, ld, ru;
  std::set<float> cmpx, cmpy;
  float vyaw = ego.z + M_PI / 2.0;
  pt.x = ego.x + headCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
  pt.y = ego.y + headCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x + headCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
  pt.y = ego.y + headCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
  pt.y = ego.y - tailCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
  pt.y = ego.y - tailCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  ld.x = *(cmpx.begin());
  ld.y = *(cmpy.begin());
  ru.x = *(cmpx.rbegin());
  ru.y = *(cmpy.rbegin());
  auto rtn = std::forward_as_tuple(ld, ru);
  return rtn;
}

static std::tuple<geometry_msgs::Point32, geometry_msgs::Point32> carmodel_right(
      geometry_msgs::Point32 ego, geometry_msgs::Polygon &polygon,
      float headCarLength, float tailCarLength, float halfWheelTrack) 
{
  polygon.points.clear();
  geometry_msgs::Point32 pt, ld, ru;
  std::set<float> cmpx, cmpy;
  float vyaw = ego.z + M_PI / 2.0;
  pt.x = ego.x + headCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
  pt.y = ego.y + headCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x + headCarLength * cos(ego.z);
  pt.y = ego.y + headCarLength * sin(ego.z);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z);
  pt.y = ego.y - tailCarLength * sin(ego.z);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z) - halfWheelTrack * cos(vyaw);
  pt.y = ego.y - tailCarLength * sin(ego.z) - halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  ld.x = *(cmpx.begin());
  ld.y = *(cmpy.begin());
  ru.x = *(cmpx.rbegin());
  ru.y = *(cmpy.rbegin());
  auto rtn = std::forward_as_tuple(ld, ru);
  return rtn;
}

static std::tuple<geometry_msgs::Point32, geometry_msgs::Point32> carmodel_left(
      geometry_msgs::Point32 ego, geometry_msgs::Polygon &polygon,
      float headCarLength, float tailCarLength, float halfWheelTrack) 
{
  polygon.points.clear();
  geometry_msgs::Point32 pt, ld, ru;
  std::set<float> cmpx, cmpy;
  float vyaw = ego.z + M_PI / 2.0;
  pt.x = ego.x + headCarLength * cos(ego.z);
  pt.y = ego.y + headCarLength * sin(ego.z);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x + headCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
  pt.y = ego.y + headCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z) + halfWheelTrack * cos(vyaw);
  pt.y = ego.y - tailCarLength * sin(ego.z) + halfWheelTrack * sin(vyaw);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  pt.x = ego.x - tailCarLength * cos(ego.z);
  pt.y = ego.y - tailCarLength * sin(ego.z);
  // std::cout << "carmode::" << pt.x << "," << pt.y << std::endl;
  cmpx.insert(pt.x);
  cmpy.insert(pt.y);
  polygon.points.push_back(pt);
  ld.x = *(cmpx.begin());
  ld.y = *(cmpy.begin());
  ru.x = *(cmpx.rbegin());
  ru.y = *(cmpy.rbegin());
  auto rtn = std::forward_as_tuple(ld, ru);
  return rtn;
}

} // namespace geometry
#endif  // __COMMON_GEOMETRY_LINE_BASE_H__