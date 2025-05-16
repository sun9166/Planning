#ifndef DATAPOOL_INCLUDE_PREDICTION_TYPEDEF_H_
#define DATAPOOL_INCLUDE_PREDICTION_TYPEDEF_H_

#include "public_typedef.h"
// #include "datapool/include/data_pool.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"
#include "common/toolbox/geometry/include/geoheader.h"

using namespace acu::common;
using namespace acu::common::math;
using namespace std;
using geometry::Site;

namespace acu {
namespace planning {

struct ST {
  double p;
  // vector<double> a;// 占据此点的加速度
  vector<pair<double, double>> a_p;
  int s;
  int t;
  ST() {
    p = 0.0;
    s = 0;
    t = 0;
    a_p.clear();
  }
  ST(double in_s, double in_t, double in_p = 0.0) {
    s = in_s;// 0.5m倍数
    t = in_t;// 0.1s倍数
    p = in_p;
  }
  // double average_a() {
  //   double num = 0.0;
  //   if (a.empty()) return num;
  //   for (auto &ai : a) {
  //     num += ai;
  //   }
  //   return num / (double)(a.size());
  // }
};

struct PredictionPoint {
  double x; 
  double y; 
  double angle; // degree
  double xg; 
  double yg; 
  double globalangle; // degree
  double v; 
  double a; 
  double t;
  string lane_id;
  PredictionPoint() {
    Reset();
  }
  void Reset() {
    x = 0.0; 
    y = 0.0; 
    angle = 0.0;
    xg = 0.0; 
    yg = 0.0; 
    globalangle = 0.0;
    v = 0.0; 
    a = 0.0;
    t = 0.0;
    lane_id = "";
  }
  void ToSite(Site &out_p) {
    out_p.Reset();
    out_p.x = x;
    out_p.y = y;
    out_p.angle = angle;
    out_p.xg = xg; 
    out_p.yg = yg; 
    out_p.globalangle = globalangle;
    out_p.velocity = v;
  }
};

struct LocalSTArea {
  double p;
  double a;
  vector<pair<Vec2d, Vec2d> > st_boundary;
  LocalSTArea() {
    Reset();
  }
  void Reset() {
    p = 0.0;
    a = 0.0;
    st_boundary.clear();
  }
};

struct PredictionTrajectory {
  double probability;
  string intentbylane;
  vector<PredictionPoint> points;
  vector<string> lane_ids;
  vector<ST> st_points;
  vector<LocalSTArea> londiscrete_areas;
  vector<LocalSTArea> latdiscrete_areas;
  vector<pair<acu::common::math::Vec2d, 
              acu::common::math::Vec2d>> st_boundary;
  pair<double, double> range_a;
  pair<double, double> range_s;
  vector<double> range_pd_s;
  PredictionTrajectory() {
    Reset();
  }
  void Reset() {
    probability = 0.0;
    intentbylane = "";
    points.clear();
    lane_ids.clear();
    st_points.clear();
    londiscrete_areas.clear();
    latdiscrete_areas.clear();
    st_boundary.clear();
    range_pd_s.clear();
    range_a.first = 0.0;
    range_a.second = 0.0;
    range_s.first = 0.0;
    range_s.second = 0.0;
  }

  PredictionPoint GetPredictionPointAtTime(const double time) const {
    PredictionPoint rtvalue;
    if(points.empty()){
      return rtvalue;
    }
    if(points.size() < 2) {
      return *points.rbegin();;
    }else{
      auto comp = [](const PredictionPoint p, const double time) {
        return p.t < time;
      };

      auto it_lower =
          std::lower_bound(points.begin(), points.end(), time, comp);

      if (it_lower == points.begin()) {
        return *points.begin();
      } else if (it_lower == points.end()) {
        return *points.rbegin();
      }
      //依赖于points的密度，是否需要插值
      // rtvalue = math::InterpolateUsingLinearApproximation(
      //           *(it_lower - 1), *it_lower, time);
      rtvalue = *it_lower;
      return rtvalue;
    }
  }

};

struct PredictionObject {
  double time_stamp;
  int id;
  string type;
  int priority;
  double predicted_period;
  double speed;
  bool is_static;
  bool is_ultra_static;
  vector<PredictionTrajectory> trajectories;
  string obj_lane_id;

  PredictionObject () {
    Reset();
  }
  void Reset() {
    time_stamp = 0.0;
    id = 0;
    type = "UNKNOWN Type";
    priority = 3;
    predicted_period = 8.0;
    speed = 0.0;
    is_static = false;
    is_ultra_static = false;
    trajectories.clear();
    obj_lane_id = "";
  }
};

struct PredictionData {
  double time;
  double start_timestamp;
  double end_timestamp;
  double perception_time;
  vector<PredictionObject> prediction_objects;
  FaultStatus prediction_status;
  PredictionData () {
    Reset();
  }
  void Reset() {
    time = 0.0;
    start_timestamp = 0.0;
    end_timestamp = 0.0;
    perception_time = 0.0;
    prediction_objects.clear();
  }
};

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_PREDICTION_TYPEDEF_H_
