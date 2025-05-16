#include "base_func.h"
 
namespace acu{ 
namespace planning {

double NormalizeAngle(const double angle) {
  const double new_angle = std::fmod(angle + 180.0, 180.0 * 2.0);
  return (new_angle < 0 ? new_angle + 180.0 * 2.0 : new_angle) - 180.0;
}

double IncludeAngle(double angle1, double angle2) {
  if (angle1 > angle2 + 180) {
    angle2 = angle2 + 360;
  } else if (angle2 > angle1 + 180) {
    angle1 = angle1 + 360;
  }
  double anglerr = angle1 - angle2;
  return anglerr;
}

string LaneToRoad(string lane) {
  string road = "";
  for (auto &it : lane) {
    if (it == '_') break;
    road.push_back(it);
  }
  return road;
}

double PointsSimLine(vector<pair<double, double> > &points) {
  int num = points.size();
  double k = 0.0;
  double sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_xy = 0.0;
  for (int i = 0; i < num; i++) {
    sum_x += points[i].first;
    sum_y += points[i].second;
    sum_x2 += (points[i].first * points[i].first);
    sum_xy += (points[i].first * points[i].second);
  }
  double delta = num * sum_x2 - sum_x * sum_x;

  if (delta < 1e-3) {
    k = 1000;
  }
  else {
    k = (num * sum_xy - sum_x * sum_y)/(delta);
  }
  return k;
}

double PointsSimLine(vector<double> &list_t, vector<double> &list_s, 
                     double &k, double &b, int start, int end) {
  int num = min((int)list_t.size(), end - start);
  k = 0.0, b = 0.0;
  double sum_x = 0.0, sum_y = 0.0, sum_x2 = 0.0, sum_xy = 0.0;
  for (int i = max(0, start); i < min((int)list_t.size(), end); i++) {
    sum_x += (double)list_t[i];
    sum_y += (double)list_s[i];
    sum_x2 += (double)(list_t[i] * list_t[i]);
    sum_xy += (double)(list_t[i] * list_s[i]);
  }
  double delta = num * sum_x2 - sum_x * sum_x;
  if (delta < 1e-3) {
    k = 1000;
  }
  else {
    k = (num * sum_xy - sum_x * sum_y)/(delta);
  }
  b = -k * sum_x / num  + sum_y / num;
  return 0.0;
}

int RoundN(const double &a, const double &b) {
  int aa = (int)(1000 * fabs(a));
  int bb = (int)(1000 * fabs(b));
  int c = 0;
  if (aa < bb) {
    c = (aa < 300)? 0 : 1;
  }
  else {
    int cc = aa % bb;
    c = aa / bb;
    c += (cc < 300)? 0 : 1;
  }
  c = (a > 0.0)? c : -c;
  return c;
}

void Point2Line(int x1, int y1, int x2, int y2, double &k, double &b) {
  if (x1 == x2) {
    k = 1000.0;
    b = y1 - 1000.0 * x1;
    return;
  }
  k = (double)(y2 -y1)/(double)(x2 - x1);
  b = y1 - k * x1;
}


void GetVerticalLine(const Site &change_point, double &a, double &b, double &c) {
  double k_angle = change_point.globalangle - 90.0;
  k_angle = (k_angle < -360.0)? (k_angle + 360.0) : k_angle;
  k_angle = (k_angle > 360.0)? (k_angle - 360.0) : k_angle;
  k_angle = k_angle * M_PI / 180.0;
  if (fabs(tan(k_angle)) < 10.0) {
    a = tan(k_angle);
  }
  else if (tan(k_angle) < -10.0) {
    a = -10.0;
  }
  else {
    a = 10.0;
  }
  b = -1.0;
  c = change_point.yg - a * change_point.xg;
}

Box2d CellsToBox(CallbackObject &per_obj) {
  Box2d cell_box;
  std::vector<Vec2d> polygon_points;
  for (auto &cell : per_obj.cells) {
    if (cell.xg < 1e3 || cell.yg < 1e3) continue;
    if (cell.xg > 1e7 || cell.yg > 1e7) continue;
    if (isnan(cell.xg)|| isnan(cell.yg)) continue;
    polygon_points.emplace_back(cell.xg, cell.yg);
  }
  if (polygon_points.size()) {
    Vec2d append_cell_point(polygon_points.back().x() + 0.01, polygon_points.back().y());
    polygon_points.emplace_back(append_cell_point);
    append_cell_point = Vec2d(polygon_points.back().x(),polygon_points.back().y() + 0.01);
    polygon_points.emplace_back(append_cell_point);
  }
  Polygon2d obj_polygon;
  double cell_width = 0.0;
  double cell_length = 0.0;
  if (Polygon2d::ComputeConvexHull(polygon_points, &obj_polygon) &&
    obj_polygon.num_points() > 0) {
    cell_box = obj_polygon.BoundingBoxWithHeading(per_obj.global_angle * M_PI / 180.0);
  }
  return cell_box;
}

Box2d GetBox(CallbackObject &per_obj) {
  double length = per_obj.length;
  double width = per_obj.width;
  Box2d obj_box(Vec2d(per_obj.xabs, per_obj.yabs), 
                per_obj.global_angle * M_PI / 180.0, length, width);
  return obj_box;
}

void OverlayObjProbability(vector<ST> &old_points, vector<ST> &new_points, 
                           int &update_s, int &update_t, int expand_state) {
  for (int i = 0; i < old_points.size(); i++) {
    old_points.at(i).s -= update_s;
    old_points.at(i).t -= update_t;
    if (old_points.at(i).s < 0 || old_points.at(i).t < 0) {
      old_points.erase(old_points.begin() + i);
      i--;
    }
  }
  int min_s = 1000;
  int max_s = 0;
  int min_t = 1000;
  int max_t = 0;
  for (int i = 0; i < old_points.size(); i++) {
    min_s = min(min_s, old_points.at(i).s);
    max_s = max(max_s, old_points.at(i).s);
    min_t = min(min_t, old_points.at(i).t);
    max_t = max(max_t, old_points.at(i).t);
  }
  for (int i = 0; i < new_points.size(); i++) {
    min_s = min(min_s, new_points.at(i).s);
    max_s = max(max_s, new_points.at(i).s);
    min_t = min(min_t, new_points.at(i).t);
    max_t = max(max_t, new_points.at(i).t);
  }
  if (max_s <= min_s || max_t <= min_t) return;
  vector<vector<ST>> temp_points;
  temp_points.reserve(max_t - min_t + 1);
  for (int t = min_t; t <= max_t; t++) {
    vector<ST> t_points;
    t_points.reserve(max_s - min_s + 1);
    for (int s = min_s; s <= max_s; s++) {
      ST point(s, t);
      t_points.push_back(point);
    }
    temp_points.push_back(t_points);
  }

  for (auto &new_point : new_points) {
    if (expand_state == 0) {
      new_point.p *= FLAGS_p_o;
    } else if (expand_state == 1) {
      new_point.p *= FLAGS_p_threshold;
    }
    new_point.p = fmax(new_point.p, FLAGS_p_thd_delete+ 0.01);
    temp_points.at(new_point.t-min_t).at(new_point.s-min_s) = new_point;
  }
  if (expand_state == 2) {
    old_points.clear();// 强制挤压不要历史
  }
  for (auto &old_point : old_points) {
    auto &reference_point = temp_points.at(old_point.t-min_t).at(old_point.s-min_s);
    if (reference_point.p < FLAGS_p_thd_delete) {// 没有新点占据
      if (expand_state == 1) {// 正常挤压，强制衰减
        reference_point.p = fmin(0.6 * old_point.p, FLAGS_p_thd_delete);
      } else {
        double L = (1.0 - FLAGS_p_e)/FLAGS_p_e;
        L *= old_point.p/(1.01 - old_point.p);
        old_point.p = L / (1.0 + L);
        if (old_point.p >= FLAGS_p_thd_delete) {
          reference_point = old_point;
        }
      }
    } else {// 已经被新点占据
      if (expand_state == 1) {// 正常挤压，离散重合点权重降低
        old_point.p = fmax(old_point.p * 0.6, 0.5);// 5帧消亡
      }
      old_point.p = fmin(old_point.p, 0.999);// 5帧消亡
      old_point.p = fmax(old_point.p, FLAGS_p_thd_delete);
      if (old_point.p < 0.5) { 
        reference_point.p = fmax(old_point.p, reference_point.p);
        reference_point.p = fmax(FLAGS_p_thd_delete+0.01, reference_point.p);
      } else {
        double L = reference_point.p/(1.01 - reference_point.p);
        L *= old_point.p/(1.01 - old_point.p);
        reference_point.p = L / (1.0 + L);
      }
    }
  }
  old_points.clear();
  old_points.reserve((max_t - min_t + 1) * (max_s - min_s + 1));
  for (auto &t_points : temp_points) {
    for (auto &point : t_points) {
      if (point.p >= FLAGS_p_thd_delete) {
        old_points.push_back(point);
      }
    }
  }
}


double VT(double& init_v, double &t, double a) {
  double limit_a = 0.5, max_v;
  // 自车起步时候加速度给小一点
  if (init_v < 2.0) {
    if (t < (2.0 - init_v) / limit_a) {
      max_v = init_v + limit_a * t;
    } else {
      max_v = 2.0 + (t - (2.0 - init_v) / limit_a) * a;
    }
  } else {
    max_v = init_v + t * a;
  }
  
  return max_v;
}




} // namespace planning
} // namespace acu
