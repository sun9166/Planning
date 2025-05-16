#include "sl_boundary.h"

namespace acu {
namespace planning {

bool GetSLBoundary(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary& sl_bdy) {
	sl_bdy.Reset();
	if (temp_obj.id < 0) return ObjectToSL(map_info, temp_obj, sl_bdy);
	bool result = BoxToSL(map_info, temp_obj, sl_bdy);
	if (sl_bdy.max_l < -2.0 || sl_bdy.min_l > 2.0 || hypot(sl_bdy.min_l, sl_bdy.min_s) > 40.0 ||
      temp_obj.cells.empty() ) {
		return result;
	}
	return ObjectToSL(map_info, temp_obj, sl_bdy);
}

bool XYToSL(const MapEngineLineList &map_info, const Site &input_p, double &s, double &l) {
	s = 1000.0;
	l = 1000.0;
	int center_index = -1;
	if (!GetFrenetOriginIndex(map_info, input_p, center_index)) {
		return false;
	}
	center_index = max(2, center_index);
	int start_index = max(center_index - 100, 2);
	int end_index = min(center_index + 100, (int)map_info.path_points.size() - 1);
	double min_dis = numeric_limits<double>::max();
	int min_index = center_index;
	if (input_p.x > 0.0) {
		for (int i = start_index; i <= end_index; i++) {
			double temp_dis = hypot(input_p.xg - map_info.path_points[i].xg, 
						   			input_p.yg - map_info.path_points[i].yg);
			if (temp_dis < min_dis) {
				min_dis = temp_dis;
				min_index = i;
			}
		}
	}
	if (min_index < 2 || min_index > map_info.path_points.size()) {
		return false;
	}
	s = map_info.path_points[min_index].length;
	double delta = (map_info.path_points[min_index].xg - map_info.path_points[min_index-2].xg) * input_p.yg - 
			   	   (map_info.path_points[min_index].yg - map_info.path_points[min_index-2].yg) * input_p.xg - 
			   	    map_info.path_points[min_index].xg * map_info.path_points[min_index-2].yg +
			   	    map_info.path_points[min_index-2].xg * map_info.path_points[min_index].yg;
	l = delta / hypot(map_info.path_points[min_index].yg - map_info.path_points[min_index-2].yg,
					  map_info.path_points[min_index].xg - map_info.path_points[min_index-2].xg);
	double dis = hypot(input_p.xg - map_info.path_points[min_index].xg, 
					   input_p.yg - map_info.path_points[min_index].yg);
	//障碍物在最后一个路点外侧，且直线距离大于投影距离0.1m以上，更新s
	if ((min_index >= end_index) && dis > fabs(l) + 0.1) {
		s += sqrt(pow(dis,2) - pow(l,2));
	}	
	else if ((min_index <= start_index) && dis > fabs(l) + 0.1) {
		s -= sqrt(pow(dis,2) - pow(l,2));
	}	
	// AERROR_IF(FLAGS_log_enable)<<" min index "<<min_index<<" min l "<<l<<" dis "<<dis<<" s "<<s;
	return true;
}

bool XYToSL(const SiteVec &path_points, const Site &input_p, int center_index, double &s, double &l) {
	s = 1000.0;
	l = 1000.0;
	center_index = max(2, center_index);
	int start_index = max(center_index - 100, 2);
	int end_index = min(center_index + 100, (int)path_points.size() - 1);
	double min_dis = numeric_limits<double>::max();
	int min_index = center_index;
	for (int i = start_index; i <= end_index; i += 2) {
		double temp_dis = hypot(input_p.xg - path_points[i].xg, 
					   			input_p.yg - path_points[i].yg);
		if (temp_dis < min_dis) {
			min_dis = temp_dis;
			min_index = i;
		}
	}
	if (min_index < 2 || min_index > path_points.size()) {
		return false;
	}
	s = path_points[min_index].length;
	double delta = (path_points[min_index].xg - path_points[min_index-2].xg) * input_p.yg - 
			   	   (path_points[min_index].yg - path_points[min_index-2].yg) * input_p.xg - 
			   	    path_points[min_index].xg * path_points[min_index-2].yg +
			   	    path_points[min_index-2].xg * path_points[min_index].yg;
	l = delta / hypot(path_points[min_index].yg - path_points[min_index-2].yg,
					  path_points[min_index].xg - path_points[min_index-2].xg);
	double dis = hypot(input_p.xg - path_points[min_index].xg, 
					   input_p.yg - path_points[min_index].yg);
	//障碍物在最后一个路点外侧，且直线距离大于投影距离0.1m以上，更新s
	if ((min_index >= end_index - 4) && dis > fabs(l) + 0.1) {

		s += sqrt(pow(dis,2) - pow(l,2));
	}	
	else if ((min_index <= start_index) && dis > fabs(l) + 0.1) {
		s -= sqrt(pow(dis,2) - pow(l,2));
	}		
	return true;
}

bool BoxToSL(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary &sl_bdy) {
	sl_bdy.Reset();
	if (map_info.path_points.empty()) return false;
	Site obj_center;
	obj_center.xg = temp_obj.box.center_x();
	obj_center.yg = temp_obj.box.center_y();
	int center_index = -1;
	if (!GetFrenetOriginIndex(map_info, obj_center, center_index)) {
		AERROR_IF(FLAGS_log_enable)<<"GetFrenetOriginIndex failed.";
		return false;
	}
	vector<Vec2d> corners;
	temp_obj.box.GetAllCorners(&corners);
	double special_min_l = numeric_limits<double>::max();
	double special_max_l = numeric_limits<double>::lowest();
	for (auto &corner_p : corners) {
		double s, l;
		Site temp_p;
		temp_p.xg = corner_p.x();
		temp_p.yg = corner_p.y();
		if (!XYToSL(map_info.path_points, temp_p, center_index, s, l)) continue;
		if (s > -1.0 && l < special_min_l) {
			special_min_l = l;
		}
		if (s > -1.0 && l > special_max_l) {
			special_max_l = l;
		}
		sl_bdy.Set(s, l);
	}
	if (sl_bdy.min_s < -0.01 && sl_bdy.max_s > 0.01) {// 车侧并行障碍物
		sl_bdy.min_l = special_min_l;
		sl_bdy.max_l = special_max_l;
	}

	bool is_uturn = false;
	for (auto &lane_turn : map_info.lane_turns) {
		if (lane_turn.second != 4) continue;
		double nearest_s = (fabs(sl_bdy.min_s) < fabs(sl_bdy.max_s))? sl_bdy.min_s : sl_bdy.max_s;
		if (temp_obj.x < -5.0 && nearest_s > 0.0 && sl_bdy.min_l > 3.0) {
			sl_bdy.Reset();
			for (auto &corner_p : corners) {
				double s, l;
				Site temp_p;
				temp_p.xg = corner_p.x();
				temp_p.yg = corner_p.y();
				if (XYToSL(map_info.path_points, temp_p, 0, s, l)) {
					sl_bdy.Set(s, l);
				}
			}
		} 
		break;
	}
	sl_bdy.max_s = (sl_bdy.max_s <= sl_bdy.min_s)? (sl_bdy.min_s + 0.1) : sl_bdy.max_s;
	sl_bdy.max_l = (sl_bdy.max_l <= sl_bdy.min_l)? (sl_bdy.min_l + 0.1) : sl_bdy.max_l;
	return true;
}

bool BoxToSL(const MapEngineLineList &mapinfo, const Box2d &box, StructSLBoundary &sl_bdy) {
  sl_bdy.Reset();
  if (mapinfo.path_points.empty()) {
    AERROR<<"BoxToSL no reference path_points.";
    return false;
  }
  Site obj_center;
  obj_center.set_g(box.center_x(), box.center_y());
  int center_index = -1;
  if (!GetFrenetOriginIndex(mapinfo, obj_center, center_index)) {
    AERROR_IF(FLAGS_log_enable)<<"GetFrenetOriginIndex failed.";
    return false;
  }
  vector<Vec2d> corners;
  box.GetAllCorners(&corners);
  double special_min_l = numeric_limits<double>::max();
  double special_max_l = numeric_limits<double>::lowest();
  for (auto &corner_p : corners) {
    double s, l;
    Site temp_p;
    temp_p.xg = corner_p.x();
    temp_p.yg = corner_p.y();
    if (!XYToSL(mapinfo.path_points, temp_p, center_index, s, l)) {
      continue;
    }
    if (s > -1.0 && l < special_min_l) {
      special_min_l = l;
    }
    if (s > -1.0 && l > special_max_l) {
      special_max_l = l;
    }
    sl_bdy.Set(s, l);
  }
  if (sl_bdy.min_s < -0.01 && sl_bdy.max_s > 0.01) {// 车侧并行障碍物
    sl_bdy.min_l = special_min_l;
    sl_bdy.max_l = special_max_l;
  }
  sl_bdy.max_s = (sl_bdy.max_s <= sl_bdy.min_s)? (sl_bdy.min_s + 0.1) : sl_bdy.max_s;
  sl_bdy.max_l = (sl_bdy.max_l <= sl_bdy.min_l)? (sl_bdy.min_l + 0.1) : sl_bdy.max_l;
  return true;
}

bool ObjectToSL(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary &sl_bdy) {
	sl_bdy.Reset();
	if (map_info.path_points.empty()) return false;
	Site obj_center;
	obj_center.xg = temp_obj.xabs;
	obj_center.yg = temp_obj.yabs;
	obj_center.globalangle = temp_obj.global_angle;
	int center_index = -1;
	if (!GetFrenetOriginIndex(map_info, obj_center, center_index)) {
		AERROR_IF(FLAGS_log_enable)<<"GetFrenetOriginIndex failed.";
		return false;
	}
	double special_min_l = numeric_limits<double>::max();
	double special_max_l = numeric_limits<double>::lowest();
	for (auto &cell : temp_obj.cells) {
		double s, l;
		Site temp_p;
		temp_p.xg = cell.xg;
		temp_p.yg = cell.yg;
		if (!XYToSL(map_info.path_points, temp_p, center_index, s, l)) continue;
		if (s > -1.0 && l < special_min_l) {
			special_min_l = l;
		}
		if (s > -1.0 && l > special_max_l) {
			special_max_l = l;
		}
		sl_bdy.Set(s, l);
	}
	if (sl_bdy.min_s < -0.01 && sl_bdy.max_s > 0.01) {// 车侧并行障碍物
		sl_bdy.min_l = special_min_l;
		sl_bdy.max_l = special_max_l;
	}
	for (auto &lane_turn : map_info.lane_turns) {
		if (lane_turn.second != 4) continue;
		double nearest_s = (fabs(sl_bdy.min_s) < fabs(sl_bdy.max_s))? sl_bdy.min_s : sl_bdy.max_s;
		if (temp_obj.x < -5.0 && nearest_s > 0.0 && sl_bdy.min_l > 3.0) {
			sl_bdy.Reset();
			for (auto &cell : temp_obj.cells) {
				double s, l;
				Site temp_p;
				temp_p.xg = cell.xg;
				temp_p.yg = cell.yg;
				if (XYToSL(map_info.path_points, temp_p, 0, s, l)) {
					sl_bdy.Set(s, l);
				}
			}
		} 
		break;
	}
	sl_bdy.max_s = (sl_bdy.max_s <= sl_bdy.min_s)? (sl_bdy.min_s + 0.01) : sl_bdy.max_s;
	sl_bdy.max_l = (sl_bdy.max_l <= sl_bdy.min_l)? (sl_bdy.min_l + 0.01) : sl_bdy.max_l;
	return true;
}

bool PredictionToSL(const MapEngineLineList &map_info, LineObject &temp_obj) {
	temp_obj.sl_polygons.clear();
	if (map_info.path_points.empty()) return false;
	if (temp_obj.type < 2 || temp_obj.prediction.trajectories.empty() ||
		!temp_obj.prediction.is_ultra_static) {
		return true;
	}
	AWARN<<"Id "<<temp_obj.id<<" type "<<temp_obj.type<<", needs prediction sl."
		 <<" size "<<temp_obj.prediction.trajectories.size();
	// find referenceline points range to calculate sl
	int center_index = -1;
	Site obj_center;
	obj_center.xg = temp_obj.box.center_x();
	obj_center.yg = temp_obj.box.center_y();
	if (!GetFrenetOriginIndex(map_info, obj_center, center_index)) {
		return false;
	}
	for (auto &pd_circle : temp_obj.prediction.trajectories) {
		if (pd_circle.points.size() < 4) return true;
		vector<Vec2d> pd_sl;
		for (auto &pd_center : pd_circle.points) {
			double center_s, center_l;
			Site temp_center_p;
			temp_center_p.xg = pd_center.xg;
			temp_center_p.yg = pd_center.yg;
			XYToSL(map_info, temp_center_p, center_s, center_l);
			Vec2d temp_p;
			temp_p.set_x(pd_center.xg);
			temp_p.set_y(pd_center.yg);
			Box2d pd_box(temp_p, pd_center.globalangle*M_PI/180.0, 
						 temp_obj.box.length(), temp_obj.box.width());
			vector<Vec2d> pd_corners;
			pd_box.GetAllCorners(&pd_corners);
			double corner_s, corner_l;
			for (auto &pd_corner : pd_corners) {
				Site temp_p;
				temp_p.xg = pd_corner.x();
				temp_p.yg = pd_corner.y();
				if (!XYToSL(map_info.path_points, temp_p, center_index, corner_s, corner_l)) {
					continue;
				}
				pd_sl.push_back(Vec2d(corner_s, corner_l));
			}
		}
		Polygon2d sl_polygon;
		if (Polygon2d::ComputeConvexHull(pd_sl, &sl_polygon)) {
			sl_polygon.set_p(pd_circle.probability);
			temp_obj.sl_polygons.push_back(sl_polygon);
			AINFO_IF(FLAGS_log_enable)<<"obj "<<temp_obj.id<<" sl polygon s("
				<<sl_polygon.min_x()<<","<<sl_polygon.max_x()<<"), l("
				<<sl_polygon.min_y()<<","<<sl_polygon.max_y()<<").";
		}
	}
	return true;
}

bool GetFrenetOriginIndex(const MapEngineLineList &map_info, const Site &input_p, int &min_index) {
	min_index = -1;
	if (map_info.path_points.size() < 3) {
		AERROR <<"path points are too less to calculate s l.";
		return false;
	}
	double min_dis = std::numeric_limits<double>::max();
	if (map_info.path_points.empty()) {
		AERROR<<"GetGlobalNearestPoint no reference path_points.";
		return false;
	}
	for (int i = 0; i < map_info.path_points.size(); i += 10) {
		double distance = map_info.path_points[i].DistanceTo(input_p);
		if (distance < min_dis) {
			min_dis = distance;
			min_index = i;
		}
	}
	if (min_index < 0 || min_index >= map_info.path_points.size()) {
		return false;
	}
	int start_range = 0, end_range = map_info.path_points.size();
	if (min_index - 5 >= 0) start_range = min_index - 5;
	if (min_index + 6 <= end_range) end_range = min_index + 6;
	for (int i = start_range; i < end_range; i++) {
		double distance = map_info.path_points[i].DistanceTo(input_p);
		if (distance <= min_dis) {
			min_dis = distance;
			min_index = i;
		}
	}
	return true;
}



}
}
