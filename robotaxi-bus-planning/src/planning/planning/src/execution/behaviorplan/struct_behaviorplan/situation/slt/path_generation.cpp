#include "path_generation.h"

namespace acu {
namespace planning {
 
PathGeneration::PathGeneration() {}

PathGeneration::~PathGeneration() {}

bool PathGeneration::GeneratePath() {
  AddLineInfo();
  length_ = context_->planning_config_.car_model.length;
  width_ = context_->planning_config_.car_model.car_width;
  front_over_hang_ = context_->planning_config_.car_model.front_over_hang;
  half_width_ = 0.5 * width_;
  level_s_ = std::max(min_s_, 
      context_->planning_config_.speedplan_config.maximum_cruising_speed / 3.6);
  if (target_line_->mapinfo.path_points.empty()) {
    AERROR_IF(FLAGS_log_enable) << "Path point is empty!";
    return false;
  }
  sample_s_.clear();
  points_.clear();
  sample_points_.clear();
  GetInitState();
  BuildStaticMap();
  AddSampleLevel();
  AddSamplePoints();
  if (sample_points_.empty()) {
    return false;
  }
  if (SearchPath()) {
    GetExpandL();
  }
  return false;
}

// Get target line and reference lines
void PathGeneration::AddLineInfo() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = current_line_;
  // if (current_line_->mapinfo.dis_to_end > FLAGS_front_perception_range + 10.0) {
  //   target_line_ = current_line_;
  // } else {
  //   int all_lc_time = current_line_->mapinfo.all_lc_time;
  //   for (auto& line : context_->reference_line_map_) {
  //     if (line.second->mapinfo.all_lc_time < all_lc_time) {
  //       all_lc_time = line.second->mapinfo.all_lc_time;
  //       target_line_ = line.second;
  //     }
  //   }
  // }
  double left_w, right_w;
  target_line_->GetWidthToRoadBoundary(left_w, right_w, target_line_->loc_site_);
  road_side_ = right_w < 0.0;
  AERROR_IF(FLAGS_log_enable) << "road_side = " << road_side_ << 
  		" target_line = " << target_line_->reference_lane_id;
  lines_.clear();
  for (auto& line : context_->reference_line_map_) {
    if (line.first / 10 == 2) {
      lines_[line.first - 20] = line.second;
    } else {
      lines_[line.first] = line.second;
    }
  }
}

void PathGeneration::GetInitState() {
  // consider stitching trajectory
  // if (reference_line_->local_reference_line.mapinfo.path_points.size()) {
  //   auto& path = reference_line_->local_reference_line.mapinfo.path_points;
  //   for (auto it = path.begin(); it != path.end(); it++) {
  //     if (it->length < context_->ego_speed_) {
  //       continue;
  //     }
  //     double s, l;
  //     XYToSL(target_line_->mapinfo, *it, start_s_, start_l_);
  //     start_yaw_ = it->globalangle - target_line_->GetReferencePoint(start_s_).globalangle;
  //     return;
  //   }
  // }
  XYToSL(target_line_->mapinfo, target_line_->loc_site_, start_s_, start_l_);
  start_yaw_ = DP_->loc_perception.localization_data.yaw - 
      target_line_->mapinfo.path_points.front().globalangle;
}

// Add static objects in path bound
void PathGeneration::BuildStaticMap() {
  static_objects_.clear();
  for (auto& object : target_line_->objects_) {
    auto& obj = object.second;
    if (obj.sl_boundary.min_s > FLAGS_front_perception_range || 
        obj.sl_boundary.min_s > target_line_->mapinfo.dis_to_end || 
        obj.sl_boundary.min_s < 1e-3 || obj.is_static == false || 
        context_->cognition_info_->scenario_info.waiting_objects_.count(obj.id)) {
      continue;
    }
    double left_bd, right_bd;
    GetPathBound(obj.sl_boundary.min_s, left_bd, right_bd);
    if (obj.sl_boundary.min_l > left_bd || obj.sl_boundary.max_l < -right_bd) {
      continue;
    }
    static_objects_[obj.sl_boundary.min_s] = obj.id;
    static_boxes_.push_back(obj.box);
    // AINFO << "static_object = " << obj.id;
  }
}

// Get distance from s of target line to path bound
void PathGeneration::GetPathBound(const double s, double& left_bd, double& right_bd) {
  left_bd = 0.0;
  right_bd = 0.0;
  int index;
  Site point;
  target_line_->GetNearestPoint(s, point, index);
  double left_w, right_w;
  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (s < it->second->mapinfo.dis_to_end) {
      it->second->GetWidthToLaneBoundary(left_bd, right_w, point);
      break;
    }
  }
  if (road_side_ && s < lines_.rbegin()->second->mapinfo.dis_to_end && s < 20.0) {
    lines_.rbegin()->second->GetWidthToRoadBoundary(left_w, right_bd, point, false);
    // AINFO << "s = " << s << " right_bd = " << right_bd;
    return;
  }
  for (auto it = lines_.rbegin(); it != lines_.rend(); ++it) {
    if (s < it->second->mapinfo.dis_to_end) {
      it->second->GetWidthToLaneBoundary(left_w, right_bd, point);
      break;
    }
  }
}

// Get sample s according to speed, boundary type and static objects
void PathGeneration::AddSampleLevel() {
  double max_s = std::min(target_line_->mapinfo.dis_to_end, 
      FLAGS_front_perception_range + 10.0);
  double s = 0.0;
  while (s <= max_s) {
    sample_s_.push_back(std::make_pair(s, 0));
    s += level_s_;
  }
  // Add s from boundary type
  for (auto& bd : target_line_->mapinfo.left_bd_types) {
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (fabs(it->first - bd.first) < min_s_ && it->first > 0.0 && !it->second) {
        it->first = bd.first;
        it->second = 1;
        break;
      } else if (it->first > bd.first) {
        sample_s_.insert(it, std::make_pair(it->first, 1));
        break;
      }
    }
  }
  for (auto& bd : target_line_->mapinfo.right_bd_types) {
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (fabs(it->first - bd.first) < min_s_ && it->first > 0.0 && !it->second) {
        it->first = bd.first;
        it->second = 1;
        break;
      } else if (it->first > bd.first) {
        sample_s_.insert(it, std::make_pair(it->first, 1));
        break;
      }
    }
  }
  for (auto& object : static_objects_) {
    double temp_s = 0.0;
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (it->first >= object.first) {
        if (it->first - object.first >= min_s_ && object.first - temp_s >= min_s_) {
          sample_s_.insert(it, std::make_pair(object.first, 2));
        }
        break;
      }
      temp_s = it->first;
    }
  }
}

// Add two points in single line
void PathGeneration::AddSamplePoints() {
  double left_w, right_w, s, l;
  Site temp_site;
  MapPointStruct point;
  int index;
  for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
    // AWARN_IF(FLAGS_log_enable) << "s = " << it->first << " type = " << it->second;
    point.s = it->first;
    double radius = (pow(context_->ego_speed_, 2) - 2.0 * point.s) / 2.0;
    point.min_radius = std::max(radius, 10.0);
    std::vector<MapPointStruct> points;
    for (auto line : lines_) {
      auto it = line.second;
      it->GetWidthToLaneBoundary(left_w, right_w, point.s);
      it->GetNearestPoint(point.s, temp_site, index);
      XYToSL(target_line_->mapinfo, temp_site, s, l);
      point.l_solid = it->left_bds_.lower_bound(point.s)->second;
      point.r_solid = it->right_bds_.lower_bound(point.s)->second;
      // Add sample points on line for curvature constraint on highway
      const double on_line_width = 0.3;
      point.l = l + left_w - on_line_width;
      if (!point.l_solid && (points.empty() || point.l < points.back().l)) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l + left_w - half_width_;
      if (points.empty() || point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l;
      if (point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l + half_width_ - right_w;
      if (point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l + on_line_width - right_w;
      if (!point.r_solid && point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      if (line.first == lines_.rbegin()->first && road_side_ && point.s < 20.0) {
      	target_line_->GetWidthToRoadBoundary(left_w, right_w, point.s, false);
      	for (int i = 0; i < 10; i++) {
      		point.l = points.back().l - 0.6;
      		if (half_width_ - point.l > right_w) {
      			break;
      		}
      		// AINFO_IF(point.s < 1e-3) << "road_side_l = " << point.l;
      		AddPointLineIds(line.first, point);
      		points.push_back(point);
      	}
      } 
    }
    sample_points_.push_back(points);
  }
}

void PathGeneration::AddPointLineIds(const int id, MapPointStruct& point) {
  point.line_ids.clear();
  point.line_ids.push_back(id);
  double left_w, right_w, s, l;
  Site temp_site;
  int index, nearest_index;
  for (auto it = lines_.upper_bound(id); it != lines_.end(); ++it) {
    auto line = it->second;
    line->GetWidthToLaneBoundary(left_w, right_w, point.s);
    line->GetNearestPoint(point.s, temp_site, index);
    XYToSL(target_line_->mapinfo, temp_site, s, l);
    if (point.s < line->mapinfo.dis_to_end && point.l < left_w + l) {
      point.line_ids.push_back(it->first);
    } else {
      break;
    }
  }
  current_line_->GetNearestPoint(point.s, temp_site, nearest_index);
  point.xg = temp_site.xg - point.l * sin(temp_site.globalangle * M_PI / 180.0);
  point.yg = temp_site.yg + point.l * cos(temp_site.globalangle * M_PI / 180.0);
}

bool PathGeneration::SearchPath() {
  std::vector<MapPointStruct> points;
  MapPointStruct point = sample_points_.front().front();
  for (auto& pt : sample_points_.front()) {
    if (fabs(pt.l - start_l_) < std::fabs(point.l - start_l_)) {
      point = pt;
    }
  }
  point.delta = start_yaw_;
  if (fabs(point.delta) > 180.0) {
    point.delta += point.delta < 0.0 ? 360.0 : -360.0;
  } 
  point.delta = point.delta * M_PI / 180.0;
  point.lc_num = 0;
  points.push_back(point);
  points_.push_back(points);
  for (int i = 1; i < sample_points_.size(); i++) {
    if (points_.back().empty()) {
    	AERROR << "block_s = " << sample_s_[i - 1].first;
      return false;
    } 
    points.clear();
    int lc_num;
    double last_s = points_.back().front().s;
    for (auto& pt : sample_points_[i]) {
      for (auto& pre_point : points_.back()) {
        double cost;
        if (GetTotalCost(pre_point, pt, cost) && cost < pt.cost) {
          pt.lc_num = lc_num;
          pt.last_l = pre_point.l;
          pt.cost = cost;
        }
      }
      if (pt.cost < 1000000.0) {
        pt.delta = atan((pt.l - pt.last_l) / (pt.s - last_s));
        points.push_back(pt);
        // Get max s index for line
        for (auto index : pt.line_ids) {
          search_index_[index] = i;
        }
      }
    }
    points_.push_back(points);
  }
  return points_.back().size();
}

bool PathGeneration::GetTotalCost(const MapPointStruct& pre_point, 
                                  const MapPointStruct& point, double& cost) {
  if (point.l_solid && (point.line_ids.front() > pre_point.line_ids.back()) || 
      point.r_solid && (point.line_ids.back() < pre_point.line_ids.front())) {
    return false;
  }
  double dis = std::hypot(point.s - pre_point.s, point.l - pre_point.l) + 1e-5;
  double angle = atan2(point.l - pre_point.l, point.s - pre_point.s);
  double delta = fabs(pre_point.delta - angle);
  if (0.5 * dis < point.min_radius * sin(delta)) {
    return false;
  }
  double comfort_cost = 1.0 * fabs(point.l) + 20.0 * fabs(point.l - pre_point.l);
  if (pre_point.s > 0.0) {
    comfort_cost += 1000.0 * fabs(point.l - pre_point.last_l);
  }
  double safety_cost = 0.0;
  dis = std::hypot(point.xg - pre_point.xg, point.yg - pre_point.yg) + 1e-5;
  angle = atan2(point.yg - pre_point.yg, point.xg - pre_point.xg);
  double cos_theta = (point.xg - pre_point.xg) / dis;
  double sin_theta = (point.yg - pre_point.yg) / dis;
  double center_s_to_hang = cos_theta * (length_ - front_over_hang_);
  double center_l_to_hang = sin_theta * (length_ - front_over_hang_);
  Vec2d center(0.5 * (point.xg + pre_point.xg) + center_s_to_hang, 
      0.5 * (point.yg + pre_point.yg) + center_l_to_hang);
  Box2d ego_box(center, angle, length_, width_);
  for (auto& box : context_->decision_result_.dynamic_sl) {
    double distance = box.DistanceTo(ego_box);
    safety_cost += 25.0 / std::max(distance, 1e-5);
    if (distance < FLAGS_collision_buff) {
      return false;
    }
  }
  for (auto& box : static_boxes_) {
    double distance = box.DistanceTo(ego_box);
    safety_cost += 50.0 / std::max(distance, 1e-5);
    if (distance < FLAGS_collision_buff) {
      return false;
    }
  }
  cost = comfort_cost + safety_cost;
  return true;
}

void PathGeneration::GetExpandL() {
  double left_w, right_w;
  MapPointStruct temp_point = points_.back().front();
  for (auto& point : points_.back()) {
    if (temp_point.line_ids.front() > current_line_->reference_lane_id || 
        temp_point.line_ids.back() < current_line_->reference_lane_id) {
      if (point.line_ids.front() <= current_line_->reference_lane_id &&  
          point.line_ids.back() >= current_line_->reference_lane_id ||
          point.cost < temp_point.cost) {
        temp_point = point;
      }
    } else {
      if (point.line_ids.front() <= current_line_->reference_lane_id && 
          point.line_ids.back() >= current_line_->reference_lane_id && 
          point.cost < temp_point.cost) {
        temp_point = point;
      }
    }
  }
  current_line_->GetWidthToLaneBoundary(left_w, right_w, temp_point.s);
  context_->decision_result_.expand_l = std::max(right_w - temp_point.l, 0.0);
  AERROR_IF(FLAGS_log_enable) << "s = " << temp_point.s << " l = " << temp_point.l << 
      " offset = " << right_w - temp_point.l << " cost = " << temp_point.cost;
  for (int i = points_.size() - 2; i >= 0; i--) {
    for (auto& point : points_[i]) {
      if (point.l == temp_point.last_l) {
        temp_point = point;
        current_line_->GetWidthToLaneBoundary(left_w, right_w, point.s);
        double expand_l = -right_w - (point.l - half_width_);
        AERROR_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << 
            point.l << " expand_l = " << expand_l;
        if (expand_l > context_->decision_result_.expand_l) {
          context_->decision_result_.expand_l = expand_l;
        }
        break;
      }
    }
  }
}

}  //  namespace planning
}  //  namespace acu
