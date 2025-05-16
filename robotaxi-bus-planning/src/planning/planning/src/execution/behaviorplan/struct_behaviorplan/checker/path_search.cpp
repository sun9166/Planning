#include "path_search.h"

namespace acu {
namespace planning {
 
PathSearch::PathSearch() {}

PathSearch::~PathSearch() {}

bool PathSearch::GeneratePath() {
  AddLineInfo();
  half_width_ = 0.5 * context_->planning_config_.car_model.car_width;
  buffer_ = FLAGS_collision_buff;
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
  return SearchPath();
}

// Get target line and reference lines
void PathSearch::AddLineInfo() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = current_line_;
  if (current_line_->mapinfo.dis_to_end > FLAGS_front_perception_range + 10.0) {
    target_line_ = current_line_;
  } else {
    int all_lc_time = current_line_->mapinfo.all_lc_time;
    for (auto& line : context_->reference_line_map_) {
      if (line.second->mapinfo.all_lc_time < all_lc_time) {
        all_lc_time = line.second->mapinfo.all_lc_time;
        target_line_ = line.second;
      }
    }
  }
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

void PathSearch::GetInitState() {
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
void PathSearch::BuildStaticMap() {
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
    // AINFO << "static_object = " << obj.id;
  }
}

// Get distance from s of target line to path bound
void PathSearch::GetPathBound(const double s, double& left_bd, double& right_bd) {
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
void PathSearch::AddSampleLevel() {
  double max_s = std::min(target_line_->mapinfo.dis_to_end, 
      FLAGS_front_perception_range + 10.0);
  if (context_->scenario_type != eScenarioEnum::PULLOVER) {
     max_s = std::min(max_s, current_line_->mapinfo.dis_to_merge + FLAGS_min_lc_dis);
  }
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
void PathSearch::AddSamplePoints() {
  double left_w, right_w, s, l;
  Site temp_site;
  MapPointStruct point;
  int index;
  for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
    // AWARN_IF(FLAGS_log_enable) << "s = " << it->first << " type = " << it->second;
    point.s = it->first;
    double radius = (pow(context_->ego_speed_, 2) - 2.0 * point.s) / 2.0;
    point.min_radius = std::max(radius, 5.0);
    std::vector<MapPointStruct> points;
    for (auto line : lines_) {
      auto it = line.second;
      it->GetWidthToLaneBoundary(left_w, right_w, point.s);
      it->GetNearestPoint(point.s, temp_site, index);
      XYToSL(target_line_->mapinfo, temp_site, s, l);
      point.l_solid = it->left_bds_.lower_bound(point.s)->second;
      point.r_solid = it->right_bds_.lower_bound(point.s)->second;
      if (pull_over_) {
        point.l_solid = false;
        point.r_solid = false;
      }
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

void PathSearch::AddPointLineIds(const int id, MapPointStruct& point) {
  point.line_ids.clear();
  point.line_ids.push_back(id);
  double left_w, right_w, s, l;
  Site temp_site;
  int index;
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
}

bool PathSearch::SearchPath() {
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
        if (SatisfyConstrain(pre_point, pt, lc_num)) {
          if (lc_num < pt.lc_num) {
            pt.lc_num = lc_num;
            pt.last_l = pre_point.l;
          }
        }
      }
      if (pt.lc_num < MAX_COST) {
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

bool PathSearch::SatisfyConstrain(const MapPointStruct& pre_point, 
                                  const MapPointStruct& point, int& lc_num) {
  lc_num = MAX_COST;
  if (point.l_solid && (point.line_ids.front() > pre_point.line_ids.back()) || 
      point.r_solid && (point.line_ids.back() < pre_point.line_ids.front())) {
    return false;
  }
  double s_1 = pre_point.s, s_2 = point.s;
  double l_1 = pre_point.l, l_2 = point.l;
  double k = (l_2 - l_1) / (s_2 - s_1);
  double delta = fabs(pre_point.delta - atan(k));
  if (fabs(l_2 - l_1) > 0.1 && s_2 - s_1 > 3.0 &&
      0.5 * std::hypot(s_2 - s_1, l_2 - l_1) < point.min_radius * sin(delta)) {
    return false;
  }
  for (auto it = static_objects_.begin(); it != static_objects_.end(); ++it) {
    auto& object = target_line_->objects_[it->second];
    if (object.sl_boundary.min_s > point.s) {
      break;
    } else if (object.sl_boundary.max_s < pre_point.s) {
      continue;
    }
    double min_l = l_1 + k * std::max(object.sl_boundary.min_s - s_1, 0.0);
    double max_l = l_1 + k * std::min(object.sl_boundary.max_s - s_1, s_2 - s_1);
    if (min_l + (half_width_ + buffer_) > object.sl_boundary.min_l && 
        min_l - (half_width_ + buffer_) <  object.sl_boundary.max_l || 
        max_l + (half_width_ + buffer_) > object.sl_boundary.min_l && 
        max_l - (half_width_ + buffer_) <  object.sl_boundary.max_l) {
      return false;
    }
  }
  lc_num = pre_point.lc_num;
  if (point.line_ids.front() > pre_point.line_ids.back()) {
    lc_num += point.line_ids.front() - pre_point.line_ids.back();
  } else if (pre_point.line_ids.front() > point.line_ids.back()) {
    lc_num += pre_point.line_ids.front() - point.line_ids.back();
  }
  return true;
}

void PathSearch::GeneratePath(std::map<int, ReferenceLineFrame*> lines) {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = current_line_;
  if (current_line_->mapinfo.dis_to_end > FLAGS_front_perception_range + 10.0) {
    target_line_ = current_line_;
  } else {
    for (auto it = lines.begin(); it != lines.end(); ++it) {
      if (it->second->mapinfo.dis_to_end > current_line_->mapinfo.dis_to_end + 5.0) {
        target_line_ = it->second;
      }
    }
  }
  AWARN_IF(FLAGS_log_enable) << "target_line = " << target_line_->reference_lane_id;
  road_side_ = false;
  lines_ = lines;
  half_width_ = 0.5 * context_->planning_config_.car_model.car_width;
  buffer_ = FLAGS_collision_buff;
  level_s_ = 5.0;
  pull_over_ = true;
  sample_s_.clear();
  points_.clear();
  sample_points_.clear();
  if (target_line_->mapinfo.path_points.empty()) {
    AERROR_IF(FLAGS_log_enable) << "Path point is empty!";
    return;
  }
  GetInitState();
  BuildStaticMap();
  AddSampleLevel();
  AddSamplePoints();
  if (sample_points_.empty()) {
    return;
  }
  SearchPath();
}

bool PathSearch::CheckDestPassable(const int line_id, const double s) {
  int index = (int)(s / level_s_);
  if (points_.size() > index) {
    for (auto& point : points_[index]) {
      if (line_id >= point.line_ids.front() && line_id <= point.line_ids.back()) {
        AWARN_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << point.l;
        return true;
      }
    }
  }
  if (points_.size() > index + 1) {
    for (auto& point : points_[index + 1]) {
      if (line_id >= point.line_ids.front() && line_id <= point.line_ids.back()) {
        AWARN_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << point.l;
        return true;
      }
    }
  }
  return false;
}

// // int PathSearch::LineLcNumber(const int line_id) {
// //   int lc_num = MAX_COST;
// //   if (search_index_.count(line_id) && search_index_[line_id] >= 0) {
// //     double search_s = sample_s_[search_index_[line_id]].first;
// //     if (search_s < frame_info_->line_frames[line_id].mapinfo.dis_to_end && 
// //         search_s < sample_s_.back().first) {
// //       return MAX_COST;
// //     }
// //     for (auto& point : points_[search_index_[line_id]]) {
// //       if (line_id >= point.line_ids.front() && line_id <= point.line_ids.back()) {
// //         if (point.lc_num < lc_num) {
// //           lc_num = point.lc_num;
// //         }
// //       }
// //     }
// //   } else {
// //     return MAX_COST;
// //   }
// //   return lc_num;
// // }

// // bool PathSearch::GetPathAndBoundary(std::vector<MapPointStruct>& points,
// //                                     PathBound& bound) {
// //   points.clear();
// //   bound.clear();
// //   MapPointStruct point;
// //   int line_id = target_line_->line_id;
// //   int lc_num = MAX_COST;
// //   if (search_index_.count(line_id) && search_index_[line_id] >= 0) {
// //     for (auto& pt : points_[search_index_[line_id]]) {
// //       if (line_id >= pt.line_ids.front() && line_id <= pt.line_ids.back()) {
// //         if (pt.lc_num < lc_num || pt.lc_num == lc_num && fabs(pt.l) < fabs(point.l)) {
// //           lc_num = pt.lc_num;
// //           point = pt;
// //         }
// //       }
// //     }
// //     points.push_back(point);
// //     int left_bd = point.is_solid ? point.line_ids.front() : left_bd_;
// //     int right_bd = point.is_solid ? point.line_ids.back() : right_bd_;
// //     bound.push_back(make_tuple(point.s, left_bd, right_bd));
// //     AWARN_IF(FLAGS_lat_log) << "s = " << point.s << " left_bd = " << 
// //         left_bd << " right_bd = " << right_bd;
// //     AWARN_IF(FLAGS_lat_log) << "s = " << point.s << " l = " << point.l;
// //     for (int i = search_index_[line_id] - 1; i >= 0 ; i--) {
// //       for (auto& pt : points_[i]) {
// //         if (point.last_l == pt.l) {
// //           AWARN_IF(FLAGS_lat_log) << "s = " << pt.s << " l = " << pt.l;
// //           points.push_back(pt);
// //           if (pt.is_solid != point.is_solid) {
// //             left_bd = pt.is_solid ? pt.line_ids.front() : left_bd_;
// //             right_bd = pt.is_solid ? pt.line_ids.back() : right_bd_;
// //             bound.push_back(make_tuple(pt.s, left_bd, right_bd));
// //             AWARN_IF(FLAGS_lat_log) << "s = " << pt.s << " left_bd = " << 
// //                 left_bd << " right_bd = " << right_bd;
// //           }
// //           point = pt;
// //           break;
// //         }
// //       }
// //     }
// //     reverse(bound.begin(), bound.end());
// //     return true;
// //   }
// //   return false;
// // }

}  //  namespace planning
}  //  namespace acu
