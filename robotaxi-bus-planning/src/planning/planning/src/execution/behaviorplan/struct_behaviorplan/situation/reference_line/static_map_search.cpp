#include "static_map_search.h"

namespace acu {
namespace planning {

StaticMapSearch::StaticMapSearch() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  length_ = context_->planning_config_.car_model.length;
  half_width_ = 0.5 * context_->planning_config_.car_model.car_width;
  front_over_hang_ = context_->planning_config_.car_model.front_over_hang;
  back_over_hang_ = context_->planning_config_.car_model.back_over_hang;
}

StaticMapSearch::~StaticMapSearch() {}

void StaticMapSearch::GeneratePath(const bool is_same, const int direction, 
                                   OptionStruct& option) {
  option.lc_cost = MAX_COST;
  points_.clear();
  buffer_ = option.type == 2 && context_->dis_to_junction_ > 100.0 ? 
      FLAGS_collision_buff + 0.2 : FLAGS_collision_buff;
  GenerateGrid(is_same, direction, option);
  if (points_.empty() || points_.front().empty()) {
    return;
  }
  graph_nodes_.clear();
  MapNodeStruct node = points_.front().front();
  std::vector<MapNodeStruct> line;
  double ego_l = current_line_->mapinfo.dis2line;
  for (int i = 1; i < points_.front().size(); i++) {  
    if (std::fabs(points_.front().at(i).l - ego_l) < std::fabs(node.l - ego_l)) {
      node = points_.front().at(i);
    }
  }
  if (node.lane_num > 0 && points_.front().at(1).lane_num == 0 && 
      ego_l <= points_.front().at(0).l && ego_l >= points_.front().at(1).l) {
    node = fabs(ego_l - points_.front().at(0).l) > fabs(ego_l - points_.front().at(1).l) ? 
        points_.front().at(1) : points_.front().at(0);
  }
  node.lc_cost = 0;
  line.push_back(node);
  graph_nodes_.push_back(line);
  for (int level = 1; level < points_.size(); level++) {
    if (graph_nodes_.back().empty()) {
      return;
    } 
    std::vector<MapNodeStruct> cur_line;
    int lc_cost;
    for (int i = 0; i < points_[level].size(); i++) {
      MapNodeStruct cur_node = points_[level][i];
      bool collision_free = false;
      for (auto& last_node : graph_nodes_.back()) {
        if (CheckCollision(last_node, cur_node, lc_cost) == false) {
          collision_free = true;
          if (lc_cost < cur_node.lc_cost || lc_cost == cur_node.lc_cost && 
              lc_cost == 1 && !is_same && LcEarly(last_node, cur_node)) {
            cur_node.lc_cost = lc_cost;
            cur_node.last_l = last_node.l;
            cur_node.last_lane_num = last_node.lane_num;
            cur_node.lc_dis = last_node.lc_dis;
          }
        }
      }
      if (collision_free) {
        if (cur_node.lc_cost == 1 && cur_node.lane_num != cur_node.last_lane_num) {
          cur_node.lc_dis = cur_node.s;
        } 
        cur_line.push_back(cur_node);
      }
    }
    graph_nodes_.push_back(cur_line);
  }
  if (graph_nodes_.back().empty()) {
    return;
  }
  for (auto& node : graph_nodes_.back()) {
    if (node.lc_cost < option.lc_cost && graph_nodes_.size() > 1) {
      option.lc_cost = node.lc_cost;
      AERROR << "size = " << graph_nodes_.size() << " s = " << node.s;
      BoundaryDecision(direction, node, option);
    }
  }
}

bool StaticMapSearch::LcEarly(const MapNodeStruct& last_node, const MapNodeStruct& cur_node) {
  double lc_dis = last_node.lc_cost == 1 ? last_node.lc_dis : cur_node.s;
  if (lc_dis < 4.0 * context_->ego_speed_) {
    return false;
  }
  if (cur_node.lane_num > cur_node.last_lane_num) {
    return cur_node.s > lc_dis;
  } else {
    return cur_node.lc_dis > lc_dis;
  }
}

void StaticMapSearch::BoundaryDecision(const int direction, 
                                       const MapNodeStruct& last_node,
                                       OptionStruct& option) {
  auto line_type = direction == 1 ? 
      current_line_->mapinfo.left_bd_types : 
      current_line_->mapinfo.right_bd_types;
  double last_l = last_node.last_l;
  int last_lane_num = last_node.last_lane_num;
  std::map<double, MapNodeStruct> s_l;
  s_l[last_node.s] = last_node;
  // option.lc_num = last_node.lane_num;
  for (int i = graph_nodes_.size() - 2; i >= 0; i--) {
    for(auto node : graph_nodes_.at(i)) {
      if (node.l == last_l && node.lane_num == last_lane_num) {
        s_l[node.s] = node;
        last_l = node.last_l;
        last_lane_num = node.last_lane_num;
        // AERROR << "s = " << node.s << " l = " << node.l << " lane_num = " << node.lane_num;  
        // option.lc_num = std::max(option.lc_num, node.lane_num);
      }
    }
  }
  option.boundary.clear();
  for (int j = 0; j < line_type.size(); j++) {
    if (line_type.at(j).second > 2) {
      if (line_type.at(j).first >= s_l.rbegin()->first) {
        option.boundary.push_back(s_l.rbegin()->second.lane_num == 0);
      } else {
        auto lane_num = s_l.lower_bound(line_type.at(j).first)->second.lane_num;
        option.boundary.push_back(lane_num == 0);
      }
    } else {
      option.boundary.push_back(false);
    }
    if (line_type.at(j).first > current_line_->mapinfo.dis_to_end) {
      break;
    }
  }
  for (int k = 0; k < line_type.size() - option.boundary.size(); k++) {
    option.boundary.push_back(option.boundary.back());
  }
}

bool StaticMapSearch::CheckPolygon(const MapNodeStruct& last_node, 
                                   const MapNodeStruct& cur_node, LineObject& object) {
  double cos_theta = (cur_node.s - last_node.s) / 
      std::hypot(cur_node.s - last_node.s, cur_node.l - last_node.l);
  double sin_theta = (cur_node.l - last_node.l) / 
      std::hypot(cur_node.s - last_node.s, cur_node.l - last_node.l);
  double tan_theta = (cur_node.l - last_node.l) / (cur_node.s - last_node.s);
  double s_1 = last_node.s - back_over_hang_ * cos_theta;
  double s_2 = cur_node.s + front_over_hang_ * cos_theta;
  double l_1 = last_node.l - back_over_hang_ * sin_theta;
  double l_2 = cur_node.l + front_over_hang_ * sin_theta;
  Vec2d point(0.5 * (s_1 + s_2), 0.5 * (l_1 + l_2));
  Box2d ego_box(point, atan(tan_theta), length_, 2.0 * half_width_);
  if (object.sl_polygons.front().DistanceTo(ego_box) < 1e-3) {
    return true;
  }
  return false;
}

bool StaticMapSearch::CheckCollision(const MapNodeStruct& last_node, 
                                     const MapNodeStruct& cur_node, int& cost) {
  if (cur_node.is_solid && cur_node.lane_num != last_node.lane_num || 
      fabs(cur_node.lane_num - last_node.lane_num) > 1 && 
      !cur_node.is_opposite && !last_node.is_opposite) {
    return true;
  }
  double s_1 = last_node.s, s_2 = cur_node.s;
  double l_1 = last_node.l, l_2 = cur_node.l;
  double k = (l_2 - l_1) / (s_2 - s_1);
  for (auto it = current_line_->objects_.begin(); it != current_line_->objects_.end(); it++) {
    auto &object = it->second;
    if (object.sl_boundary.max_s < s_1 || object.sl_boundary.min_s > s_2 || 
        (!object.is_static && !object.prediction.is_ultra_static)) {
      continue;
    }
    double buffer = context_->planning_config_.car_model.front_over_hang > 
        object.sl_boundary.min_s - context_->ego_speed_ ? 0.2 : buffer_;
    double min_l = l_1 + k * std::max(object.sl_boundary.min_s - s_1, 0.0);
    double max_l = l_1 + k * std::min(object.sl_boundary.max_s - s_1, s_2 - s_1);
    if (min_l + (half_width_ + buffer) > object.sl_boundary.min_l && 
        min_l - (half_width_ + buffer) <  object.sl_boundary.max_l || 
        max_l + (half_width_ + buffer) > object.sl_boundary.min_l && 
        max_l - (half_width_ + buffer) <  object.sl_boundary.max_l) {
      return true;
    }
    if (object.sl_polygons.size() && CheckPolygon(last_node, cur_node, object)) {
      return true;
    }
  }
  cost = last_node.lc_cost + fabs(cur_node.lane_num - last_node.lane_num);
  return false;
}

void StaticMapSearch::GenerateGrid(const bool is_same, const int direction, 
                                   OptionStruct& option) {
  if (option.type == 1) {
    current_line_ = option.line_info;
  } else {
    current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  }
  double range = FLAGS_front_perception_range + 1.5 * LEVEL_S;
  if (current_line_->dis_to_last_lc < FLAGS_front_perception_range && 
      option.line_info->mapinfo.all_lc_time < current_line_->mapinfo.all_lc_time) {
    range = std::max(6.7 * context_->ego_speed_, FLAGS_min_lc_dis) + LEVEL_S;
    if (option.line_info->plan_length > LEVEL_S && 
        option.line_info->plan_length < range) {
      range = option.line_info->plan_length;
      AWARN_IF(FLAGS_log_enable) << "follow " << option.line_info->plan_follow_id;
    }
  }
  if (option.line_info->jam_level > 0) {
    range = std::min(option.line_info->plan_length, range);
    AWARN_IF(FLAGS_log_enable) << "Modify plan_length = " << range;
  }
  double current_length = std::min(current_line_->mapinfo.dis_to_end, range);
  double left_length = -1.0;
  double right_length = -1.0;
  if (direction == 1) {
    if (reference_line_->left_reference_line.size()) {
      left_line_ = &reference_line_->left_reference_line.back();
      left_length = std::min(left_line_->mapinfo.dis_to_end, range);
    } else {
      left_line_ = &reference_line_->reverse_reference_line;
      if (context_->decider_config_->reference_line().borrow_road_enable() && 
          left_line_->reference_lane_id > 0 && left_line_->mapinfo.path_points.size()) {
        AERROR << "rev_id = " << left_line_->reference_lane_id;
        left_length = std::min(left_line_->mapinfo.path_points.back().length, range);
      }
    }
    if (is_same) {
      if (left_line_->mapinfo.right_bd_types.size() > 1 && 
          left_line_->mapinfo.right_bd_types.back().second > 2) {
        int index = left_line_->mapinfo.right_bd_types.size() - 2;
        left_length = left_line_->mapinfo.right_bd_types[index].first;
      }
      left_length = std::min(left_length, current_length - LEVEL_S);
    } else {
      left_length = std::min(option.line_info->mapinfo.dis_to_end, range);
      current_length = std::min(current_length, left_length - LEVEL_S);
    }
  } else if (direction == 2) {
    if (reference_line_->right_reference_line.size()) {
      right_line_ = &reference_line_->right_reference_line.front();
      right_length = std::min(right_line_->mapinfo.dis_to_end, range);
    }
    if (is_same) {
      if (reference_line_->right_reference_line.size() && 
          right_line_->mapinfo.left_bd_types.size() > 1 && 
          right_line_->mapinfo.left_bd_types.back().second > 2) {
        int index = right_line_->mapinfo.left_bd_types.size() - 2;
        right_length = right_line_->mapinfo.left_bd_types[index].first;
      }
      right_length = std::min(right_length, current_length - LEVEL_S);
    } else {
      right_length = std::min(option.line_info->mapinfo.dis_to_end, range);
      current_length = std::min(current_length, right_length - LEVEL_S);
    }
  }
  AWARN_IF(FLAGS_log_enable) << "left = " << left_length << " cur = " << 
      current_length << " right = " << right_length;
  AddBoundary(direction, current_length, left_length, right_length);
}

void StaticMapSearch::AddBoundary(const int direction, const double current_len, 
                                  const double left_len, const double right_len) {
  MapNodeStruct point;
  std::vector<MapNodeStruct> level_points;
  double accumulate_s = 0.0, seg_s = 100.0;
  auto line_type = direction == 1 ? current_line_->mapinfo.left_bd_types : 
      current_line_->mapinfo.right_bd_types;
  int index = 0, num = 0, level_solid = line_type.size() ? line_type.front().second > 2 : 0;
  for (auto &pathpoint : current_line_->mapinfo.path_points) {
    if (line_type.size() > index) {
      seg_s = line_type.at(index).first;
      level_solid = line_type.at(index).second > 2;
    }
    if (pathpoint.length > accumulate_s) {
      point.s = accumulate_s;
      point.is_solid = level_solid;
      if (fabs(accumulate_s - seg_s) < 0.5 * LEVEL_S) {
        index++;
        if (accumulate_s < LEVEL_S) {
          point.is_solid = !point.is_solid;
        } else {
          point.s = seg_s;
        }       
      } 
      accumulate_s += LEVEL_S;
    } else {
      continue;
    }
    level_points.clear();    
    if (current_len > point.s) {
      AddPoints(0, point, level_points);
      num++;
    }
    if (left_len > point.s) {
      AddPoints(1, point, level_points);
    }
    if (right_len > point.s) {
      AddPoints(2, point, level_points);
    }
    if (level_points.empty()) {
      break;
    }
    points_.push_back(level_points);
  }
  if (points_.size() && num == points_.size()) {
    level_points.clear();
    point = points_.back().back();
    point.lane_num = direction == 1 ? 1 : -1;
    if (left_len > current_len) {
      point.s = left_len;
      level_points.push_back(point);
      points_.push_back(level_points);
    } else if (right_len > current_len) {
      point.s = right_len;
      level_points.push_back(point);
      points_.push_back(level_points);
    }
  }
}

void StaticMapSearch::AddPoints(const int direction, MapNodeStruct& point, 
                                std::vector<MapNodeStruct>& points) {
  int i = 0;
  Site temp_site;
  double left_l, right_l, left_w, right_w, s, l = 0.0;
  point.is_opposite = false;
  point.lc_cost = std::numeric_limits<int>::max();
  point.lc_dis = 0.0;
  if (direction == 0) {
    point.lane_num = 0;
    current_line_->GetWidthToLaneBoundary(left_l, right_l, point.s);
  } else {
    if (direction == 1) {
      point.lane_num = reference_line_->left_reference_line.size() ? 1 : 2;
      point.is_opposite = reference_line_->left_reference_line.empty();
      left_line_->GetWidthToLaneBoundary(left_l, right_l, point.s);
      left_line_->GetNearestPoint(point.s, temp_site, i);
    } else {
      point.lane_num = -1;
      right_line_->GetWidthToLaneBoundary(left_l, right_l, point.s);
      right_line_->GetNearestPoint(point.s, temp_site, i);
    }
    XYToSL(current_line_->mapinfo, temp_site, s, l);
  }
  point.l = l + left_l - half_width_;
  points.push_back(point);
  point.l = l + half_width_ - right_l;
  points.push_back(point);
  // Add road boundary point
  current_line_->GetWidthToRoadBoundary(left_w, right_w, point.s);
  if (direction == 1 && reference_line_->left_reference_line.size()) {
    point.lane_num = 2;
    point.l = left_w - half_width_;
    points.push_back(point);
  } else if (direction == 2) {
    point.lane_num = -2;
    point.l = half_width_ - right_w;
    points.push_back(point);
  }
}

}  //  namespace planning
}  //  namespace acu
