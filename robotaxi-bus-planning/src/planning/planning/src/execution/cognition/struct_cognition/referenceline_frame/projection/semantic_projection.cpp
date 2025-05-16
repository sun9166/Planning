#include "semantic_projection.h"

namespace acu{
namespace planning {

SemanticProjection::SemanticProjection() {} 

SemanticProjection::~SemanticProjection() {}

void SemanticProjection::AddSemanticInfo(ReferenceLineFrame& reference_line) {
  reference_line_ = &reference_line;
  target_line_= reference_line_;
  if (reference_line_->reference_lane_id == DP_->reference_line_info.current_line_id) {
    LineLimitSpace();
  }
  else if (reference_line_->reference_lane_id == 40) {
    for (auto &lineframe : DP_->reference_line_info.current_reference_line) {
      if (lineframe.reference_lane_id == DP_->reference_line_info.current_line_id) {
        reference_line_->block_id = lineframe.block_id;
      }
      if (lineframe.reference_lane_id == DP_->reference_line_info.target_line_id) {
        target_line_ = &lineframe;
      }
    }
    if (DP_->reference_line_info.target_line_id >= 30) {
      for (auto &lineframe : DP_->reference_line_info.right_reference_line) {
        if (lineframe.reference_lane_id == DP_->reference_line_info.target_line_id) {
          target_line_ = &lineframe;
        }
      }
    } else if(DP_->reference_line_info.target_line_id >= 20){
      for (auto &lineframe : DP_->reference_line_info.left_reference_line) {
        if (lineframe.reference_lane_id == DP_->reference_line_info.target_line_id) {
          target_line_ = &lineframe;
        }
      }
    }
  }
  follow_objects_.clear();
  park_objects_.clear();
  GetBlockL();
  AddBoundaryInfo();
  AddPassableInfo();
  GetJamLevel();
  AddQueueInfo();
	AddBlockInfo();
  GetQueueLength();
	AddSpeedInfo();
  AddSafetyInfo();
  AddSpeedCost();
  if (reference_line_->reference_lane_id == DP_->reference_line_info.current_line_id) {
    AddReverseInfo();
    AddConflictInfo();
  } 
  AERROR_IF(FLAGS_log_enable) <<"Line "<<reference_line_->reference_lane_id<<
      " block counter = " << reference_line_->block_counter << 
      " slow counter = " << reference_line_->slow_counter;
  AERROR_IF(FLAGS_log_enable) << 
      " block state = " << reference_line_->line_blocked << 
      " slow state = " << reference_line_->line_slow << 
      " temp block = " << reference_line_->temp_blocked <<
      " line_queue = " << reference_line_->line_queue << 
      " line_speed = " << reference_line_->line_speed << 
      " average_v = " << reference_line_->average_v << 
      " is_congestion = " << reference_line_->is_congestion;
}

void SemanticProjection::AddBoundaryInfo() {
  reference_line_->left_bds_.clear();
  for (auto& bd : reference_line_->mapinfo.left_bd_types) {
    reference_line_->left_bds_[bd.first] = bd.second > 2;
  }
  reference_line_->right_bds_.clear();
  for (auto& bd : reference_line_->mapinfo.right_bd_types) {
    reference_line_->right_bds_[bd.first] = bd.second > 2;
  }
}

void SemanticProjection::AddPassableInfo() {
  reference_line_->dis_to_last_lc = 1000.0;
  if (reference_line_->mapinfo.all_lc_time == 1000) {
    reference_line_->dis_to_last_lc = 0.0;
    return;
  } else if (reference_line_->mapinfo.first_lc_time == 0) {
    return;
  }
  auto passable_s = reference_line_->mapinfo.first_lc_time < 0 ? 
      reference_line_->mapinfo.left_passable_distances : 
      reference_line_->mapinfo.right_passable_distances;
  int num = fabs(reference_line_->mapinfo.first_lc_time);
  for (auto it = passable_s.rbegin(); it != passable_s.rend(); it++) {
    if (it->first > reference_line_->mapinfo.dis2missionpoint) {
      continue;
    } 
    double s = reference_line_->mapinfo.dis2missionpoint;
    if (s < reference_line_->mapinfo.dis_to_end) {
      double road_l, road_r, bound_l, bound_r;
      reference_line_->GetWidthToRoadBoundary(road_l, road_r, s, true);
      reference_line_->GetWidthToRoadBoundary(bound_l, bound_r, s, false);
      if (bound_r > road_r + 0.5) {
        s -= FLAGS_min_lc_dis;
      }
    }
    double end_s = std::min(it->second, s);
    if (it->first + num * FLAGS_min_lc_dis > end_s) {
      num -= (int)((end_s - it->first) / FLAGS_min_lc_dis);
      AWARN_IF(FLAGS_log_enable) << "Need set lc point before last road!";
      continue;
    } 
    reference_line_->dis_to_last_lc = end_s - num * FLAGS_min_lc_dis;
    AWARN_IF(FLAGS_log_enable) << "dis_to_last_lc = " << reference_line_->dis_to_last_lc;
    return;
  }
  reference_line_->dis_to_last_lc = 0.0;
  AWARN_IF(FLAGS_log_enable) << "Lc distance is not enough!";
}

void SemanticProjection::AddBlockInfo() {
  reference_line_->line_blocked = false;
  if (IsTempCollision()) {
    reference_line_->block_counter++;
    if (reference_line_->block_counter >= FLAGS_block_times) {
      reference_line_->line_blocked = true;
      return;
    }
  } else {
    reference_line_->block_counter = 0;
  }
}

void SemanticProjection::GetBlockL() {
  for (auto it = reference_line_->objects_.begin(); it != reference_line_->objects_.end(); it++) {
    auto &object = it->second;
    double left_w, right_w;
    reference_line_->GetWidthToLaneBoundary(left_w, right_w, object.sl_boundary.min_s);
    double left_l = left_w - object.sl_boundary.max_l;
    double right_l = object.sl_boundary.min_l + right_w;
    if (object.sl_polygons.size()) {
      for (auto& point : object.sl_polygons.front().points()) {
        if (point.y() > object.sl_boundary.max_l) {
          left_l = left_w - point.y();
        } else if (point.y() < object.sl_boundary.min_l) {
          right_l = point.y() + right_w;
        }
      }
      AINFO_IF(FLAGS_log_enable) << "left_l = " << left_l << " right_l = " << right_l;
    }
    if (left_l < reference_line_->car_model_.car_width + FLAGS_collision_buff + 0.2 && 
        right_l < reference_line_->car_model_.car_width + FLAGS_collision_buff + 0.2) {
      double block_l = std::max(left_l, right_l) - reference_line_->car_model_.car_width;
      if (object.sl_boundary.max_s > reference_line_->car_model_.front_over_hang) {
        reference_line_->block_l_[object.id] = std::max(block_l, 0.0);
      }
      if (object.type < 2 && (object.conflict_type < 5 || object.is_static) && 
          block_l < FLAGS_collision_buff) {
        reference_line_->car_info_[object.sl_boundary.min_s] = object.id;
      }
      AINFO_IF(FLAGS_log_enable) << "id = " << object.id << 
          " passable_l = " << block_l << " v = " << object.speed;
    }
  }
  FindFollowObjects();
  if (reference_line_->reference_lane_id == DP_->reference_line_info.current_line_id) {
    AddBlindDis();
  }
}

void SemanticProjection::AddBlindDis() {
  reference_line_->blind_dis.first = FLAGS_front_perception_range;
  reference_line_->blind_dis.second = FLAGS_front_perception_range;
  if (fabs(reference_line_->mapinfo.dis2line) > 0.5) {
    return;
  }
  auto& car_info = reference_line_->car_info_;
  for (auto it = car_info.upper_bound(0.0); it != car_info.end(); ++it) {
    auto& object = reference_line_->objects_.at(it->second);
    if (object.sl_boundary.max_l > 0.0 && object.sl_boundary.min_l < 0.0) {
      double right = -object.sl_boundary.min_s * 5.0 / object.sl_boundary.min_l;
      reference_line_->blind_dis.first = std::min(reference_line_->blind_dis.first, 
          object.sl_boundary.min_s * 5.0 / object.sl_boundary.max_l);
      reference_line_->blind_dis.second = std::min(reference_line_->blind_dis.second, 
          -object.sl_boundary.min_s * 5.0 / object.sl_boundary.min_l);
    }
  }
  AINFO_IF(FLAGS_log_enable) << "blind_dis = " << reference_line_->blind_dis.first << 
      " right = " << reference_line_->blind_dis.second;
}

void SemanticProjection::GetQueueLength() {
  auto& car_info = reference_line_->car_info_;
  double front_s = FLAGS_front_perception_range;
  if (reference_line_->reference_lane_id >= 40 || car_info.empty()) {
    return;
  }
  double dis_to_solid = reference_line_->dis_to_last_lc + 
      fabs(reference_line_->mapinfo.first_lc_time) * FLAGS_min_lc_dis;
  if (reference_line_->mapinfo.distance_to_junctions.size() && 
      reference_line_->mapinfo.distance_to_junctions.front().first > 1e-3 && 
      reference_line_->mapinfo.distance_to_junctions.front().first < 100.0) {
    front_s = reference_line_->mapinfo.distance_to_junctions.front().first;
  } else if (dis_to_solid < FLAGS_front_perception_range) {
    front_s = reference_line_->mapinfo.dis_to_end;
  } else {
    return;
  }
  double front_speed = 0.0;
  for (auto it = car_info.rbegin(); it != car_info.rend(); it++)  {
    if (it->first > front_s) {
      continue;
    }
    auto& object = reference_line_->objects_.at(it->second);
    double gap_dis = front_s - object.sl_boundary.max_s;
    if (gap_dis < FLAGS_min_lc_dis || object.speed > front_speed && 
        gap_dis < (object.speed - front_speed) * 6.0|| 
        follow_objects_.count(object.id)) {
      double a = std::min(FLAGS_recommended_planning_dec, object.acc);
      double s = it->first + object.speed * object.speed / 2.0 / fabs(a);
      reference_line_->plan_length = s;          
      reference_line_->plan_follow_id = it->second;
      AINFO_IF(FLAGS_log_enable) << "follow_length = " << s << 
          " follow_id = " << reference_line_->plan_follow_id;
    } else {
      return;
    }
    front_s = it->first;
    front_speed = object.speed;
  }
}

void SemanticProjection::GetJamLevel() {
  auto& car_info = reference_line_->car_info_;
  double front_dis = FLAGS_front_perception_range;
  if (reference_line_->reference_lane_id >= 40 || car_info.size() < 2) {
    reference_line_->jam_level = 0;
    return;
  } else if (reference_line_->reference_lane_id >= 20) {
    for (auto &line : DP_->reference_line_info.current_reference_line) {
      if (line.reference_lane_id == DP_->reference_line_info.current_line_id) {
        front_dis = reference_line_->reference_lane_id < 30 ? 
            line.blind_dis.first : line.blind_dis.second;
      }
    }
  }
  const double safe_dis = 5.0;
  double max_s = reference_line_->objects_.at(car_info.rbegin()->second).sl_boundary.max_s;
  bool modify_length = max_s + FLAGS_min_lc_dis > front_dis || 
        follow_objects_.count(car_info.rbegin()->second);
  double thw = 0.0;
  int front_id = car_info.rbegin()->second;
  for (auto it = car_info.rbegin(); it != car_info.rend(); it++) {
    if (it->second == front_id) {
      continue;
    }
    if (reference_line_->reference_lane_id ==
        DP_->reference_line_info.current_line_id && it->first < 0.0) {
      break;
    }
    auto& front = reference_line_->objects_.at(front_id);
    auto& rear = reference_line_->objects_.at(it->second);
    double gap_dis = front.sl_boundary.min_s - rear.sl_boundary.max_s;
    if (gap_dis < FLAGS_min_lc_dis) {
      thw += 0.0;
    } else if (rear.speed > front.speed && gap_dis < (rear.speed - front.speed) * 6.0) {
      thw += (gap_dis - safe_dis) / rear.speed;
    } else {
      thw += 2.0 * (gap_dis - safe_dis) / (rear.speed + front.speed);
      modify_length = modify_length && follow_objects_.count(rear.id);
    }
    if (modify_length) {
      double a = std::min(FLAGS_recommended_planning_dec, rear.acc);
      double length = rear.sl_boundary.min_s + rear.speed * rear.speed / 2.0 / fabs(a);
      if (reference_line_->plan_length > length) {
        reference_line_->plan_length = length;
        reference_line_->plan_follow_id = it->second;
      }
    }
    front_id = it->second;
  }
  double aver_thw = thw / (car_info.size() - 1);
  if (aver_thw < 4.0) {
    reference_line_->jam_level = aver_thw < 2.0 ? 2 : 1;
  } else if (car_info.size() > 2) {
    reference_line_->jam_level = 1;
  }
  AINFO_IF(FLAGS_log_enable) << "aver_thw = " << aver_thw << " plan_len = " << 
      reference_line_->plan_length << " id = " << reference_line_->plan_follow_id;
}

bool SemanticProjection::IsTempCollision() {
	bool temp_collision = false;
  int block_id = -1;
	double half_car_width = 0.5 * reference_line_->car_model_.car_width;
  double dis_to_end = std::min(reference_line_->mapinfo.dis_to_end, 
      reference_line_->mapinfo.dis2missionpoint);
	reference_line_->block_dis = 100.0;
  double dis_to_junction = 1000.0;
  if (reference_line_->mapinfo.distance_to_junctions.size()) {
    dis_to_junction = reference_line_->mapinfo.distance_to_junctions.front().first;
  }
  for (auto it = reference_line_->objects_.begin(); it != reference_line_->objects_.end(); it++) {
    auto &object = it->second;
    if (object.dis_to_junction < 1.0) {
      int num = object.was_dynamic ? 2 : 1;
      if (object.static_counter < num * FLAGS_junction_static_times) {
        continue;
      }
      Id lane_id;
      lane_id.set_id(object.obj_lane_id);
      acu::hdmap::Lane::LaneType type;
      if (dis_to_junction > 1.0 && vectormap_->GetLaneType(lane_id, type) == 0 && 
          type == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT) {
        continue;
      }
    }
    if (reference_line_->reference_lane_id == 40 && 
        reference_line_->mapinfo.path_points.size()) {
      dis_to_end = std::min(reference_line_->mapinfo.dis_to_end, 
                     reference_line_->mapinfo.path_points.back().length);
    }
    if (object.sl_boundary.min_s < FLAGS_front_perception_range + 10.0 &&
        object.sl_boundary.min_s < dis_to_end + FLAGS_collision_buff && 
        object.sl_boundary.max_s > 0.0 && (object.is_static && 
        object.sl_boundary.min_l < half_car_width + FLAGS_collision_buff + 0.2 && 
        object.sl_boundary.max_l > -(half_car_width + FLAGS_collision_buff + 0.2) || 
        object.sl_polygons.size() && GetPolygonL(object) < half_car_width + 0.2)) {
      if (object.sl_boundary.min_l < half_car_width + FLAGS_collision_buff && 
          object.sl_boundary.max_l > -(half_car_width + FLAGS_collision_buff)) {
        reference_line_->temp_blocked = true;
        if (reference_line_->block_dis > object.sl_boundary.min_s) {
          reference_line_->block_dis = object.sl_boundary.min_s;
          block_id = object.id;
        }
      }
    } else {
      continue;
    }
    // parallel waiting cars
    if (follow_objects_.count(object.id)) {
      if (object.sl_boundary.min_l < half_car_width + FLAGS_collision_buff && 
          object.sl_boundary.max_l > -(half_car_width + FLAGS_collision_buff) && 
          (target_line_->block_l_.count(object.id) == 0 || 
          target_line_->block_l_[object.id] > FLAGS_collision_buff)) {
        temp_collision = true;
      }
      continue;
    }
    temp_collision = true;
  }
  if (reference_line_->temp_blocked && reference_line_->block_id == block_id) {
    reference_line_->temp_blocked = false;
  }
  
  return temp_collision;
}

double SemanticProjection::GetPolygonL(const LineObject& object) {
  double min_l = std::numeric_limits<double>::max();
  for (auto& point : object.sl_polygons.front().points()) {
    if (point.x() < 30.0 && fabs(point.y()) < min_l) {
      min_l = fabs(point.y());
    }
  }
  AWARN_IF(FLAGS_log_enable) << object.id << " min_dis_to_line = " << min_l;
  return min_l;
}

void SemanticProjection::FindFollowObjects() {
  auto& car_info = reference_line_->car_info_;
  double front_dis = 50.0, front_speed = 0.0, bound_l, bound_r;
  bool have_car_waiting = false;
  for (auto it = car_info.rbegin(); it != car_info.rend(); ++it) {
    auto& object = reference_line_->objects_[it->second];
    if (object.dis_to_junction > 10.0 && object.is_static) {
      reference_line_->GetWidthToRoadBoundary(bound_l, bound_r, object.sl_boundary.min_s, false);
      double dis_to_boundary = bound_r + object.sl_boundary.min_l;
      if (dis_to_boundary < 0.3 || 
          dis_to_boundary < 1.0 && (front_speed > 5.0 || front_dis > 6.0)) {
        AWARN_IF(FLAGS_log_enable) << "roadside_parking_id = " << object.id;
        park_objects_.insert(object.id);
        continue;
      } 
    }
    if (object.is_waiting) {
      have_car_waiting = true;
    }
    front_dis = object.sl_boundary.min_s;
    front_speed = object.speed;
  }
  if (have_car_waiting) {
    for (auto it = car_info.begin(); it != car_info.end();) {
      if (park_objects_.count(it->second)) {
        it = car_info.erase(it);
      }
      auto& object = reference_line_->objects_[it->second];
      if (object.sl_boundary.min_s > 0.0) {
        AWARN_IF(FLAGS_log_enable) << "juntion_follow_id = " << object.id;
        follow_objects_.insert(object.id);
      }
      ++it;
    }
  }

  reference_line_->follow_objects_ = follow_objects_;
  reference_line_->park_objects_ = park_objects_;
}

void SemanticProjection::AddSpeedInfo() {
	GetFrontSpeed();
  if (reference_line_->mapinfo.distance_to_junctions.size() && 
      reference_line_->mapinfo.distance_to_junctions.front().first < 0.0) {
    reference_line_->short_term_speed = FLAGS_free_speed;
    reference_line_->long_term_speed = FLAGS_free_speed;
  } else {
    reference_line_->short_term_speed *= 0.8;
    reference_line_->short_term_speed += 0.2 * reference_line_->line_speed;
    reference_line_->long_term_speed *= 0.9;
    reference_line_->long_term_speed += 0.1 * reference_line_->line_speed;
  }
  AINFO_IF(FLAGS_log_enable) << "short_term_speed = " << reference_line_->short_term_speed << 
      " long_term_speed = " << reference_line_->long_term_speed;
  reference_line_->line_slow = 
      reference_line_->short_term_speed < FLAGS_speed_threshold;
  double speed_threshold = FLAGS_speed_threshold;
  if (reference_line_->mapinfo.expected_speeds.size()) {
    speed_threshold = 0.6 * reference_line_->mapinfo.expected_speeds.front().second;
  } 
  if (jam_junction_ids_.count(reference_line_->mapinfo.junction_id) && 
      reference_line_->long_term_speed < speed_threshold) {
    reference_line_->is_congestion = true;
  } else {
    reference_line_->is_congestion = false;
  }  
}

void SemanticProjection::GetFrontSpeed() {
	reference_line_->line_speed = FLAGS_free_speed;
  reference_line_->average_v = FLAGS_free_speed;
  double buffer = reference_line_->car_model_.car_width + FLAGS_collision_buff;
  double sum = 0.0, car_sum = 0.0, front_s = FLAGS_front_perception_range;
  int num = 0, car_num = 0;
  for (auto& object : reference_line_->objects_) {
    auto& it = object.second;
    if (reference_line_->block_l_.count(it.id) == 0) {
      continue;
    }
    if (it.is_static && it.sl_boundary.max_s > 10.0 || 
        !it.is_static && it.conflict_type == 1) {
      reference_line_->line_speed = std::min(it.speed, reference_line_->line_speed);
      sum += it.speed;
      num++;
      reference_line_->average_v = sum / num;
    }
  }
}

void SemanticProjection::AddSafetyInfo() {
  bool collision_flag = false;
  reference_line_->safety_cost = 0.0;
  for (auto it = reference_line_->objects_.begin(); it != reference_line_->objects_.end(); it++) {
    auto &object = it->second;
    if (object.is_static || object.conflict_type != 1 && object.conflict_type != 3) {
      continue;
    }
    reference_line_->safety_cost += FLAGS_side_risk_range / fabs(object.x);
    if (object.sl_boundary.min_s > FLAGS_side_risk_range) {
      if (object.speed < reference_line_->car_speed_) {
        double ttc = (object.sl_boundary.min_s - FLAGS_side_risk_range) / 
            (reference_line_->car_speed_ - object.speed);
        reference_line_->safety_cost += FLAGS_risk_ttc / ttc;
        double t = reference_line_->car_speed_ - object.speed;
        double dec_s = (pow(reference_line_->car_speed_, 2) - pow(object.speed, 2)) / 2.0;
        if (object.sl_boundary.min_s + object.speed * t < dec_s + FLAGS_side_risk_range || ttc < 4.0) {
          AWARN_IF(FLAGS_log_enable) << object.id << " ttc is small!";
          collision_flag = true;
        }
      }
    } else if (object.sl_boundary.max_s < -FLAGS_side_risk_range) {
      if (object.sl_boundary.max_s > -15.0 && 
          fabs(object.speed - reference_line_->car_speed_) < 2.0) {
        AWARN_IF(FLAGS_log_enable) << object.id << " thw is small!";
        collision_flag = true;
      } 
      if (object.speed > reference_line_->car_speed_) {
        double ttc = (object.sl_boundary.max_s + FLAGS_side_risk_range) / 
            (reference_line_->car_speed_ - object.speed);
        reference_line_->safety_cost += FLAGS_risk_ttc / ttc;
        if (ttc < FLAGS_risk_ttc) {
          AWARN_IF(FLAGS_log_enable) << object.id << " ttc is small!";
          collision_flag = true;
        }
      }
    } else {
      reference_line_->safety_cost += 5.0;
      AWARN_IF(FLAGS_log_enable) << object.id << " dis is small!";
      collision_flag = true;
    }
  }
  if (collision_flag) {
    AWARN_IF(FLAGS_log_enable) << "safety_cost = " << reference_line_->safety_cost;
    reference_line_->line_safety = false;
    reference_line_->safety_counter = 0;
  } else if (reference_line_->safety_counter > FLAGS_safety_times) {
    reference_line_->line_safety = true;
  } else {
    reference_line_->safety_counter++;
  }
}

void SemanticProjection::AddSpeedCost(int direction) { 
  double cost = 0.0, speed_threshold = FLAGS_speed_threshold;
  if (reference_line_->mapinfo.expected_speeds.size()) {
    if (jam_junction_ids_.count(reference_line_->mapinfo.junction_id)) {
      speed_threshold = 0.7 * reference_line_->mapinfo.expected_speeds.front().second;
    } else {
      speed_threshold = 0.6 * reference_line_->mapinfo.expected_speeds.front().second;
    }
  }
  auto DP = DataPool::Instance()->GetMainDataPtr();
  double dis_to_junction = 1000.0;
  if (reference_line_->mapinfo.distance_to_junctions.size()) {
    dis_to_junction = reference_line_->mapinfo.distance_to_junctions.front().first;
  }
  for (auto& block : reference_line_->block_l_) {
    if (!reference_line_->objects_.count(block.first)) continue;
    auto& object = reference_line_->objects_.at(block.first);
    double l_cost = pow(1.0 - block.second, 2);
    l_cost *= std::min(std::max(object.dis_to_junction / 180.0, 0.4), 1.0);
    // AERROR << "dis_to_junction = " << object.dis_to_junction << " l_cost = " << l_cost;
    if (object.sl_boundary.min_s > FLAGS_front_perception_range) {
      continue;
    } else if (object.is_static) {
      if (object.sl_boundary.max_s < 10.0) {
        continue;
      }
      cost = std::max(cost, l_cost * 5.0);
    } else if (object.conflict_type == 1) {
      double v_cost = speed_threshold - object.speed;
      if (object.speed < speed_threshold) {
        v_cost = sqrt(v_cost);
      }
      cost = std::max(cost, l_cost * std::min(pow(1.5, v_cost), 5.0));
      if (object.speed < 0.5 * speed_threshold && !follow_objects_.count(object.id) && 
          reference_line_->car_info_.size() < 2 && 
          reference_line_->car_on_line_.count(object.id) && 
          reference_line_->car_on_line_[object.id] > 50) {
        double v = reference_line_->mapinfo.expected_speeds.size() ? 
            reference_line_->mapinfo.expected_speeds.front().second : 
            DP->config_info.speedplan_config.maximum_cruising_speed;
        double overtake_dis = object.sl_boundary.min_s * v / (v - object.speed);
        if (overtake_dis + FLAGS_front_perception_range < dis_to_junction) {
          cost += 300.0 / dis_to_junction;
          AWARN_IF(FLAGS_log_enable) << "overtake_dis = " << overtake_dis;
        }
      }
    }
  }
  reference_line_->speed_cost *= 0.96;
  reference_line_->speed_cost += 0.04 * cost;
  if (dis_to_junction < 0.0) {
    reference_line_->speed_cost = 0.0;
  }
  AERROR_IF(FLAGS_log_enable) << "temp_cost = " << cost << " speed_cost = " << 
      reference_line_->speed_cost << " speed_threshold = " << speed_threshold;
  if (reference_line_->mapinfo.rightside_length > 0.0) {
    if (reference_line_->mapinfo.distance_to_junctions.size() && 
        reference_line_->mapinfo.distance_to_junctions.front().first > 150.0) {
      double dis = reference_line_->mapinfo.distance_to_junctions.front().first;
      reference_line_->side_cost += std::min(dis / 150.0, 1.5);
    } else {
      reference_line_->side_cost += 1.0;
    }
    if (reference_line_->mapinfo.junction_id == "225746464" || 
        jam_junction_ids_.count(reference_line_->mapinfo.junction_id)) {
      reference_line_->side_cost += 1.0;
    } 
    if (reference_line_->mapinfo.junction_id == "187116559" ||
        reference_line_->mapinfo.front_lane_ids.size() && 
        (reference_line_->mapinfo.front_lane_ids.at(0) == "217377566_1_-2" || 
        reference_line_->mapinfo.front_lane_ids.at(0) == "187225193_1_-2"  || 
        reference_line_->mapinfo.front_lane_ids.at(0) == "241360261_1_-3" )) {
      reference_line_->side_cost = 0.0;
    }
    reference_line_->side_cost = 0.0;
  } else if (reference_line_->mapinfo.dis_to_merge < 1000.0) {
    int lc_time = 0;
    if (DP_->reference_line_info.right_reference_line.size() && 
        DP_->reference_line_info.right_reference_line.back().mapinfo.rightside_length > 0.0) {
      lc_time = DP_->reference_line_info.right_reference_line.back().mapinfo.all_lc_time;
    } else if (DP_->reference_line_info.current_reference_line.size() && 
        DP_->reference_line_info.current_reference_line.back().mapinfo.rightside_length > 0.0) {
      lc_time = DP_->reference_line_info.current_reference_line.back().mapinfo.all_lc_time;
    }
    if (reference_line_->mapinfo.all_lc_time == lc_time) {
      for (auto& turn : reference_line_->mapinfo.lane_turns) {
        if (turn.first == reference_line_->mapinfo.dis_to_merge && turn.second == 6) {
          //reference_line_->side_cost += 1.0;
          break;
        }
      } 
    }
  }
}

void SemanticProjection::AddReverseInfo() {
  bool meeting_flag = false;
  reference_line_->meeting_state = 0;
  reference_line_->meeting_objects_.clear();
  if (DP_->reference_line_info.reverse_reference_line.reference_lane_id && 
      DP_->reference_line_info.left_reference_line.empty() && 
      reference_line_->mapinfo.dis_to_end > 80.0 && 
      reference_line_->mapinfo.distance_to_junctions.size() && 
      reference_line_->mapinfo.distance_to_junctions.front().first > 60.0) {
    double min_dis = 1.0, max_dis = 1.0;
    for (auto it = reference_line_->objects_.begin(); it != reference_line_->objects_.end(); it++) {
      auto &object = it->second;
      if (DataPool::Instance()->GetMainDataRef().decision_info.meeting_ids.count(object.id)) {
        object.vsabs -= 1.0;
      }
      if (object.sl_boundary.max_s > 0.0 && object.type < 2 && !object.is_static && 
          object.sl_boundary.max_l > -1.5 && object.vsabs < -1.5) {
        double dis = object.sl_boundary.min_l - 0.5 * reference_line_->car_model_.car_width;
        if (dis < min_dis) {
          meeting_flag = true;
          min_dis = dis;
          double left_w, right_w;
          reference_line_->GetWidthToRoadBoundary(left_w, right_w);
          max_dis = object.sl_boundary.min_l + right_w - reference_line_->car_model_.car_width;
          if (dis < 0.6 && object.sl_boundary.min_s < 60.0) {
            reference_line_->meeting_objects_.push_back(object.id);
          }
          AERROR_IF(FLAGS_log_enable) << "meeting_id = " << object.id << 
              " dis = " << dis << " max_dis = " << max_dis;
        }
      }
    }
    if (min_dis < 0.4) {
      reference_line_->meeting_state = 2;
    } else if (min_dis < 0.8) {
      reference_line_->meeting_state = 1;
    } 
  } 
  if (reference_line_->meeting_car == meeting_flag) {
    reference_line_->meeting_counter = 0;
  } else if (reference_line_->meeting_counter > FLAGS_meeting_times) {
    reference_line_->meeting_car = meeting_flag;
    reference_line_->meeting_counter = 0;
  } else {
    reference_line_->meeting_counter++;
  } 
}

void SemanticProjection::AddConflictInfo() {
  auto& junction_s = reference_line_->mapinfo.distance_to_junctions;
  if (junction_s.empty() || junction_s.front().first > 30.0) {
    return;
  }
  double total_s = -reference_line_->mapinfo.first_lane_start_s;
  for (auto lane_id : reference_line_->mapinfo.front_lane_ids) {
    LaneInfoConstPtr lane = vectormap_->GetLaneById(lane_id);
    if (lane == nullptr) continue;
    total_s += lane->total_length();
    AWARN_IF(FLAGS_log_enable) << "total_s = " << total_s << " end_s = " << 
        junction_s.front().second;
    if (total_s > junction_s.front().second + 5.0) {
      return;
    } 
    hdmap::Lane::LaneTurn turn_type;
    vectormap_->GetLaneTurn(lane->id(), turn_type);
    if (int(turn_type) < 3) {
      continue;
    }
    AWARN_IF(FLAGS_log_enable) << "junction lane id = " << lane_id;
    // find conflict road
    std::vector<Id> next_road, pre_road;
    vectormap_->GetRoadSuccessorIDs(lane->road_id(), next_road);
    if (next_road.size() != 1) {
      AWARN_IF(FLAGS_log_enable) << lane_id << "can't find conflict zone";
      return;
    }
    std::set<string> conflict_road;
    vectormap_->GetRoadPredecessorIDs(next_road.front(), pre_road);
    for (auto& road : pre_road) {
      if (road.id() != lane->road_id().id()) {
        conflict_road.insert(road.id());
      }
    }
    // find conflict s
    for (int i = 0; i < (int)lane->total_length(); i++) {
      PointENU point = lane->GetSmoothPoint((double)i);
      std::vector<LaneInfoConstPtr> lanes;
      vectormap_->GetLanes(point, 1.0, lanes);
      for (auto& conflict_lane : lanes) {
        if (conflict_road.count(conflict_lane->road_id().id())) {
          reference_line_->conflict_s = i + total_s - lane->total_length();
          AINFO_IF(FLAGS_log_enable)<< "conflict with " << conflict_lane->id().id() << " at " << i;
          return;
        }
      }     
    }
  }
}

void SemanticProjection::AddQueueInfo() {
  auto& mapinfo = reference_line_->mapinfo;
  double dis_to_junction = 1000.0;
  if (mapinfo.distance_to_junctions.size()) {
    dis_to_junction = mapinfo.distance_to_junctions.front().first;
  }
  // AINFO << " light_s = " << mapinfo.trafficlight.light_s << " dis_to_junction = " << 
  //     dis_to_junction << " light_id = " << mapinfo.trafficlight.light_id;
  // if (mapinfo.trafficlight.history_light.size()) {
  //   AINFO << "color = " << mapinfo.trafficlight.history_light.back().color << 
  //       " time = " << mapinfo.trafficlight.history_light.back().time;
  // }
  // No traffic light nearby or light is green for some time
  if (mapinfo.trafficlight.light_id == "" || mapinfo.trafficlight.light_s < 1.0 || 
      mapinfo.trafficlight.light_s > std::min(dis_to_junction + 1.0, 150.0) || 
      mapinfo.trafficlight.history_light.size() && 
      mapinfo.trafficlight.history_light.back().color == 3 && 
      mapinfo.trafficlight.history_light.back().time > 5.0 && 
      reference_line_->jam_level == 0) {
    reference_line_->line_queue = false;
    return;
  }
  // In solid zone of junction
  if (mapinfo.left_bd_types.size() && mapinfo.left_bd_types.front().second > 2 && 
      mapinfo.right_bd_types.size() && mapinfo.right_bd_types.front().second > 2) { 
    reference_line_->line_queue = true;
    return;
  }
  // Exist objects waiting in line
  for (auto& object : reference_line_->objects_) { 
    auto& it = object.second;
    if (it.sl_boundary.min_s < mapinfo.trafficlight.light_s && 
        follow_objects_.count(it.id)) {
      reference_line_->line_queue = true;
      AINFO_IF(FLAGS_log_enable)<< "waiting_object = " << it.id << " conflict = " << it.conflict_type;
      return;
    }
  }
  reference_line_->line_queue = false;
}

void SemanticProjection::LineLimitSpace() {
  reference_line_->limit_space.clear();
  if (reference_line_->objects_.empty()) {
    return;
  }
  vector<ObstacleEdge> sorted_objects;
  SortObjects(sorted_objects);
  auto &line_boundary = reference_line_->limit_space;
  InitBoundary(sorted_objects, line_boundary);
  int block_object_id = -1;
  CalculateBoundary(sorted_objects, line_boundary, block_object_id);
  if (block_object_id >= 0) {
    reference_line_->block_id = block_object_id;
  }
}

void SemanticProjection::SortObjects(vector<ObstacleEdge>& sorted_objects) {
  sorted_objects.clear();
  double dis_to_end = std::min(reference_line_->mapinfo.dis_to_end, 
                        reference_line_->mapinfo.dis2missionpoint);
  for (auto &it : reference_line_->objects_) {
    auto &object = it.second;
    if (object.sl_boundary.min_s > dis_to_end ||
        object.sl_boundary.min_l > FLAGS_lane_width ||
        object.sl_boundary.max_l < -FLAGS_lane_width) {
      continue;
    }
    if (object.sl_boundary.max_s < 0.0 || !object.is_static) {
      continue;
    }
    sorted_objects.emplace_back(1, object.sl_boundary.min_s - FLAGS_front_buff,
        object.sl_boundary.min_l, object.sl_boundary.max_l, object.id);
    sorted_objects.emplace_back(0, object.sl_boundary.max_s + FLAGS_front_buff,
        object.sl_boundary.min_l, object.sl_boundary.max_l, object.id);
  }
  std::sort(sorted_objects.begin(), sorted_objects.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });
  // AERROR<<"SortObjects-------------";
  // for (auto &obj : sorted_objects) {
  //   AERROR<<"sorted obj "<<std::get<4>(obj)<<" start "<<std::get<0>(obj)
  //         <<" s "<<std::get<1>(obj)<<" min l "<<std::get<2>(obj)<<" max l "<<std::get<3>(obj);
  // }
}

void SemanticProjection::InitBoundary(const vector<ObstacleEdge>& sorted_objects, LaneBound& line_boundary) {
  line_boundary.clear();
  line_boundary.reserve(sorted_objects.size());
  for (int i = 0; i < sorted_objects.size(); i++) {
    LaneBoundPoint temp_width;
    double s = std::get<1>(sorted_objects.at(i));
    std::get<0>(temp_width) = s;
    double left_w, right_w;
    if (!reference_line_->GetWidthToLaneBoundary(left_w, right_w, s)) {
      std::get<1>(temp_width) = -FLAGS_lane_width/2.0;
      std::get<2>(temp_width) = FLAGS_lane_width/2.0;
      line_boundary.push_back(temp_width);
      continue;
    }
    std::get<1>(temp_width) = -right_w;
    std::get<2>(temp_width) = left_w;
    line_boundary.push_back(temp_width);
  }
  // AERROR<<"InitBoundary-------------";
  // for (auto &space : line_boundary) {
  //   AERROR<<"init s "<<std::get<0>(space)<<" min l "<<std::get<1>(space)<<" max l "<<std::get<2>(space);
  // }
}

void SemanticProjection::CalculateBoundary(const vector<ObstacleEdge>& sorted_objects,
                                      LaneBound& line_boundary, int &block_obejct_id) 
{
  double center_line = 0.0;
  int obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;//从大到小排序
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;//默认从小到大排序
  left_bounds.insert(std::numeric_limits<double>::max());
  // pass from left, then true; otherwise, false.
  std::unordered_map<int, bool> obs_id_to_direction;

  // Step through every path point.
  for (int i = 0; i < line_boundary.size(); i++) {
    double curr_s = std::get<0>(line_boundary[i]);
    while (obs_idx < sorted_objects.size() && std::get<1>(sorted_objects[obs_idx]) <= curr_s) {
      const auto& curr_obstacle = sorted_objects[obs_idx];
      int is_start_s = std::get<0>(curr_obstacle);
      double curr_obstacle_l_min = std::get<2>(curr_obstacle);
      double curr_obstacle_l_max = std::get<3>(curr_obstacle);
      int curr_obstacle_id = std::get<4>(curr_obstacle);
      if (is_start_s == 1) {
        if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
          // Obstacle is to the right of center-line, should pass from left.
          AINFO_IF(FLAGS_log_enable)<<"should pass from left s = "<<curr_s;
          obs_id_to_direction[curr_obstacle_id] = true;
          right_bounds.insert(curr_obstacle_l_max);
          if (!UpdateBoundaryAndCenterLine(i, *left_bounds.begin(), 
              *right_bounds.begin(), line_boundary, center_line)) {// 当前车道被阻塞，后续位置不再更新
            path_blocked_idx = i;
            block_obejct_id = curr_obstacle_id;
            break;
          }
        }
        else {
          // Obstacle is to the left of center-line, should pass from right.
          AINFO_IF(FLAGS_log_enable)<<"should pass from right";
          obs_id_to_direction[curr_obstacle_id] = false;
          left_bounds.insert(curr_obstacle_l_min);
          if (!UpdateBoundaryAndCenterLine(i, *left_bounds.begin(), 
              *right_bounds.begin(), line_boundary, center_line)) {
              path_blocked_idx = i;
              block_obejct_id = curr_obstacle_id;
              break;
            }
        }
      }
      else {// 不是start_s（min_s）
        // An existing obstacle exits our scope.
        if (obs_id_to_direction[curr_obstacle_id]) {
          right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
        } 
        else {
          left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
        }
        obs_id_to_direction.erase(curr_obstacle_id);
      }
      std::get<1>(line_boundary[i]) = fmax(std::get<1>(line_boundary[i]), *right_bounds.begin());
      std::get<2>(line_boundary[i]) = fmin(std::get<2>(line_boundary[i]), *left_bounds.begin());
      if (std::get<2>(line_boundary[i]) - std::get<1>(line_boundary[i]) < 
          reference_line_->car_model_.car_width + FLAGS_collision_buff) {
        AINFO_IF(FLAGS_log_enable) << "Path is blocked at s = " << curr_s;
        path_blocked_idx = i;
        if (!obs_id_to_direction.empty()) {
          block_obejct_id = obs_id_to_direction.begin()->first;
        }
        break;
      } 
      center_line = (std::get<1>(line_boundary[i]) + std::get<2>(line_boundary[i])) / 2.0;
      ++obs_idx;
    }
    // Early exit if path is blocked.
    if (path_blocked_idx != -1) break;
  }

  if (path_blocked_idx != -1) {
    int range = path_blocked_idx + 1;
    line_boundary.erase(line_boundary.begin() + range, line_boundary.end());
  }
  for (int i = 1; i < line_boundary.size(); i++) {
    if (fabs(std::get<1>(line_boundary.at(i)) - std::get<1>(line_boundary.at(i-1))) < 0.1 &&
        fabs(std::get<2>(line_boundary.at(i)) - std::get<2>(line_boundary.at(i-1))) < 0.1) {
      std::get<0>(line_boundary.at(i-1)) = std::get<0>(line_boundary.at(i));
      std::get<1>(line_boundary.at(i-1)) = fmax(std::get<1>(line_boundary.at(i-1)), 
                                                std::get<1>(line_boundary.at(i)));
      std::get<2>(line_boundary.at(i-1)) = fmin(std::get<2>(line_boundary.at(i-1)), 
                                                std::get<2>(line_boundary.at(i)));
      line_boundary.erase(line_boundary.begin() + i);
      i--;
    }
  }
}

bool SemanticProjection::UpdateBoundaryAndCenterLine(int idx, const double &left_w, const double &right_w,
                                        LaneBound& line_boundary, double &center_line) 
{
  // Update the right bound (l_min):
  double new_l_min = fmax(std::get<1>(line_boundary[idx]), right_w);
  // Update the left bound (l_max):
  double new_l_max = fmin(std::get<2>(line_boundary[idx]), left_w);
  std::get<1>(line_boundary[idx]) = new_l_min;
  std::get<2>(line_boundary[idx]) = new_l_max;
  double buff = 0.0;
  if (new_l_min >= right_w - 0.01) {
    buff += FLAGS_collision_buff;
  }
  if (new_l_max <= left_w + 0.01) {
    buff += FLAGS_collision_buff;
  }
  if (new_l_max - new_l_min < reference_line_->car_model_.car_width + buff) {
    AINFO_IF(FLAGS_log_enable) << "Line is blocked at idx = " << idx;
    return false;
  }
  center_line = (new_l_min + new_l_max) / 2.0;
  return true;
}



}
}
