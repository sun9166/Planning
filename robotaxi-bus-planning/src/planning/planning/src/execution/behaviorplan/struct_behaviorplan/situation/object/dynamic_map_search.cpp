#include "dynamic_map_search.h"

namespace acu {
namespace planning {

DynamicMapSearch::DynamicMapSearch() {}

DynamicMapSearch::~DynamicMapSearch() {}

bool DynamicMapSearch::GetSearchResult() {
  if (context_->trajectory_info_.path.empty()) {
    return true;
  }
  // get target line
  auto reference_line = &context_->cognition_info_->reference_line_info;
  FiltSpecialObjects();
  if (context_->replan_state_ == eReplanStateEnum::EXCUTE && 
      context_->best_option_.type > 2) {
    target_line_ = context_->best_option_.line_info;
    FiltCrossObjects(false);
  } else {
    target_line_ = &reference_line->local_reference_line;
    FiltCrossObjects(true);
  }  
  path_length_ = std::min(target_line_->mapinfo.dis_to_end, context_->stop_s_);
  ExpandST();
  if (GenerateTrajectory(true) || GenerateTrajectory(false)) {
    GetFinalGoal();
    GetFinalDecision();
    context_->decision_result_.object_decision_success = 1;
  } else {
    RecoverST();
    if (GenerateTrajectory(false)) {
      GetFinalGoal();
      GetFinalDecision();
      context_->decision_result_.object_decision_success = 0;
    } else {
      context_->decision_result_.object_decision_success = -1;
    } 
  }
  return context_->decision_result_.object_decision_success >= 0;
}

void DynamicMapSearch::FiltSpecialObjects() {
  FindParaObjects();
  follow_objects_.clear();
  special_objects_.clear();
  auto line_id = context_->decision_result_.reference_line_id;
  if (context_->reference_line_map_.count(line_id)) {
    for (auto it = context_->reference_line_map_[line_id]->objects_.begin(); 
              it != context_->reference_line_map_[line_id]->objects_.end(); it++) {
      auto &object = it->second;
      if (object.conflict_type > 0 && object.conflict_type < 5 && 
          object.sl_boundary.min_s > 0.0) {
        follow_objects_.insert(object.id);
      }
      if (context_->dis_to_junction_ > 10.0 && object.sl_boundary.max_s < 0.0) {
        FiltRearObjects(object, 1.5);
      }
    }
  }
  line_id = context_->cognition_info_->reference_line_info.current_line_id;
  if (context_->reference_line_map_.count(line_id)) {
    for (auto it = context_->reference_line_map_[line_id]->objects_.begin(); 
              it != context_->reference_line_map_[line_id]->objects_.end(); it++) {
      auto &object = it->second;
      if (object.conflict_type > 0 && object.conflict_type < 5 && 
          object.sl_boundary.min_s > 0.0) {
        follow_objects_.insert(object.id);
      }
      if (object.conflict_type == 1 && object.sl_boundary.min_s < 0.0) {
        special_objects_.insert(object.id);
      }
      if (object.conflict_type != 2 && object.sl_boundary.max_s < 0.0 && 
          object.dis_to_junction > 0.0) {
        AWARN_IF(FLAGS_log_enable) << "Ignore rear object " << object.id;
        special_objects_.insert(object.id);
      }
      if (context_->dis_to_junction_ < 10.0) {
        FiltRearObjects(object, 3.0);
      }
      if ((object.conflict_type == 0 || object.conflict_type == 5) && 
          object.sl_boundary.max_s < 0.0) {
        FiltParaObjects(object);
      }      
    }
  }
}

void DynamicMapSearch::FindParaObjects() {
  para_objects_.clear();
  conflict_objects_.clear();
  auto reference_line = &context_->cognition_info_->reference_line_info;
  auto front_over_hang = context_->planning_config_.car_model.front_over_hang;
  double half_width = 0.5 * context_->planning_config_.car_model.car_width;
  if (reference_line->left_reference_line.size()) {
    auto line = &reference_line->left_reference_line.back();
	for (auto it = line->objects_.begin(); it != line->objects_.end(); it++) {
	  auto &object = it->second;
      if (object.conflict_type == 1 && object.sl_boundary.max_s > front_over_hang) {
        para_objects_.insert(object.id);
      }
    }
  }
  if (reference_line->right_reference_line.size()) {
    auto line = &reference_line->right_reference_line.front();
    for (auto it = line->objects_.begin(); it != line->objects_.end(); it++) {
	  auto &object = it->second;
      if (object.conflict_type == 1 && object.sl_boundary.max_s > front_over_hang) {
        para_objects_.insert(object.id);
      }
    }
  }
  for (auto it = reference_line->local_reference_line.objects_.begin(); 
  			it != reference_line->local_reference_line.objects_.end(); it++) {
	  auto &object = it->second;
    if (object.type > 1 || object.is_static || !para_objects_.count(object.id)) {
      continue;
    }
    if (object.st_boundary.s_t.size() && 
        object.st_boundary.s_t.front().first.x() < FLAGS_collision_buff || 
        object.sl_boundary.min_s < front_over_hang + FLAGS_collision_buff && 
        object.sl_boundary.min_l < FLAGS_collision_buff + half_width && 
        object.sl_boundary.max_l > -FLAGS_collision_buff - half_width) {
      conflict_objects_.insert(object.id);
      AINFO_IF(FLAGS_log_enable) << "conflict_objects = " << object.id;
    }
  }
}

void DynamicMapSearch::FiltParaObjects(LineObject& object) {
  auto reference_line = &context_->cognition_info_->reference_line_info;
  if (context_->decision_result_.reference_line_id >= 20) {
    return;
  }
  for (auto& line : reference_line->left_reference_line) {
    for (auto it = line.objects_.begin(); it != line.objects_.end(); it++) {
      auto &obj = it->second;
      if (obj.id == object.id && (obj.conflict_type == 1 || obj.conflict_type == 3)) { 
        AWARN_IF(FLAGS_log_enable) << "Parallel with " << object.id;
        special_objects_.insert(object.id);
        return;
      }
    }
  }
  for (auto& line : reference_line->right_reference_line) {
    for (auto it = line.objects_.begin(); it != line.objects_.end(); it++) {
      auto &obj = it->second;
      if (obj.id == object.id && (obj.conflict_type == 1 || obj.conflict_type == 3)) { 
        AWARN_IF(FLAGS_log_enable) << "Parallel with " << object.id;
        special_objects_.insert(object.id);
        return;
      }
    }
  }
}

void DynamicMapSearch::FiltRearObjects(LineObject& object, const double ttc) {
  if (object.st_boundary.s_t.empty()) {
    return;
  } 
  double min_s = object.st_boundary.s_t.front().first.x();
  double min_t = object.st_boundary.s_t.front().first.y();
  double max_s = object.st_boundary.s_t.back().first.x();
  double conflict_s = std::min(min_s, max_s);
  if (conflict_s < pow(context_->ego_speed_, 2) / 3.0 + FLAGS_min_yield_dis && 
      min_t > std::max(conflict_s, 0.0) / (context_->ego_speed_ + 0.01) + ttc) {
    special_objects_.insert(object.id);
    AWARN_IF(FLAGS_log_enable) << "Can't overtake " << object.id;
  } else if (conflict_s < FLAGS_collision_buff) {
    special_objects_.insert(object.id);
    AWARN_IF(FLAGS_log_enable) << "Blocked by " << object.id;
  } 
}

void DynamicMapSearch::FiltCrossObjects(const bool use_local) {
  for (auto it = target_line_->objects_.begin(); it != target_line_->objects_.end(); it++) {
    auto &object = it->second;
    if (context_->decision_result_.reference_line_id < 20) {
      FiltRearObjects(object, 3.0);
    }
    AWARN_IF(FLAGS_log_enable) << "local " << object.id << " static = " << 
        object.is_static << " speed = " << object.speed << " type = " << 
        object.type << " st_size = " << object.st_boundary.s_t.size();
    if (special_objects_.count(object.id)) {
      context_->decision_result_.object_decision[object.id] = eObjectDecisionEnum::IGNORE;
      continue;
    }
    if (object.is_static || object.st_boundary.s_t.empty()) {
      continue;
    }
    AWARN_IF(FLAGS_log_enable) << "target_id = " << object.id << " conflict_type = " << 
        object.conflict_type << " right = " << object.right_of_way;
    if (!use_local && !ModifyST(true, object)) {
      continue;
    }
    if (object.right_of_way > 0) {
      prior_objects_.push_back(object);
    } 
    objects_.push_back(object);
    // For display
    planning_debug_msgs::STGraph graph;
    graph.set_id(object.id);
    graph.set_right_of_way(object.right_of_way > 0);
    graph.set_conflict_type(object.conflict_type);
    graph.set_t_1(object.st_boundary.s_t.front().first.y());
    graph.set_t_2(object.st_boundary.s_t.back().first.y());
    graph.set_min_s_1(object.st_boundary.s_t.front().first.x());
    graph.set_min_s_2(object.st_boundary.s_t.back().first.x());
    graph.set_max_s_1(object.st_boundary.s_t.front().second.x());
    graph.set_max_s_2(object.st_boundary.s_t.back().second.x());
    context_->decision_result_.st_graph.add_cognition_objects()->CopyFrom(graph);
    AINFO<<"t_1 = " << graph.t_1() << " t_2 = " << graph.t_2() << 
        " s_1 = " << graph.min_s_1() << " s_2 = " << graph.min_s_2();
  }
  if (use_local) {
    return;
  } 
  auto line_id = context_->cognition_info_->reference_line_info.current_line_id;
  for (auto it = context_->reference_line_map_[line_id]->objects_.begin(); 
              it != context_->reference_line_map_[line_id]->objects_.end(); it++) {
    auto &object = it->second;
    AWARN_IF(FLAGS_log_enable)<< "current_id = " << object.id << " static = " << 
        object.is_static << " type = " << object.type << " conflict_type = " << 
        object.conflict_type << " right = " << object.right_of_way;
    if (special_objects_.count(object.id)) {
      context_->decision_result_.object_decision[object.id] = eObjectDecisionEnum::IGNORE;
      continue;
    }
    if (object.is_static || object.st_boundary.s_t.empty()) {
      continue;
    }
    if (!ModifyST(false, object)) {
      continue;
    }
    if (object.right_of_way > 0) {
      prior_objects_.push_back(object);
    } 
    objects_.push_back(object);
  }
}

bool DynamicMapSearch::ModifyST(const bool is_target, LineObject& object) {
  double lc_time = 3.0;
  if (is_target && object.st_boundary.s_t.back().first.y() < lc_time || 
      !is_target && object.st_boundary.s_t.front().first.y() > lc_time) {
    return false;
  }
  double min_t = object.st_boundary.s_t.front().first.y();
  double max_t = object.st_boundary.s_t.back().first.y();
  double min_s_1 = object.st_boundary.s_t.front().first.x();
  double min_s_2 = object.st_boundary.s_t.back().first.x();
  double max_s_1 = object.st_boundary.s_t.front().second.x();
  double max_s_2 = object.st_boundary.s_t.back().second.x();
  if (is_target && min_t < lc_time) {
    double offset = (min_s_2 - min_s_1) / (max_t - min_t) * (lc_time - min_t);
    object.st_boundary.s_t.front().first = 
        common::math::Vec2d(min_s_1 + offset, lc_time);
    object.st_boundary.s_t.front().second = 
        common::math::Vec2d(max_s_1 + offset, lc_time);
  }
  if (!is_target && max_t > lc_time) {
    double offset = (min_s_2 - min_s_1) / (max_t - min_t) * (lc_time - min_t);
    object.st_boundary.s_t.back().first = 
        common::math::Vec2d(min_s_2 + offset, lc_time);
    object.st_boundary.s_t.back().second = 
        common::math::Vec2d(max_s_2 + offset, lc_time);
  }
  return true;
}

void DynamicMapSearch::ExpandST() {
  for (auto& object : objects_) {
    auto& s_t = object.st_boundary.s_t;
    double dis_below = FLAGS_min_yield_dis;
    // double dis_below = max(FLAGS_min_yield_dis, context_->ego_speed_);
    double dis_upper = FLAGS_min_yield_dis + s_t.back().first.y() - s_t.front().first.y();
    if (follow_objects_.count(object.id)) {
      dis_below = 0.0;
    }
    for (int i = 0; i < s_t.size(); i++) {
      s_t[i].first = common::math::Vec2d(
          s_t[i].first.x() - dis_below, s_t[i].first.y());
      s_t[i].second = common::math::Vec2d(
          s_t[i].second.x() + dis_upper, s_t[i].second.y());
    }

    // s_t.front().first = common::math::Vec2d(
    //     s_t.front().first.x() - dis_below, s_t.front().first.y());
    // s_t.back().first = common::math::Vec2d(
    //     s_t.back().first.x() - dis_below, s_t.back().first.y());
    // s_t.front().second = common::math::Vec2d(
    //     s_t.front().second.x() + dis_upper, s_t.front().second.y());
    // s_t.back().second = common::math::Vec2d(
    //     s_t.back().second.x() + dis_upper, s_t.back().second.y());
    // AWARN_IF(FLAGS_log_enable) << object.id << " expand = (" << 
    //     s_t.front().first.y() << ", " << s_t.front().first.x() << ", " << 
    //     s_t.front().second.x() << ") (" << s_t.back().first.y() << ", " << 
    //     s_t.back().first.x() << ", " << s_t.back().second.x() << ")";
  }
}

void DynamicMapSearch::RecoverST() {
  for (auto& object : objects_) {
    auto& s_t = object.st_boundary.s_t;
    double dis_upper = s_t.front().first.y() - s_t.back().first.y();
    for (int i = 0; i < s_t.size(); i++) {
      s_t[i].second = common::math::Vec2d(
          s_t[i].second.x() + dis_upper, s_t[i].second.y());
    }
    // s_t.front().second = common::math::Vec2d(
    //     s_t.front().second.x() + dis_upper, s_t.front().second.y());
    // s_t.back().second = common::math::Vec2d(
    //     s_t.back().second.x() + dis_upper, s_t.back().second.y());
    AWARN_IF(FLAGS_log_enable) << object.id << " recover = (" << 
        s_t.front().first.y() << ", " << s_t.front().first.x() << ", " << 
        s_t.front().second.x() << ") (" << s_t.back().first.y() << ", " << 
        s_t.back().first.x() << ", " << s_t.back().second.x() << ")";
  }
}

bool DynamicMapSearch::GenerateTrajectory(const bool is_comfort) {
  if (target_line_->mapinfo.expected_speeds.empty()) {
    AWARN_IF(FLAGS_log_enable) << "Expected speed empty!";
    return false;
  }
  if (is_comfort) {
    max_dec_ = FLAGS_recommended_planning_dec;
    max_acc_ = FLAGS_recommended_planning_acc;
  } else {
    AWARN_IF(FLAGS_log_enable) << "Make decision of larger acc or space!";
    max_dec_ = FLAGS_max_planning_dec;
    max_acc_ = FLAGS_max_planning_acc;
  }
  std::map<int, SpeedNodeStruct> last_level;
  SpeedNodeStruct node;
  node.s = 0;
  node.v = context_->ego_speed_;
  last_level[0] = node;
  points_.clear();
  points_.push_back(last_level);
  for (int t = 1; t < context_->speedplan_config_.matrix_dimension_t(); t++) {
    std::map<int, SpeedNodeStruct> level;
    double min_s = last_level.begin()->first + last_level.begin()->second.v + max_dec_;
    double max_s = last_level.rbegin()->first + last_level.rbegin()->second.v + max_acc_;
    int start_s = std::max((int)min_s, 0);
    int end_s = std::min((int)max_s, (int)path_length_);
    for (int s = start_s; s <= end_s; s++) {
      node.s = s;
      for (auto it = last_level.rbegin(); it != last_level.rend(); it++) {
        double acc = s - it->second.s - it->second.v;
        if (s < it->second.s || acc < max_dec_ || acc > max_acc_) {
          continue;
        }
        if (SatisfyConstrain(t, s, it->second.s) == false) {
          continue;
        }
        node.last_s = it->second.s;
        node.v = (double)(s - it->second.s);
        double speed_limit = 0.0;
        for (auto& speed : target_line_->mapinfo.expected_speeds) {
          if (speed.first > node.s) {
            speed_limit = speed.second;
            break;
          }
        }
        if (node.v > speed_limit) {
          continue;
        }
        level[s] = node;
        break;
      }
    }
    points_.push_back(level);
    last_level = level;
    if (level.empty()) {
      return false;
    }
  }
  return true;
}

bool DynamicMapSearch::SatisfyConstrain(const int t, const int s, const int last_s) {
  double s_2 = (double)s;
  double t_2 = (double)t;
  double s_1 = (double)last_s;
  double t_1 = (double)(t - 1);
  double k_1 = (s_2 - s_1) / (t_2 - t_1);
  double b_1 = s_1 - k_1 * t_1;
  for (auto& object : objects_) {
    for (int i = 1; i < object.st_boundary.s_t.size(); i++) {
      double t_3 = object.st_boundary.s_t[i - 1].first.y();
      double t_4 = object.st_boundary.s_t[i].first.y();
      if (t_1 > t_4 || t_2 < t_3) {
        continue;
      }
      double s_3 = object.st_boundary.s_t[i - 1].first.x();
      double s_4 = object.st_boundary.s_t[i].first.x();
      double s_5 = object.st_boundary.s_t[i - 1].second.x();
      double s_6 = object.st_boundary.s_t[i].second.x();
      if (t_3 == t_4) {
        if (s_2 > s_4 || s_2 < s_3) {
          continue;
        } else {
          return false;
        }
      }
      double k_2 = (s_4 - s_3) / (t_4 - t_3);
      double k_3 = (s_6 - s_5) / (t_4 - t_3);
      double b_2 = s_3 - k_2 * t_3;
      double b_3 = s_5 - k_3 * t_3;
      // Check whether current node is in the object
      if (t_2 < t_4) {
        bool under_min = s_2 < s_3 + k_2 * (t_2 - t_3);
        bool under_max = s_2 < s_5 + k_3 * (t_2 - t_3);
        if (under_min != under_max) {
          return false;
        }
      } else if (t_1 < t_3) {
        bool under_min = s_3 < s_1 + k_1 * (t_3 - t_1);
        bool under_max = s_5 < s_1 + k_1 * (t_3 - t_1);
        if (under_min != under_max) {
          return false;
        }
      }
      // Check whether current edge is cross the object edge
      if (k_2 != k_1) {
        double cross_t = (b_2 - b_1) / (k_1 - k_2);
        if (cross_t > std::max(t_1, t_3) && cross_t < std::min(t_2, t_4)) {
          return false;
        }
      }
      if (k_3 != k_1) {
        double cross_t = (b_3 - b_1) / (k_1 - k_3);
        if (cross_t > std::max(t_1, t_3) && cross_t < std::min(t_2, t_4)) {
          return false;
        }
      }
    }
  }
  return true;
}

void DynamicMapSearch::GetFinalGoal() {
  int break_time = std::numeric_limits<int>::max();
  for (auto it = points_.back().rbegin(); it != points_.back().rend(); it++) {
    // Generate s sequence according to last s
    std::vector<int> s_sequence;
    s_sequence.push_back(it->first);
    for (int i = points_.size() - 1; i > 0; i--) {
      s_sequence.push_back(points_.at(i).at(s_sequence.back()).last_s);
    }
    reverse(s_sequence.begin(), s_sequence.end());
    int time = 0;
    // Calculate take over prior object times
    for (auto &object : prior_objects_) {
      double min_s = object.st_boundary.s_t.front().first.x();
      double min_t = object.st_boundary.s_t.front().first.y();
      double max_t = object.st_boundary.s_t.back().first.y();
      if (max_t < 0.0 || min_t >= context_->speedplan_config_.total_time()) {
        continue;
      }
      int t = (int)min_t;
      double s = s_sequence.at(t) + 
          (s_sequence.at(t + 1) - s_sequence.at(t)) * (min_t - t);
      if (s > min_s) {
        time++;
      }     
    }
    // Get fastest s sequence which has least break times
    if (time < break_time) {
      break_time = time;
      s_sequence_ = s_sequence;
    }
    if (break_time == 0) {
      return;
    }
  }
}

void DynamicMapSearch::GetFinalDecision() {
  for (auto &s : s_sequence_) {
    planning_debug_msgs::DecisionPoint point;
    point.set_s(s);
    point.set_t(context_->decision_result_.st_graph.decision_points().size());
    context_->decision_result_.st_graph.add_decision_points()->CopyFrom(point);
  }
  for (auto &object : objects_) {
    double min_s = object.st_boundary.s_t.front().first.x();
    double min_t = object.st_boundary.s_t.front().first.y();
    double max_t = object.st_boundary.s_t.back().first.y();
    if (max_t < 0.0 || min_t >= context_->speedplan_config_.total_time()) {
      continue;
    }
    int t = (int)min_t;
    double s = s_sequence_.at(t) + 
        (s_sequence_.at(t + 1) - s_sequence_.at(t)) * (min_t - t);
    eObjectDecisionEnum decision = s > min_s ? 
        eObjectDecisionEnum::TAKEWAY : eObjectDecisionEnum::GIVEWAY;
    if ((follow_objects_.count(object.id) || para_objects_.count(object.id)) && 
        s < min_s || object.is_in_line) {
      context_->decision_result_.object_decision[object.id] = 
          eObjectDecisionEnum::FOLLOW;
      AWARN_IF(FLAGS_log_enable) << "id = " << object.id << " decision = 3";
      continue;
    }
    context_->decision_result_.object_decision[object.id] = decision;
    AWARN_IF(FLAGS_log_enable) << "id = " << object.id << 
        " decision = " << (int)decision;
  }
  if (FLAGS_gap_enable && context_->replan_state_ == eReplanStateEnum::EXCUTE) {
    int id = context_->decision_result_.gap_id.first;
    context_->decision_result_.object_decision[id] = eObjectDecisionEnum::TAKEWAY;
    AWARN_IF(FLAGS_log_enable) << "Overtake rear car " << id;
  }
  // for (auto &object : conflict_objects_) {
  //   context_->decision_result_.object_decision[object] = eObjectDecisionEnum::CONFLICT;
  //   AWARN_IF(FLAGS_log_enable) << "id = " << object << " decision = 5";
  // }
}

}  //  namespace planning
}  //  namespace acu