#include "reference_line_evaluation.h"

namespace acu {
namespace planning {

ReferenceLineEvaluation::ReferenceLineEvaluation() {}

ReferenceLineEvaluation::~ReferenceLineEvaluation() {}

void ReferenceLineEvaluation::GetLineCost() {
  Init();
  GetGlobalCost();
  GetExcuteCost();
  GetStableCost();
  GetExtraCost();
  if (options_.front().type == 3 || options_.front().type == 4) {
    double min_s = 0.0, max_s = 0.0;
    auto& bound = options_.front().type == 3 ? 
        current_line_->mapinfo.left_bd_types : 
        current_line_->mapinfo.right_bd_types;
    for (int i = 0; i < bound.size(); i++) {
      if (bound.at(i).second > 2) {
        if (max_s > 0.0) {   
          break;
        } 
        min_s = bound.at(i).first;
      } else if (bound.at(i).first > FLAGS_min_lc_dis) {
        max_s = bound.at(i).first;
      }
    }
    AWARN_IF(FLAGS_log_enable) << "min_s = " << min_s << " max_s = " << max_s;
    options_.front().lc_s = std::make_pair(min_s, max_s);
  }
}

void ReferenceLineEvaluation::Init() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  OptionStruct option;
  option.speed = 100.0;
  for (auto &line : reference_line_->current_reference_line) {    
    if (line.reference_lane_id == reference_line_->current_line_id) {
      current_line_ = &line;
      option.type = 0;
      option.line_info = &line;
      options_.push_back(option);
      if (line.line_blocked || reference_line_->target_line_id < 20 && 
          reference_line_->local_reference_line.line_blocked) {
        option.type = 2;
        options_.push_back(option);
      }
    } else {
      option.type = 1;
      option.line_info = &line;
      options_.push_back(option);
    }
  }
  if (!context_->decider_config_->reference_line().borrow_lane_enable()) {
    return;
  }
  for (auto &line : reference_line_->left_reference_line) {
    option.type = 3;
    option.line_info = &line;
    options_.push_back(option);
  }
  for (auto &line : reference_line_->right_reference_line) {
    option.type = 4;
    option.line_info = &line;
    options_.push_back(option);
  }
}

void ReferenceLineEvaluation::GetGlobalCost() {
  int min_lc_time = current_line_->mapinfo.all_lc_time;
  for (auto it = options_.begin(); it != options_.end(); ++it) {
    if (it->line_info->mapinfo.all_lc_time < min_lc_time) {
      min_lc_time = it->line_info->mapinfo.all_lc_time;
    }
  }
  for (auto it = options_.begin(); it != options_.end(); ++it) {
    it->global_cost = 0;
    // if (it->type > 1 && it->line_info->mapinfo.all_lc_time < lc_time) {
    //   it->global_cost = -0.1 * (lc_time - it->line_info->mapinfo.all_lc_time);
    //   continue;
    // }
    double extra_cost = it->line_info->mapinfo.global_cost / 10.0 - 
        it->line_info->mapinfo.all_lc_time;
    double dis_to_end = it->line_info->mapinfo.dis_to_end;
    auto& bd_type = it->line_info->mapinfo.first_lc_time < 0 ? 
        it->line_info->mapinfo.left_bd_types : 
        it->line_info->mapinfo.right_bd_types;
    if (bd_type.size() > 1 && bd_type.back().second > 2) {
      dis_to_end = bd_type.at(bd_type.size() - 2).first;
    }
    dis_to_end = std::min(dis_to_end, context_->dis_to_mission_);
    it->global_cost = std::max(extra_cost, 0.0) + FLAGS_expected_lc_dis * 
        fabs(it->line_info->mapinfo.first_lc_time) / dis_to_end;
    if (it->line_info->mapinfo.all_lc_time == 1000) {
      it->global_cost = MAX_COST;
    } else if (it->line_info->mapinfo.all_lc_time == min_lc_time) {
      it->global_cost = 0.9 * extra_cost;
    }
    AWARN_IF(FLAGS_log_enable) << it->type << " global cost = " << it->global_cost;
  }
}

void ReferenceLineEvaluation::GetExcuteCost() {
  StaticMapSearch search;
  for (auto it = options_.begin(); it != options_.end();) {
    if (it->type == 2) {
      search.GeneratePath(true, 1, *it);
      if (it->lc_cost > 0) {
        AWARN_IF(FLAGS_log_enable) << it->type << " lc cost = " << it->lc_cost;
        if (!context_->decider_config_->reference_line().borrow_lane_enable()) {
          it = options_.erase(it);
          continue;
        } 
        for (auto& bd :current_line_->mapinfo.left_bd_types) {
          // AINFO << "s = " << bd.first << " type = " << bd.second;
        }
        OptionStruct temp = *it;
        search.GeneratePath(true, 2, temp);
        if (temp.lc_cost < it->lc_cost) {
          *it = temp;
          it->type = 6;
        } else if (it->lc_cost < MAX_COST) {
          it->type = 5;
        } else {
          it = options_.erase(it);
          continue;
        }
      } else {
        it->lc_cost = 0.5;
      }
    } else if (it->type == 0) {
      it->lc_cost = it->line_info->line_blocked ? MAX_COST : 0;
    } else if (it->type == 1) {
      if (OffGlobalPath(it->line_info)) {
        it->lc_cost = MAX_COST;
      } else {
        search.GeneratePath(true, 0, *it);
      }
    } else if (it->type == 3) {
      if (OffGlobalPath(it->line_info)) {
        it->lc_cost = MAX_COST;
      } else {
        search.GeneratePath(false, 1, *it);
      }
    } else if (it->type == 4) {
      if (OffGlobalPath(it->line_info)) {
        it->lc_cost = MAX_COST;
      } else {
        search.GeneratePath(false, 2, *it);
      }
    } else {
      AERROR << "Invalid option type " << it->type;
    }
    AWARN_IF(FLAGS_log_enable) << it->type << " lc cost = " << it->lc_cost;
    ++it;
  }
}

bool ReferenceLineEvaluation::OffGlobalPath(ReferenceLineFrame* line) {
  auto& passable_s = line->mapinfo.left_passable_distances;
  if (line->reference_lane_id < 20) {
    if (line->reference_lane_id < current_line_->reference_lane_id) {
      passable_s = line->mapinfo.right_passable_distances;
    }
  } else if (line->reference_lane_id < 30) {
    passable_s = reference_line_->left_reference_line.back().mapinfo.right_passable_distances;
  } else {
    passable_s = reference_line_->right_reference_line.front().mapinfo.left_passable_distances;
  }
  // if (passable_s.size()) {
  //   AERROR << line->reference_lane_id << " s = " << passable_s.back().second;
  // }
  if (passable_s.empty() || passable_s.back().second < FLAGS_front_perception_range && 
      line->mapinfo.all_lc_time > current_line_->mapinfo.all_lc_time) {
    return true;
  } 
  return false;
}

void ReferenceLineEvaluation::GetStableCost() {
  auto DP = DataPool::Instance()->GetMainDataPtr();
  float stable_time = (float)context_->stable_counter_ / 10.0;
  float time_cost = stable_time < FLAGS_stable_time ? 
      FLAGS_stable_time - stable_time / FLAGS_stable_time : 0.0;
  float l_cost = fabs(current_line_->mapinfo.dis2line / FLAGS_stable_l);
  float delta = DP->loc_perception.localization_data.yaw - 
      current_line_->mapinfo.path_points.front().globalangle;
  if (fabs(delta) > 180.0) {
    delta += delta > 0.0 ? -360.0 : 360.0;
  }
  float angle_cost = std::max(0.0, fabs(delta / FLAGS_stable_angle) - 0.2);
  float stable_cost = time_cost + std::max(0.0, l_cost - 0.2) + angle_cost;
  if (context_->ego_speed_ < 1.0) {
    stable_cost = std::max(0.0, l_cost - 0.2);
  }
  AWARN_IF(FLAGS_log_enable) << "time_cost = " << time_cost << " l_cost = " << 
      l_cost << " angle_cost = " << angle_cost << " stable_cost = " << stable_cost;
  for (auto it = options_.begin(); it != options_.end(); ++it) {
    if (it->type < 3) {
      it->stable_cost = 0.0;
    } else if (it->type < 5) {
      it->stable_cost = stable_cost;
      auto& passable_s = it->type == 3 ? 
          current_line_->mapinfo.left_passable_distances : 
          current_line_->mapinfo.right_passable_distances; 
      if (current_line_->mapinfo.first_lc_time && 
          passable_s.size() > 1 && !current_line_->line_blocked) {
        float time = (FLAGS_decision_frame + FLAGS_turnlight_frame - context_->replan_counter_) / 10.0;
        double dis = passable_s.front().second;
        if (dis < FLAGS_min_lc_dis) {
          it->stable_cost += MAX_COST;
        } else {
          // it->stable_cost += 0.5 * (6.0 + time) / (dis / context_->ego_speed_);
          it->stable_cost += pow(2.0, 3.0 + time - dis / context_->ego_speed_);
        }
        AWARN_IF(FLAGS_log_enable) << "lc_stable_cost = " << it->stable_cost;
      }
    } else {
      it->stable_cost = stable_cost;
    }
  } 
}

bool ReferenceLineEvaluation::IsRoadCongestion() {
  return context_->decision_result_.is_congestion;
  if (FLAGS_congestion_enable && current_line_->is_congestion && 
      context_->dis_to_junction_ > 0.0) {
    if (reference_line_->left_reference_line.size() && 
        reference_line_->left_reference_line.back().is_congestion &&
        reference_line_->left_reference_line.back().mapinfo.dis_to_merge > 100.0 || 
        reference_line_->right_reference_line.size() && 
        reference_line_->right_reference_line.front().is_congestion &&
        reference_line_->right_reference_line.front().mapinfo.dis_to_merge > 100.0) {
      return true;
    }
  }
  return false;
}

void ReferenceLineEvaluation::GetExtraCost() {
  for (auto it = options_.begin(); it != options_.end();) {
    it->speed_cost = it->line_info->speed_cost + it->line_info->side_cost;
    it->total_cost = it->global_cost + it->lc_cost + it->speed_cost;
    if (context_->replan_state_ != eReplanStateEnum::EXCUTE) {
      it->total_cost += it->stable_cost;
    } else if (context_->replan_reason_ == eReplanReasonEnum::ABANDON_LC) {
      it->total_cost += it->line_info->safety_cost;
    }
    AWARN_IF(FLAGS_log_enable) << it->type << " total cost = "  << it->total_cost
                                           << " global_cost = " << it->global_cost
                                           << " lc_cost = "     << it->lc_cost
                                           << " speed_cost = "  << it->line_info->speed_cost
                                           << " side_cost = "   << it->line_info->side_cost;
    int direction = it->type % 2 ? 1 : 2;
    if (it->type < 3) {
      if (context_->replan_state_ == eReplanStateEnum::PREPARE) {
        it->total_cost += GetPrepareCost();
      } else if (context_->replan_state_ == eReplanStateEnum::EXCUTE) {
        it->total_cost += GetAbandonCost();
        AWARN_IF(FLAGS_log_enable) << "abandon_cost = " << GetAbandonCost();
      }
    } else {
      if (context_->replan_state_ == eReplanStateEnum::EXCUTE) {
        if (it->lc_cost < MAX_COST) {
          it->total_cost -= it->lc_cost;
        }
      } else if (IsRoadCongestion()) {
        it->total_cost += MAX_COST;
        AWARN_IF(FLAGS_log_enable) << "Cautious lc while congestion!";
      } else if (current_line_->dis_to_last_lc > context_->dis_to_junction_ && 
          it->line_info->line_queue) {
        it->total_cost += MAX_COST;
        AWARN_IF(FLAGS_log_enable) << it->type << " line up!";
      } else if (!context_->lc_emergency_level_ && it->line_info->average_v < 5.0) {
        it->total_cost += MAX_COST;
        AWARN_IF(FLAGS_log_enable) << it->type << " is slow for not urgent intention";
      }
      if (context_->target_direction_ > 0 && context_->target_direction_ != direction) {
        AWARN_IF(FLAGS_log_enable) << "Erase other line " << it->type;
        it = options_.erase(it);
        continue;
      } else if (direction == 2) {
        if (reference_line_->right_reference_line.size()) {
          it->is_safe = reference_line_->right_reference_line.front().line_safety;
          if (context_->replan_reason_ == eReplanReasonEnum::ABANDON_LC && it->type > 4) {
            it->total_cost += reference_line_->right_reference_line.front().safety_cost;
          }
        } else {
          AERROR << "Failed to get type " << it->type;
        }        
      } else {
        if (reference_line_->left_reference_line.size()) {
          it->is_safe = reference_line_->left_reference_line.back().line_safety;
          if (context_->replan_reason_ == eReplanReasonEnum::ABANDON_LC && it->type > 4) {
            it->total_cost += reference_line_->left_reference_line.back().safety_cost;
          }
        } else if (reference_line_->reverse_reference_line.reference_lane_id) {
          it->is_safe = reference_line_->reverse_reference_line.line_safety;
        } else {
          AERROR << "Failed to get type " << it->type;
        }
      }
    }

    AWARN_IF(FLAGS_log_enable) << it->type << " total cost final = " << it->total_cost;
    ++it;
  }
  sort(options_.begin(), options_.end(), CompareCost);
}

float ReferenceLineEvaluation::GetPrepareCost() {
  return 0.5 * context_->replan_counter_ / 
      (FLAGS_decision_frame + FLAGS_turnlight_frame);
}

float ReferenceLineEvaluation::GetAbandonCost() {
  double left_w, right_w, l = current_line_->mapinfo.dis2line;
  current_line_->GetWidthToLaneBoundary(left_w, right_w);
  if ((context_->target_direction_ == 1) == (l > 0.0)) {
    return fabs(l) / (left_w + right_w) * 2.0;
  } else {
    return 0.0;
  }  
}

bool ReferenceLineEvaluation::CompareCost(OptionStruct s1, OptionStruct s2) {
  if (s1.type > 0 && s1.total_cost > MAX_COST - 1.0) {
    return false;
  } else if (s2.type > 0 && s2.total_cost > MAX_COST - 1.0) {
    return true;
  } else {
    if (s1.type == 0 && s2.type == 2 || s2.type == 0 && s1.type == 2) {
      return s1.type == 2;
    }
    return s1.total_cost < s2.total_cost;
  }
}

}  //  namespace planning
}  //  namespace acu
