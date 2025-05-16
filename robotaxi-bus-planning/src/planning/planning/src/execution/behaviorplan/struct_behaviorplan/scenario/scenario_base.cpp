#include "scenario_base.h"

namespace acu {
namespace planning {

ScenarioBase::ScenarioBase() {}

bool ScenarioBase::MakeIntentionDecision() {
	context_->replan_reason_ = eReplanReasonEnum::DEFAULT_VALUE;
  context_->decision_result_.reference_line_id = 
      context_->cognition_info_->reference_line_info.target_line_id;
	BehaviorIntention intention;
	if (intention.Init() == false) {
		return false;
	}	
  if (context_->decider_config_->intention().departure_enable() && 
      intention.HaveDeparture()) {
    context_->replan_reason_ = eReplanReasonEnum::DEPARTURE;
    AWARN_IF(FLAGS_log_enable) << "DEPARTURE intention!";
    return true;
  }
  if (context_->decider_config_->intention().pull_over_enable() && 
      intention.HavePullOver()) {
    context_->replan_reason_ = eReplanReasonEnum::PULL_OVER;
    AWARN_IF(FLAGS_log_enable) << "PULL_OVER intention!";
    return true;
  } 
  if (context_->decider_config_->intention().abandon_lc_enable() && 
      intention.HaveAbandonLC()) {
    context_->replan_reason_ = eReplanReasonEnum::ABANDON_LC;
    AWARN_IF(FLAGS_log_enable) << "ABANDON_LC intention!";
    return true;
  }
  if (context_->replan_state_ == eReplanStateEnum::PREPARE) {
    context_->replan_reason_ = eReplanReasonEnum::PREPARE;
    AWARN_IF(FLAGS_log_enable) << "PREPARE intention!";
    return true;
  }
  if (context_->decider_config_->intention().command_lc_enable() && 
      intention.HaveCommandLC()) {
    context_->replan_reason_ = eReplanReasonEnum::COMMAND_LC;
    AWARN_IF(FLAGS_log_enable) << "COMMAND_LC intention!";
    return true;
  }
	if (context_->decider_config_->intention().mission_lc_enable() && 
			intention.HaveMissionLC()) {
		context_->replan_reason_ = eReplanReasonEnum::MISSION_LC;
		AWARN_IF(FLAGS_log_enable) << "MISSION_LC intention!";
		return true;
	}
  if (context_->decider_config_->intention().meeting_enable() && 
      intention.HaveMeeting()) {
    context_->replan_reason_ = eReplanReasonEnum::MEETING;
    AWARN_IF(FLAGS_log_enable) << "MEETING intention!";
    return true;
  }
	if (context_->decider_config_->intention().obstacle_enable() && 
			intention.HaveObstacleAvoidance()) {
		context_->replan_reason_ = eReplanReasonEnum::OBSTACLE;
		AWARN_IF(FLAGS_log_enable) << "OBSTACLE intention!";
		return true;
	}
	if (context_->decider_config_->intention().speed_lc_enable() && 
			intention.HaveSpeedLC()) {
		context_->replan_reason_ = eReplanReasonEnum::SPEED_LC;
		AWARN_IF(FLAGS_log_enable) << "SPEED_LC intention!";
		return true;
	}
  if (context_->decider_config_->intention().offset_enable() && 
      intention.HaveOffsetCenter()) {
    context_->replan_reason_ = eReplanReasonEnum::OFFSET;
    AWARN_IF(FLAGS_log_enable) << "OFFSET intention!";
    return true;
  } 
	AWARN_IF(FLAGS_log_enable) << "No intention!";
	return false;
}

bool ScenarioBase::MakeLateralDecision() {
  context_->decision_result_.reference_line_id = 
      context_->cognition_info_->reference_line_info.current_line_id;
	ReferenceLineEvaluation evaluator;
  evaluator.GetLineCost();
  context_->best_option_ = evaluator.best_option();
  AWARN_IF(FLAGS_log_enable) << "current_option = " << context_->best_option_.type;
  if (context_->replan_reason_ == eReplanReasonEnum::DEPARTURE || 
      context_->replan_reason_ == eReplanReasonEnum::PULL_OVER) {
    GenerateSpecialSentence();
  } else if (context_->replan_state_ == eReplanStateEnum::CRUISE) { 
    if (context_->best_option_.type < 3) {
      GenerateSentence();
    } else {
      context_->replan_state_ = eReplanStateEnum::PREPARE;
      context_->target_direction_ = context_->best_option_.type % 2 ? 1 : 2;
    }
  } else if (context_->replan_state_ == eReplanStateEnum::PREPARE) {
    if (context_->best_option_.type < 3) {
      context_->replan_state_ = eReplanStateEnum::CRUISE;
    } else if (context_->replan_counter_ < FLAGS_decision_frame) {
      context_->replan_counter_++;
    } else if (FLAGS_gap_enable) {
      FindTargetGap();
      FitGapSpeed();
    } else if (context_->best_option_.is_safe || context_->pedal_command_) {
      context_->replan_state_ = eReplanStateEnum::EXCUTE;
      GenerateSentence();
    }
  } else {
    if (context_->best_option_.type == 0) {
      context_->replan_state_ = eReplanStateEnum::CRUISE;
      SentenceStruct sentence;
      sentence.action = eActionEnum::ABANDON_LANE_CHANGE;
      if (context_->replan_reason_ == eReplanReasonEnum::ABANDON_LC) {
        sentence.intention = eReplanReasonEnum::ABANDON_LC;
      }
      context_->decision_result_.sentence.push_back(sentence);
    } else if (context_->best_option_.type < 3) {
      context_->replan_state_ = eReplanStateEnum::CRUISE;
    } else if (context_->best_option_.line_info->jam_level > 0) {
        auto& follow_id = context_->best_option_.line_info->plan_follow_id;
        context_->decision_result_.follow_id = follow_id;
        AINFO_IF(FLAGS_log_enable) << "follow_id = " << follow_id;
    }
    AERROR << "Replan when state is EXCUTE!";
    GenerateSentence();
  } 
	return true;
}

void ScenarioBase::FitGapSpeed() {
  double rear_speed = 100.0;
  auto target_line = context_->best_option_.line_info;
  if (target_line->reference_lane_id < 20) {
    context_->fit_gap_counter_ = 0;
    return;
  }
  for (auto& gap : target_line->line_gap) {
    if (gap.safety_level > 0) {
      context_->fit_gap_counter_ = 0;
      return;
    }
    if (target_line->objects_.count(gap.start_id)) {
      rear_speed = target_line->objects_[gap.start_id].speed;
    }
  }
  AWARN_IF(FLAGS_log_enable) << "rear_speed = " << rear_speed;
  if (context_->fit_gap_counter_ < 20) {
    context_->fit_gap_counter_++;
  } else if (context_->lc_emergency_level_ > 0) {
    double speed = 0.8 * rear_speed;
    if (target_line->mapinfo.expected_speeds.size()) {
      double lane_speed = target_line->mapinfo.expected_speeds.front().second;
      context_->fit_gap_speed_ = std::max(0.4 * lane_speed, 0.8 * rear_speed);
      AWARN_IF(FLAGS_log_enable) << "lane_speed = " << lane_speed << " speed = " << 
          context_->fit_gap_speed_;
    }
  }
  AWARN_IF(FLAGS_log_enable) << "fit_gap_counter = " << context_->fit_gap_counter_;
}

void ScenarioBase::FindTargetGap() {
  context_->decision_result_.gap_id = std::make_pair(-2, -1);
  if (context_->pedal_command_) {
    context_->replan_state_ = eReplanStateEnum::EXCUTE;
    GenerateSentence();
    return;
  } else if (context_->ego_speed_ < 2.0 && context_->lc_emergency_level_ == 0 && 
      reference_line_->local_reference_line.block_counter == 0) {
    context_->replan_counter_ = FLAGS_decision_frame;
    AINFO_IF(FLAGS_log_enable) << "Wait ego startup to lane change!!!";
    return;
  }
  auto target_line = context_->best_option_.line_info;
  if (context_->best_option_.type == 6) {
    if (reference_line_->right_reference_line.size()) {
      target_line = &reference_line_->right_reference_line.front();
    } else {
      //context_->replan_state_ = eReplanStateEnum::EXCUTE;
      //GenerateSentence();
      return;
    } 
  } else if (context_->best_option_.type == 5) {
    if (reference_line_->left_reference_line.size()) {
      target_line = &reference_line_->left_reference_line.back();
    } else {
      target_line = &reference_line_->reverse_reference_line;
      for (auto& object : target_line->objects_) {
        double left_w, right_w, s = object.second.sl_boundary.min_s;
        target_line->GetWidthToLaneBoundary(left_w, right_w, s);
        if (object.second.is_static || object.second.sl_boundary.min_l > right_w || 
            object.second.sl_boundary.max_l < -left_w || 
            object.second.sl_boundary.min_s > 5.0 && object.second.vsabs > 0.0 || 
            object.second.sl_boundary.max_s < 0.0 && object.second.vsabs < 0.0) {
          continue;
        }
        AWARN_IF(FLAGS_log_enable) << "conflict_id = " << object.first;
        return;
      }
      context_->replan_state_ = eReplanStateEnum::EXCUTE;
      GenerateSentence();
      return;
    }
  }
  int min_level = 0;
  GapStruct* best_gap = nullptr;
  for (auto& gap : target_line->line_gap) {
    if (gap.safety_level == 0 || 
        context_->lc_emergency_level_ < 1 && gap.safety_level == 1) { 
      continue;
    } 
    if (gap.safety_level > min_level || gap.safety_level == min_level && 
        gap.allow_min_t < best_gap->allow_min_t) {
      best_gap = &gap;
      min_level = gap.safety_level;
      context_->decision_result_.gap_id = std::make_pair(gap.start_id, gap.end_id);
      if (target_line->jam_level > 1) {
        context_->decision_result_.follow_id = gap.end_id;
        AINFO_IF(FLAGS_log_enable) << "follow_id = " << gap.end_id;
      }
    }
  }
  if (min_level > 0) {
    if (target_line->mapinfo.path_points.size() && 
        target_line->mapinfo.expected_speeds.size() && 
        target_line->mapinfo.expected_speeds.front().second < best_gap->allow_min_v) {
      auto& point = target_line->mapinfo.path_points.back();
      context_->decision_result_.speed_limit_point = std::make_pair(point.xg, point.yg);
    } else {
      context_->decision_result_.speed_limit_point = std::make_pair(0.0, 0.0);
    }
    // Set speed limit for cognition of next frame (FLAGS_turnlight_frame > 0)
    if (best_gap->allow_min_t > 0.1 || 
        context_->replan_counter_ < FLAGS_decision_frame + FLAGS_turnlight_frame) {
      AINFO_IF(FLAGS_log_enable)<< "approach gap between " << best_gap->start_id << " and " << best_gap->end_id;
      if (best_gap->allow_min_t > 0.1 && best_gap->allow_max_v < context_->ego_speed_) {
        int line_id = reference_line_->current_line_id, index;
        auto current_line = context_->reference_line_map_[line_id];
        Site dec_point;
        double dec_s = pow(context_->ego_speed_, 2) / 1.2;
        dec_s += context_->planning_config_.car_model.front_over_hang;
        current_line->GetNearestPoint(dec_s, dec_point, index);
        Vec2d stop_point(dec_point.xg, dec_point.yg);
        double angle = dec_point.globalangle * acos(-1) / 180.0;
        SentenceStruct sentence;
        sentence.action = eActionEnum::STOP;    
        sentence.box = common::math::Box2d(stop_point, angle, 0.1, 4.0);
        context_->decision_result_.sentence.push_back(sentence);
        context_->stop_type_ += pow(2, 6);
      }
      context_->replan_counter_ = std::min(context_->replan_counter_ + 1, 
          FLAGS_decision_frame + FLAGS_turnlight_frame);
      context_->decision_result_.prepare_direction = context_->best_option_.type - 2;
    } else {
      AINFO_IF(FLAGS_log_enable)<< "choose gap between " << best_gap->start_id << " and " << best_gap->end_id;
      context_->replan_state_ = eReplanStateEnum::EXCUTE;
      GenerateSentence();
    }
  } else {
    context_->replan_counter_ = FLAGS_decision_frame;
  }
}

// for departure and simple pull over
void ScenarioBase::GenerateSpecialSentence() {
  SentenceStruct sentence;
  int line_id = reference_line_->current_line_id;
  auto current_line = context_->reference_line_map_[line_id];
  double end_s = current_line->mapinfo.dis_to_end;
  PathBoundPoint path_bound = make_tuple(end_s, line_id, line_id);
  if (context_->replan_reason_ == eReplanReasonEnum::DEPARTURE) {
    if (end_s > FLAGS_base_lc_dis) {
      path_bound = make_tuple(FLAGS_base_lc_dis, line_id, line_id);
    }
    context_->decision_result_.path_bound.push_back(path_bound);
    sentence.action = eActionEnum::START;
    context_->decision_result_.sentence.push_back(sentence);
  } else if (context_->replan_reason_ == eReplanReasonEnum::PULL_OVER) {
    if (context_->pull_over_ == false) {
      context_->pull_over_ = true;
      sentence.action = eActionEnum::PULL_OVER;
      sentence.dis_to_end = context_->dis_to_mission_ - 1.0;
      sentence.dis_to_boundary = 
          context_->decider_config_->reference_line().dis_to_curb();
      sentence.boundary_enable = FLAGS_pull_over_boundary_enable;
      context_->decision_result_.sentence.push_back(sentence);
      AWARN_IF(FLAGS_log_enable) << sentence.dis_to_end << " to PULL_OVER!";
    }
  }
}

void ScenarioBase::GenerateSentence() {
  SentenceStruct sentence;
  PathBoundPoint path_bound;
  double s = context_->best_option_.line_info->mapinfo.dis_to_end;
  double left_bound, right_bound;
  if (context_->best_option_.type < 1) {
    return;
  } else if (context_->best_option_.type < 3) {
    sentence.direction = eDirectionEnum::DEFAULT_VALUE;
    int bound = context_->best_option_.line_info->reference_lane_id;
    path_bound = make_tuple(s, bound, bound);
    context_->decision_result_.path_bound.push_back(path_bound);
    if (context_->best_option_.type == 2) {
      if (reference_line_->left_reference_line.size()) {
        for (auto& gap : reference_line_->left_reference_line.back().line_gap) {
          if (gap.safety_level == 2) { 
            context_->decision_result_.borrow_lane_type += 1;
            break;
          } 
        }
      }
      if (reference_line_->right_reference_line.size()) {
        for (auto& gap : reference_line_->right_reference_line.front().line_gap) {
          if (gap.safety_level == 2) { 
            context_->decision_result_.borrow_lane_type += 2;
            break;
          } 
        }
      }      
    }
  } else if (context_->best_option_.type % 2) {
    context_->target_direction_ = 1;
    sentence.direction = eDirectionEnum::LEFT;
    GenerateBoundary();
  } else {
    context_->target_direction_ = 2;
    sentence.direction = eDirectionEnum::RIGHT;
    GenerateBoundary();
  }
  if (context_->replan_reason_ == eReplanReasonEnum::OBSTACLE || context_->best_option_.type > 4) {
    sentence.action = eActionEnum::OBSTACLE_AVOID;
  } else {
    sentence.action = eActionEnum::LANE_CHANGE;
  }
  AWARN_IF(FLAGS_log_enable) << "best_option = " << context_->best_option_.type;
  context_->decision_result_.sentence.push_back(sentence);
  context_->decision_result_.reference_line_id = 
      context_->best_option_.line_info->reference_lane_id;
}

void ScenarioBase::GenerateBoundary() {
  PathBoundPoint path_bound;
  int left_bound, right_bound, left_id, right_id;
  int line_id = context_->cognition_info_->reference_line_info.current_line_id;
  auto current_line = context_->reference_line_map_[line_id];
  std::vector<std::pair<double, int>> line_type;
  if (context_->best_option_.type % 2) {
    line_type = context_->reference_line_map_[line_id]->mapinfo.left_bd_types;
    if (context_->best_option_.type > 4) {
      left_id = reference_line_->left_reference_line.size() ? 
          reference_line_->left_reference_line.back().reference_lane_id : 
          reference_line_->reverse_reference_line.reference_lane_id;
    } else {
      left_id = context_->best_option_.line_info->reference_lane_id;
    }
  } else {
    line_type = context_->reference_line_map_[line_id]->mapinfo.right_bd_types;
    right_id = context_->best_option_.type > 4 ? 
        reference_line_->right_reference_line.front().reference_lane_id : 
        context_->best_option_.line_info->reference_lane_id;
  }
  if (context_->replan_reason_ == eReplanReasonEnum::COMMAND_LC) {
    double s = context_->best_option_.line_info->mapinfo.dis_to_end;
    left_bound = context_->best_option_.type % 2 ? left_id : line_id;
    right_bound = context_->best_option_.type % 2 ? line_id : right_id;
    path_bound = make_tuple(s, left_bound, right_bound);
    context_->decision_result_.path_bound.push_back(path_bound);
    AWARN_IF(FLAGS_log_enable) << "s = " << s << 
        " left = " << left_bound << " right = " << right_bound;
    return;
  }
  for(int i = 0; i < line_type.size(); i++) {
    if (context_->best_option_.type % 2) {
      left_bound = left_id;
      right_bound = line_id;
    } else {
      left_bound = line_id;
      right_bound = right_id;
    }
    if (i < line_type.size() && line_type.at(i).second > 2) {
      if (context_->best_option_.boundary.at(i)) {
        if (context_->best_option_.type % 2) {
          left_bound = line_id;
        } else {
          right_bound = line_id;
        }       
      } else {        
        if (context_->best_option_.type % 2) {
          right_bound = left_id;
        } else {
          left_bound = right_id;
        } 
      }
    }
    AWARN_IF(FLAGS_log_enable) << "s = " << line_type.at(i).first << 
        " left = " << left_bound << " right = " << right_bound;
    path_bound = make_tuple(line_type.at(i).first, left_bound, right_bound);
    context_->decision_result_.path_bound.push_back(path_bound);
    if (line_type.at(i).first > FLAGS_expected_lc_dis) {
      break;
    }
  }
  if (context_->best_option_.type < 5 && 
      fabs(current_line->mapinfo.first_lc_time) > 1 && 
      current_line->mapinfo.dis_to_end + 10.0 > 
      context_->best_option_.line_info->mapinfo.dis_to_end) {
    ModifyBoundary();
  } else if (context_->best_option_.type < 5 && context_->dis_to_mission_ < 100.0) {
    ModifyBoundary();
  }
}

void ScenarioBase::ModifyBoundary() {
  if (context_->decision_result_.path_bound.empty()) {
    AERROR_IF(FLAGS_log_enable) << "Path bound doesn't exist!";
    return;
  }
  int line_id = reference_line_->current_line_id;
  auto current_line = context_->reference_line_map_[line_id];
  auto passable_s = current_line->mapinfo.first_lc_time < 0 ? 
      current_line->mapinfo.left_passable_distances : 
      current_line->mapinfo.right_passable_distances;
  auto target_line = context_->best_option_.line_info;
  int lc_num = fabs(target_line->mapinfo.first_lc_time);
  double end_s = context_->park_road_side_ ? 
      context_->dis_to_mission_ - FLAGS_min_lc_dis : context_->dis_to_mission_;
  end_s = std::min(end_s, current_line->mapinfo.dis_to_end);
  if (end_s / (lc_num + 1) > 100.0) {
    AERROR_IF(FLAGS_log_enable) << "Average lc dis is big enough " << end_s / (lc_num + 1);
    return;
  } else if (passable_s.size() != 1) {
    AERROR_IF(FLAGS_log_enable) << "passable size is wrong " << passable_s.size();
    return;
  } 
  double min_s = context_->best_option_.lc_s.first;
  double max_s = context_->best_option_.lc_s.second;
  if (max_s - min_s < FLAGS_min_lc_dis - 1.0) {
    AERROR_IF(FLAGS_log_enable) << "There's no enough space for lc!";
    return;
  }
  // max_s = std::min(max_s, end_s);
  // double s_1 = std::max(0.7 * context_->ego_speed_, 5.0);
  // double s_2 = std::min(0.7 * context_->ego_speed_, 5.0);
  // AWARN_IF(FLAGS_log_enable) << "****lc_num = " << lc_num;
  // if ((max_s - min_s - s_1) / (lc_num + 1) > FLAGS_min_lc_dis) {
  //   end_s = min_s + (max_s - min_s - s_1) / (lc_num + 1) + s_1;
  // } else if ((max_s - min_s - s_2) / (lc_num + 1) > FLAGS_min_lc_dis) {
  //   end_s = min_s + (max_s - min_s - s_2) / (lc_num + 1) + s_2;
  // } else{
  //   end_s = min_s + (max_s - min_s) / (lc_num + 1);
  // }
  // end_s = min_s + (max_s - min_s) / (lc_num + 1);
  // AWARN_IF(FLAGS_log_enable) << "Set average lc dis = " << end_s;
  // double max_lc_dis = std::max(6.0 * context_->ego_speed_, FLAGS_base_lc_dis);
  // if (end_s - min_s > max_lc_dis) {
  //   end_s = min_s + max_lc_dis;
  //   AWARN_IF(FLAGS_log_enable) << "Reduce lc dis = " << end_s;
  // }
  auto& path_bound = context_->decision_result_.path_bound;
  for (int i = 0; i < path_bound.size(); i++) {
    if (std::get<0>(path_bound.at(i)) == end_s) {
      return;
    } else if (std::get<0>(path_bound.at(i)) > end_s) {
      double s = std::get<0>(path_bound.at(i));
      int left = std::get<1>(path_bound.at(i));
      int right = std::get<2>(path_bound.at(i));
      int target = left < 20 ? right : left;
      path_bound.at(i) = make_tuple(s, target, target);
      path_bound.insert(path_bound.begin() + i, make_tuple(end_s, left, right));
      break;
    }
  }
  for (int i = 0; i < path_bound.size(); i++) {
        AINFO_IF(FLAGS_log_enable)<< "s = " << std::get<0>(path_bound.at(i)) << " left = " << 
        std::get<1>(path_bound.at(i)) << " right = " << std::get<2>(path_bound.at(i));
  }
}

bool ScenarioBase::MakeObjectDecision() {
	ObjectEvaluation evaluator;
  if (evaluator.DepartureDecision() == false) {
    return false;
  }
  evaluator.SetLastLCPoint();
  evaluator.SetFollowLCPoint();
  // evaluator.CrosswalkDecision();
  evaluator.SpeedLimitDecision();
  evaluator.VirtualObjectDecision();
  evaluator.ConflictZoneDecision();
  evaluator.CrossObjectDecision();

	return true;
}

bool ScenarioBase::MakeUpperDecision() {}

ScenarioBase::~ScenarioBase() {}

}  // namespace planning
}  // namespace acu
