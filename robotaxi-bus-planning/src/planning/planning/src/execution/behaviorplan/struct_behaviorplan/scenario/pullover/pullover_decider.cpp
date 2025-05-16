#include "pullover_decider.h"

namespace acu {
namespace planning {

PulloverDecider::PulloverDecider() {}

bool PulloverDecider::MakeIntentionDecision() {
	reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  AWARN_IF(FLAGS_log_enable) << "cur = " << reference_line_->current_line_id << 
      " target = " << reference_line_->target_line_id << " ego_l = " << 
      current_line_->mapinfo.dis2line << " v = " << context_->ego_speed_;
  if (context_->cognition_info_->vehicle_info.chassis.drive_state < 1) {
    if (context_->cognition_info_->vehicle_info.chassis.velocity < 0.1 && 
        current_line_->IsRoadSide()) {
      AWARN_IF(FLAGS_log_enable) << "Roadside Parking!";
      context_->decision_result_.reference_line_id = 0;
      context_->departure_counter_ = 0;
    } else {
      context_->decision_result_.reference_line_id = reference_line_->current_line_id;
    }
    context_->replan_state_ = eReplanStateEnum::CRUISE;
    context_->stable_counter_ = 0;
    AWARN_IF(FLAGS_log_enable) << "In standby mode!";
    return false;
  } 
  AERROR_IF(FLAGS_log_enable) << "enable_struct_pull_over = " << 
  		context_->planning_config_.behavior_config.enable_struct_pull_over;
  if (!context_->planning_config_.behavior_config.enable_struct_pull_over || 
  		reference_line_->target_line_id > 20 && 
  		reference_line_->target_line_id < 30 &&
  		reference_line_->local_reference_line.line_blocked == false && 
  		context_->dis_to_mission_ > FLAGS_min_lc_dis + 1.0 + 
  		context_->planning_config_.car_model.front_over_hang) {
  	context_->decision_result_.reference_line_id = 
      	context_->cognition_info_->reference_line_info.target_line_id;
    return false;
  }
  GetOptionLine();
  context_->decision_result_.reference_line_id = 
      context_->cognition_info_->reference_line_info.current_line_id;
  context_->best_option_.type = 0;
  if (context_->find_destination_ && context_->trajectory_info_.path.size()) {
  	double left_w, right_w;
  	auto& point = context_->trajectory_info_.path.back();
  	current_line_->GetWidthToRoadBoundary(left_w, right_w, point, false);
  	if (right_w - 0.5 * context_->planning_config_.car_model.car_width > 
  			context_->decider_config_->reference_line().dis_to_curb() + 0.1 && 
  			context_->dis_to_mission_ > FLAGS_min_lc_dis) {
  		AERROR << right_w - 0.5 * context_->planning_config_.car_model.car_width;
  		return true;
  	}
  } 
	return !context_->find_destination_ || 
			reference_line_->local_reference_line.line_blocked;
}

bool PulloverDecider::MakeLateralDecision() {
	AWARN_IF(FLAGS_log_enable) << "PULL_OVER intention!";
	if (context_->departure_counter_ < FLAGS_start_yield_frame) {
    return false;
  } else if (IsEgoStable() == false) {
  	AWARN_IF(FLAGS_log_enable) << "Ego is unstable now!";
  	return false;
  } else if (reference_line_->right_reference_line.size() && 
  		!reference_line_->right_reference_line.front().line_safety &&
  		reference_line_->path_in_current == true) {
  	AWARN_IF(FLAGS_log_enable) << "Right line is unsafe now!";
  	return false;
  }
  return FindTargetLine();
}

bool PulloverDecider::IsEgoStable() {
	if (reference_line_->target_line_id > 20 || context_->find_destination_) {
		AWARN_IF(FLAGS_log_enable) << "Ignore stability after start pull over!";
		return true;
	} else if (reference_line_->path_in_current == false) {
		AWARN_IF(FLAGS_log_enable) << "Ignore stability if path is zigzag!";
		return true;
	}
	auto DP = DataPool::Instance()->GetMainDataPtr();
	float angle = DP->loc_perception.localization_data.yaw - 
      current_line_->mapinfo.path_points.front().globalangle;
  AWARN_IF(FLAGS_log_enable) << "angle = " << angle << " l = " << 
  		current_line_->mapinfo.dis2line;
  if (fabs(angle) > 180.0) {
    angle += angle > 0.0 ? -360.0 : 360.0;
  }
	if (context_->ego_speed_ < 1.0 || fabs(angle) < FLAGS_stable_angle && 
		fabs(current_line_->mapinfo.dis2line) < FLAGS_stable_l) {
  	return true;
  }
  return false;
}

void PulloverDecider::GetOptionLine() {
	lines_.clear();
	for (auto& line : reference_line_->current_reference_line) {
		lines_[line.reference_lane_id] = &line;
	}
	for (auto& line : reference_line_->right_reference_line) {
		lines_[line.reference_lane_id] = &line;
	}
	AERROR_IF(FLAGS_log_enable) << "rightest_line_id = " << reference_line_->rightest_line_id;
	AERROR_IF(FLAGS_log_enable && lines_.size()) << "reference_line from " << 
			lines_.begin()->first << " to " << lines_.rbegin()->first;
}

bool PulloverDecider::FindTargetLine() {
	PathSearch searcher;
	searcher.GeneratePath(lines_);
	for (auto it = lines_.rbegin(); it != lines_.rend(); ++it) {
		BuildBlockSet(it->second);
		bool find_target = false;
		double dis = context_->dis_to_mission_;
		double min_s = pow(context_->ego_speed_, 2) / 2.0;
		double max_s = std::min(it->second->mapinfo.dis_to_end, 
				dis + context_->decider_config_->parking().allowable_offset());
		AWARN_IF(FLAGS_log_enable) << it->first << " min_s = " << min_s << " max_s = " << max_s;
		for (int i = 0; i < (int)FLAGS_pull_over_dis; i++) {
			if (CheckDestClear(dis + i, min_s, max_s) && 
					searcher.CheckDestPassable(it->first, dis + i)) {
				context_->pullover_position = std::make_pair(it->first, i);
				context_->dis_to_mission_ = dis + i;
				find_target = true;
				break;
			}
			if (CheckDestClear(dis - i, min_s, max_s) && 
					searcher.CheckDestPassable(it->first, dis - i)) {
				context_->pullover_position = std::make_pair(it->first, i);
				context_->dis_to_mission_ = dis - i;
				find_target = true;
				break;
			}				
		}
		if (find_target) {
			SentenceStruct sentence;
			sentence.action = eActionEnum::PULL_OVER;
			sentence.pullover_line_id = it->first == reference_line_->rightest_line_id ? -1 : it->first;
			sentence.dis_to_end = context_->dis_to_mission_;
			sentence.dis_to_boundary = 
			    context_->decider_config_->reference_line().dis_to_curb();
			sentence.boundary_enable = FLAGS_pull_over_boundary_enable;
			context_->decision_result_.sentence.push_back(sentence);
			context_->decision_result_.reference_line_id = reference_line_->current_line_id;
			context_->find_destination_ = true;
			AWARN_IF(FLAGS_log_enable) << sentence.dis_to_end << " to PULL_OVER!";
			return true;
		}	
	}
	AWARN_IF(FLAGS_log_enable) << "Can't generate parking path!";
	return false;
}

void PulloverDecider::BuildBlockSet(const ReferenceLineFrame* line) {
	block_set_.clear();
	clear_area_.clear();
	double left_w, right_w;
	double car_width = context_->planning_config_.car_model.car_width;
	for (auto it = line->objects_.begin(); it != line->objects_.end(); it++) {
		auto &object = it->second;
		if (!object.is_static || object.sl_boundary.max_s < 0.0) {
			continue;
		}
		auto& sl = object.sl_boundary;
		if (line->reference_lane_id != reference_line_->rightest_line_id) {
			line->GetWidthToLaneBoundary(left_w, right_w, sl.min_s);
			if (sl.max_l < -right_w || sl.min_l > left_w) {
				continue;
			}
		} else {
			line->GetWidthToRoadBoundary(left_w, right_w, sl.min_s, false);
			double width = car_width + FLAGS_collision_buff + 
					context_->decider_config_->reference_line().dis_to_curb();
			if (sl.max_l < -right_w || sl.min_l > width - right_w) {
				continue;
			}
		}
		for (int i = (int)sl.min_s; i <= (int)(sl.max_s + 1.0); i++) {
			block_set_.insert(i);
		}
	}
	for (auto& forbid_area : line->mapinfo.distance_to_forbid_areas) {
		AERROR << "first = " << forbid_area.first << " second = " << forbid_area.second;
		int i = std::max((int)forbid_area.first, 0);
		for (; i <= (int)forbid_area.second; i++) {
			clear_area_.insert(i);
		}
	}
	int start_s = 0;
	for (auto& lane_type : line->mapinfo.lane_types) {
		AERROR << "s = " << lane_type.first << " type = " << (int)lane_type.second;
		if ((int)lane_type.second == 6) {
			for (int i = start_s; i < (int)lane_type.first; i++) {
				block_set_.insert(i);
			}
		}
		start_s = (int)lane_type.first;
	}
}

bool PulloverDecider::CheckDestClear(const double dest_s, const double min_s, 
																		 const double max_s) {
	// Make sure enough room for deceleration and turning
	auto& front_over_hang = context_->planning_config_.car_model.front_over_hang;
	auto& back_over_hang = context_->planning_config_.car_model.back_over_hang;
	if (dest_s + front_over_hang > max_s || dest_s < min_s) { 
		return false;
	}
	int front_s = (int)(dest_s + front_over_hang + 3.0);
	int back_s = std::max((int)(dest_s - FLAGS_min_lc_dis - back_over_hang), 0);
	for (int i = back_s; i <= front_s; i++) {
		if (block_set_.count(i)) {
			AWARN_IF(FLAGS_log_enable) << "dest_s = " << dest_s << " block_s = " << i;
			return false;
		}
	}
	back_s = std::max((int)(dest_s - back_over_hang), 0);
	for (int i = back_s; i <= (int)(dest_s + front_over_hang); i++) {
		if (clear_area_.count(i)) {
			AWARN_IF(FLAGS_log_enable) << "clear_area_s = " << i;
			return false;
		}
	}
	AWARN_IF(FLAGS_log_enable) << "dest_s = " << dest_s << " front_s = " << 
			front_s << " back_s = " << back_s;
	return true;
}

PulloverDecider::~PulloverDecider() {}

}  // namespace planning
}  // namespace acu
