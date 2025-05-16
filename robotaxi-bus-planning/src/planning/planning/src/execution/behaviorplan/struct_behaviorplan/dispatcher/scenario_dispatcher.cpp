#include "scenario_dispatcher.h"

namespace acu {
namespace planning {

bool ScenarioDispatcher::GetScenarioType(eScenarioEnum& type) {
	reference_line_ = &context_->cognition_info_->reference_line_info;
	current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
	if (current_line_ == nullptr) {
		AWARN_IF(FLAGS_log_enable) << "Failed to get current line ptr!";
		return false;
	} 
	context_->dis_to_mission_ = 
			current_line_->mapinfo.dis2missionpoint;
	if (current_line_->mapinfo.distance_to_junctions.empty()) {
		AWARN_IF(FLAGS_log_enable) << "Failed to get junction distance!";
		type = eScenarioEnum::CRUISE;
		context_->dis_to_junction_ = 1000.0;
	} else {
		context_->dis_to_junction_ = 
				current_line_->mapinfo.distance_to_junctions.front().first;
	}	
	GetDisToSolid();
	AWARN_IF(FLAGS_log_enable) << "dis_to_mission_ = " << context_->dis_to_mission_ << 
			" dis_to_junction_ = " << context_->dis_to_junction_ << 
			" dis_to_solid_ = " << context_->dis_to_solid_;
	if (FLAGS_specific_pull_over_enable && SatisfyParkingCondition() && 
			context_->departure_counter_ >= FLAGS_start_yield_frame) {
		AWARN_IF(FLAGS_log_enable) << "Current scenario is PULLOVER!";
		type = eScenarioEnum::PULLOVER;
		return true;
	}
	if (context_->dis_to_junction_ > FLAGS_dis_to_switch_junction || 
			context_->dis_to_junction_ > FLAGS_min_lc_dis && 
			context_->dis_to_solid_ > FLAGS_min_lc_dis && 
			current_line_->mapinfo.first_lc_time != 0) {
		AWARN_IF(FLAGS_log_enable) << "Current scenario is CRUISE!";
		type = eScenarioEnum::CRUISE;
	} else {
		AWARN_IF(FLAGS_log_enable) << "Current scenario is JUNCTION!";
		type = eScenarioEnum::JUNCTION;
	}
	return true;
}

void ScenarioDispatcher::GetDisToSolid() {
	context_->dis_to_solid_ = context_->dis_to_junction_;
	double left_start_s = 0.0, left_end_s;
	for (auto& left_bound : current_line_->mapinfo.left_bd_types) {
		if (left_bound.second < 3) {
			left_start_s = left_bound.first;
			continue;
		}
		left_end_s = left_bound.first;
		double right_start_s = 0.0, right_end_s;
		for (auto& right_bound : current_line_->mapinfo.right_bd_types) {
			if (right_bound.second < 3)  {
				right_start_s = right_bound.first;
				continue;
			} 
			right_end_s = right_bound.first;
			if (right_start_s >= left_end_s || left_start_s >= right_end_s) { 
				continue;
			}
			context_->dis_to_solid_ = std::max(left_start_s, right_start_s);
			return;
		}
	}
	context_->dis_to_solid_ = current_line_->mapinfo.dis_to_end;
}

// Check global cost avoiding multiple lc to pull over
bool ScenarioDispatcher::SatisfyParkingCondition() {
	if (context_->dis_to_mission_ > FLAGS_pull_over_dis) {
		return false;
	}
	double road_l, road_r, bound_l, bound_r;
	double s = context_->dis_to_mission_;
	current_line_->GetWidthToRoadBoundary(road_l, road_r, s, true);
	current_line_->GetWidthToRoadBoundary(bound_l, bound_r, s, false);
	context_->park_road_side_ = bound_r > road_r + 0.5;
	AWARN_IF(FLAGS_log_enable) << "bound_r = " << bound_r << " road_r = " << 
			road_r << " park_road_side_ = " << context_->park_road_side_;
	if (fabs(current_line_->mapinfo.first_lc_time) == 0 || 
			context_->reference_line_map_.rbegin()->first < 30 || 
			fabs(current_line_->mapinfo.first_lc_time) == 1 && 
			!context_->park_road_side_) {
		return true;
	}
	AWARN_IF(FLAGS_log_enable) << "Need multiple lanes to PULLOVER!";
	return false;
}

ScenarioDispatcher::ScenarioDispatcher() {}

ScenarioDispatcher::~ScenarioDispatcher() {}

}  // namespace planning
}  // namespace acu