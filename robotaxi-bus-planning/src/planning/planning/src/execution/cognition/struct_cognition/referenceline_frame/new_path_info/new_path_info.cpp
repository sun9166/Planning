#include "new_path_info.h"

namespace acu{
namespace planning {

bool NewPathInfo::MotionPathToFrame(PathData &local_path, const ReferenceLineFrame &target_line_frame,
													const ReferenceLineFrame &current_line_frame, LocalizationData &localization,
                          CarModel &car_model, ReferenceLineFrame &local_line) {
  bool has_local_flag = false;
  if (local_path.path.empty()) return has_local_flag;
  local_line.ClearData();
  ReferenceLineFrame temp_frame;
  AERROR_IF(FLAGS_log_enable)<<"local_path.path size "<<local_path.path.size();
  if (temp_frame.CalculationFrame(local_path, localization, car_model)) {
    if (local_path.is_new) {
      local_line.Reset();
    }
    local_line.SetData(temp_frame);
		target_line_ptr_ = &target_line_frame;
    UpdateAllDistance(local_line);
    local_line.reference_lane_id = 40;
    local_line.mapinfo.front_lane_ids = current_line_frame.mapinfo.front_lane_ids;
    has_local_flag = true;
  }
  return has_local_flag;
}

bool NewPathInfo::UpdateAllDistance(ReferenceLineFrame &update_frame) {
	if (target_line_ptr_ == nullptr || target_line_ptr_->mapinfo.path_points.empty() || 
		update_frame.mapinfo.path_points.empty()) {
		return false;
	} 
	new_path_ptr_ = &update_frame.mapinfo.path_points;
	update_frame.mapinfo.first_lane_start_s = target_line_ptr_->mapinfo.first_lane_start_s;
	if(!UpdateS(target_line_ptr_->mapinfo.dis2missionpoint, 
				update_frame.mapinfo.dis2missionpoint)) {
		AERROR<<"Update missionpoint s failed.";
		return false;
	}
	update_frame.mapinfo.front_relation_lanes.clear();
	if (!target_line_ptr_->mapinfo.front_relation_lanes.empty()) {
		int size = target_line_ptr_->mapinfo.front_relation_lanes.size();
		update_frame.mapinfo.front_relation_lanes.reserve(size);
		for (auto &relation_lanes : target_line_ptr_->mapinfo.front_relation_lanes) {
			update_frame.mapinfo.front_relation_lanes.push_back(relation_lanes);
		}
	}
	update_frame.mapinfo.back_relation_lanes.clear();
	if (!target_line_ptr_->mapinfo.back_relation_lanes.empty()) {
		int size = target_line_ptr_->mapinfo.back_relation_lanes.size();
		update_frame.mapinfo.back_relation_lanes.reserve(size);
		for (auto &relation_lanes : target_line_ptr_->mapinfo.back_relation_lanes) {
			update_frame.mapinfo.back_relation_lanes.push_back(relation_lanes);
		}
	}
	update_frame.mapinfo.expected_speeds.clear();
	if (!target_line_ptr_->mapinfo.expected_speeds.empty()) {
		int size = target_line_ptr_->mapinfo.expected_speeds.size();
		update_frame.mapinfo.expected_speeds.reserve(size);
		for (auto &limit_speed : target_line_ptr_->mapinfo.expected_speeds) {
			pair<double, double> local_limit_speed(0.0, 1000.0);
			if(!UpdateS(limit_speed.first, local_limit_speed.first)) {
				return false;
			}
			local_limit_speed.second = limit_speed.second;
			update_frame.mapinfo.expected_speeds.push_back(local_limit_speed);
		}
	}
	update_frame.mapinfo.lane_types.clear();
	if (!target_line_ptr_->mapinfo.lane_types.empty()) {
		int size = target_line_ptr_->mapinfo.lane_types.size();
		update_frame.mapinfo.lane_types.reserve(size);
		for (auto &lane_type : target_line_ptr_->mapinfo.lane_types) {
			pair<double, eLaneType> local_lane_type(0.0, eLaneType::DEFAULT);
			if(!UpdateS(lane_type.first, local_lane_type.first)) {
				return false;
			}
			local_lane_type.second = lane_type.second;
			update_frame.mapinfo.lane_types.push_back(local_lane_type);
		}
	}
	update_frame.mapinfo.left_bd_types.clear();
	if (!target_line_ptr_->mapinfo.left_bd_types.empty()) {
		int size = target_line_ptr_->mapinfo.left_bd_types.size();
		update_frame.mapinfo.left_bd_types.reserve(size);
		for (auto &left_bd_type : target_line_ptr_->mapinfo.left_bd_types) {
			pair<double, int> local_left_bd_type(0.0, 0);
			if(!UpdateS(left_bd_type.first, local_left_bd_type.first)) {
				return false;
			}
			local_left_bd_type.second = left_bd_type.second;
			update_frame.mapinfo.left_bd_types.push_back(local_left_bd_type);
		}
	}
	update_frame.mapinfo.right_bd_types.clear();
	if (!target_line_ptr_->mapinfo.right_bd_types.empty()) {
		int size = target_line_ptr_->mapinfo.right_bd_types.size();
		update_frame.mapinfo.right_bd_types.reserve(size);
		for (auto &right_bd_type : target_line_ptr_->mapinfo.right_bd_types) {
			pair<double, int> local_right_bd_type(0.0, 0);
			if(!UpdateS(right_bd_type.first, local_right_bd_type.first)) {
				return false;
			}
			local_right_bd_type.second = right_bd_type.second;
			update_frame.mapinfo.right_bd_types.push_back(local_right_bd_type);
		}
	}

	update_frame.mapinfo.distance_to_speed_bumps.clear();
	if (!target_line_ptr_->mapinfo.distance_to_speed_bumps.empty()) {
		int size = target_line_ptr_->mapinfo.distance_to_speed_bumps.size();
		update_frame.mapinfo.distance_to_speed_bumps.reserve(size);
		for (auto &speed_bump : target_line_ptr_->mapinfo.distance_to_speed_bumps) {
			pair<double, double> local_speed_bump(100.0, 100.0);
			if(!UpdateS(speed_bump.first, local_speed_bump.first) ||
			   !UpdateS(speed_bump.second, local_speed_bump.second)) {
				return false;
			}
			update_frame.mapinfo.distance_to_speed_bumps.push_back(local_speed_bump);
		}
	}
	update_frame.mapinfo.distance_to_forbid_areas.clear();
	if (!target_line_ptr_->mapinfo.distance_to_forbid_areas.empty()) {
		int size = target_line_ptr_->mapinfo.distance_to_forbid_areas.size();
		update_frame.mapinfo.distance_to_forbid_areas.reserve(size);
		for (auto &forbid_area : target_line_ptr_->mapinfo.distance_to_forbid_areas) {
			pair<double, double> local_forbid_area(100.0, 100.0);
			if(!UpdateS(forbid_area.first, local_forbid_area.first) ||
			   !UpdateS(forbid_area.second, local_forbid_area.second)) {
				return false;
			}
			update_frame.mapinfo.distance_to_forbid_areas.push_back(local_forbid_area);
		}
	}
	update_frame.mapinfo.distance_to_junctions.clear();
	if (!target_line_ptr_->mapinfo.distance_to_junctions.empty()) {
		int size = target_line_ptr_->mapinfo.distance_to_junctions.size();
		update_frame.mapinfo.distance_to_junctions.reserve(size);
		for (auto &junction : target_line_ptr_->mapinfo.distance_to_junctions) {
			pair<double, double> local_junction(100.0, 100.0);
			if(!UpdateS(junction.first, local_junction.first) ||
			   !UpdateS(junction.second, local_junction.second)) {
				return false;
			}
			update_frame.mapinfo.distance_to_junctions.push_back(local_junction);
		}
	}
	update_frame.mapinfo.distance_to_crosswalks.clear();
	if (!target_line_ptr_->mapinfo.distance_to_crosswalks.empty()) {
		int size = target_line_ptr_->mapinfo.distance_to_crosswalks.size();
		update_frame.mapinfo.distance_to_crosswalks.reserve(size);
		for (auto &crosswalk : target_line_ptr_->mapinfo.distance_to_crosswalks) {
			pair<double, double> local_crosswalk(100.0, 100.0);
			if(!UpdateS(crosswalk.first, local_crosswalk.first) ||
			   !UpdateS(crosswalk.second, local_crosswalk.second)) {
				return false;
			}
			update_frame.mapinfo.distance_to_crosswalks.push_back(local_crosswalk);
		}
	}
	return true;
}

bool NewPathInfo::UpdateS(const double &old_s, double &new_s) {
	if (old_s < 0.1 || old_s > 100.0 || old_s > new_path_ptr_->back().length + 1.0) {
		new_s = old_s;
		return true;
	}
	Site change_point;
	int index = -1;
	if (!target_line_ptr_->GetNearestPoint(old_s, change_point, index)) {
		return false;
	}
	double a, b, c;
	GetVerticalLine(change_point, a, b, c);
	SiteVec local_new_path;
	double start_s = max(0.0, old_s - FLAGS_search_range);
	double end_s = target_line_ptr_->mapinfo.path_points.back().length;
	end_s = min(end_s - 0.1, old_s + FLAGS_search_range);
	for (auto it = new_path_ptr_->begin(); it < new_path_ptr_->end(); it++) {
		if (it->length < start_s || it->length > end_s) continue;
		local_new_path.push_back(*it);
	}
	double min_dis = std::numeric_limits<double>::max();
	int min_index = -1;
	for (int i = 0; i < local_new_path.size(); i++) {
		double dis = fabs(a * local_new_path.at(i).xg + b * local_new_path.at(i).yg + c) 
					 / sqrt(a * a + 1);
		if (dis < min_dis) {
			min_dis = dis;
			min_index = i;
		}
		// if (dis > min_dis + 5.0) break;
	}
	if (min_dis > 3.0 || min_index < 0) {
		AERROR<<"UpdateS failed. mins dis "<<min_dis<<" min_index "<<min_index;
		return false;
	}
	new_s = local_new_path.at(min_index).length;
	return true;
}





}
}