#include "state_checker.h"

namespace acu {
namespace planning {

int ReadyStateCheck::GetReadyState() {

	return 0;
	reference_line_ = &context_->cognition_info_->reference_line_info;
	current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
	if (current_line_ == nullptr || current_line_->mapinfo.path_points.empty()) {
		AWARN_IF(FLAGS_log_enable) << "Current line is empty!";
		return 10;
	}
	double angle = common::math::AngleDiff2(current_line_->loc_site_.globalangle, 
  		current_line_->mapinfo.path_points.front().globalangle);
	AWARN_IF(FLAGS_log_enable) << "ego_angle = " << angle << " steer_angle = " << 
			context_->cognition_info_->vehicle_info.chassis.steering_angle;
	AWARN_IF(FLAGS_log_enable) << "first_lc_time = " << current_line_->mapinfo.first_lc_time << 
			" dis_to_last_lc = " << current_line_->dis_to_last_lc;
	// check steer angle
	if (fabs(context_->cognition_info_->vehicle_info.chassis.steering_angle) > 
			FLAGS_ready_steer_angle) {
		return 1;
	}
	// check ego heading
	if (angle > FLAGS_ready_ego_angle) {
		return 2;
	}
	// check mission lc distance
	if (IsInPathBound() == false) {
		return 3;
	} else if (IsLcDisEnough() == false) {
		return 4;
	} else if (IsGlobalLaneBlocked()) {
		return 5;
	} else if (ReachTargetLine() == false) {
		return 6;
	} else{
		return 0;
	}
}

bool ReadyStateCheck::IsInPathBound() {
	Site corner;
	GeoTool geotool;
	PointVCS rel_point;
	PointGCCS origin_point, target_point;
	double left_w, right_w = 0.0; 
	corner.set_g(current_line_->loc_site_.xg, current_line_->loc_site_.yg);
	origin_point.xg = current_line_->loc_site_.xg;
	origin_point.yg = current_line_->loc_site_.yg;
	origin_point.angle = current_line_->loc_site_.globalangle;
	rel_point.x = context_->planning_config_.car_model.front_over_hang;
	rel_point.y = 0.5 * context_->planning_config_.car_model.car_width;
	geotool.VCS2GCCS(origin_point, rel_point, target_point);
	corner.set_g(target_point.xg, target_point.yg);
	current_line_->GetWidthToRoadBoundary(left_w, right_w, corner, false);
	if (left_w < 0.0 || right_w < 0.0) {
		AERROR << "left_w = " << left_w << " right_w = " << right_w;
	  return false;
	}
	rel_point.x = -context_->planning_config_.car_model.back_over_hang;
	geotool.VCS2GCCS(origin_point, rel_point, target_point);
	corner.set_g(target_point.xg, target_point.yg);
	current_line_->GetWidthToRoadBoundary(left_w, right_w, corner, false);
	if (left_w < 0.0 || right_w < 0.0) {
		AERROR << "left_w = " << left_w << " right_w = " << right_w;
	  return false;
	}
	rel_point.x = context_->planning_config_.car_model.front_over_hang;
	rel_point.y = -0.5 * context_->planning_config_.car_model.car_width;
	geotool.VCS2GCCS(origin_point, rel_point, target_point);
	corner.set_g(target_point.xg, target_point.yg);
	current_line_->GetWidthToRoadBoundary(left_w, right_w, corner, false);
	if (left_w < 0.0 || right_w < 0.0) {
		AERROR << "left_w = " << left_w << " right_w = " << right_w;
	  return false;
	}
	rel_point.x = -context_->planning_config_.car_model.back_over_hang;
	geotool.VCS2GCCS(origin_point, rel_point, target_point);
	corner.set_g(target_point.xg, target_point.yg);
	current_line_->GetWidthToRoadBoundary(left_w, right_w, corner, false);
	if (left_w < 0.0 || right_w < 0.0) {
		AERROR << "left_w = " << left_w << " right_w = " << right_w;
	  return false;
	}
	return true;
}

bool ReadyStateCheck::IsLcDisEnough() {
	if (current_line_->mapinfo.first_lc_time == 0) {
		return true;
	}
	double min_radius = std::max(pow(context_->ego_speed_, 2) / 1.2, 20.0);
	// Get average lc distance
	bool is_in_line = current_line_->IsInLine(current_line_->loc_site_);
	int lc_num = fabs(current_line_->mapinfo.first_lc_time) + (is_in_line ? 0 : 1);
	double dis_to_end = current_line_->dis_to_last_lc + lc_num * FLAGS_min_lc_dis;
	double lc_dis = (dis_to_end - 1.5 * context_->ego_speed_) / lc_num;
	AWARN_IF(FLAGS_log_enable) << "in_line = " << is_in_line << " lc_num = " << 
			lc_num << " lc_dis = " << lc_dis << " r = " << min_radius;
	// Get time for target steer angle
	auto& car_model = context_->planning_config_.car_model;
	double theta = std::atan(car_model.length_wheelbase / min_radius) * 180.0 / M_PI;
	double t = theta * car_model.eps_transmission_ratio / 200.0;
	AWARN_IF(FLAGS_log_enable) << "theta = " << theta << " t = " << t;
	// Get lateral dis to target line
	double l = 0.5 * fabs(current_line_->mapinfo.dis2line);
	for (auto& line : context_->reference_line_map_) {
		if (is_in_line == false){
			break;
		}
		if (line.second->mapinfo.all_lc_time < current_line_->mapinfo.all_lc_time) {
			l = 0.5 * fabs(line.second->mapinfo.dis2line);
			break;
		}
	}
	// Get radius and compare
	if (2.0 * min_radius * l - l * l > 0) {
		double s = sqrt(2.0 * min_radius * l - l * l);
		AWARN_IF(FLAGS_log_enable) << "s = " << 2.0 * s << " l = " << 2.0 * l;
		return lc_dis > 2.0 * s + 4.0 * context_->ego_speed_ * t;
	} else {
		return false;
	}
}

bool ReadyStateCheck::IsGlobalLaneBlocked() {
	int global_line_num = 0;
	auto& scenario_info = context_->cognition_info_->scenario_info;
    //路口前实线区域内可进自动
	// if ( 0 < current_line_->mapinfo.distance_to_junctions.size()            &&
	//      120.0 > current_line_->mapinfo.distance_to_junctions.front().first &&
	//      2.0 < current_line_->mapinfo.distance_to_junctions.front().first   &&
	// 	 0 < current_line_->mapinfo.left_bd_types.size()                    &&
	// 	 2 < current_line_->mapinfo.left_bd_types.front().second            && 
	// 	 0 < current_line_->mapinfo.right_bd_types.size()                   &&
	// 	 2 < current_line_->mapinfo.right_bd_types.front().second ) {
	// 	 return false;
	// }
	//路口内不进自动
	if ( 0 < current_line_->mapinfo.distance_to_junctions.size() &&
	     2.0 > current_line_->mapinfo.distance_to_junctions.front().first){
		return true;
	}
	for (auto& line : context_->reference_line_map_) {
		if (line.second->mapinfo.distance_to_junctions.empty()) {
			continue;
		}
		// far from junction
		double dis_to_junction = 
				line.second->mapinfo.distance_to_junctions.front().first;
		AWARN_IF(FLAGS_log_enable) << line.first << " junction = " << dis_to_junction;
		if (line.second->mapinfo.dis_to_end < dis_to_junction) {
			continue;
		}
		if (dis_to_junction > 120.0 || dis_to_junction < 1e-3) {
			return false;
		}
		global_line_num++;
		double max_s = -1000.0;
		for (auto& it : line.second->objects_) {
			auto& object = it.second;
			if (object.is_static && object.sl_boundary.min_s < dis_to_junction && 
					line.second->block_l_.count(object.id) && 
					line.second->block_l_[object.id] < 1e-3 && 
					scenario_info.waiting_objects_.count(object.id) == 0) {
				max_s = std::max(object.sl_boundary.max_s, max_s);
				AWARN_IF(FLAGS_log_enable) << "max_s = " << max_s << " id = " << object.id;
			}
		}
		for (auto& it : line.second->mapinfo.left_passable_distances) {
			if (it.second <= dis_to_junction && it.second - max_s > FLAGS_min_lc_dis) {
				AWARN_IF(FLAGS_log_enable) << "passable_s = " << it.second;
				return false;
			}
		}
		for (auto& it : line.second->mapinfo.right_passable_distances) {
			if (it.second <= dis_to_junction && it.second - max_s > FLAGS_min_lc_dis) {
				AWARN_IF(FLAGS_log_enable) << "passable_s = " << it.second;
				return false;
			}
		}
	}
	AWARN_IF(FLAGS_log_enable) << "global_line_num = " << global_line_num;
	return global_line_num > 0;
}

bool ReadyStateCheck::ReachTargetLine() {
	PathSearch search;
	return search.GeneratePath();
}

ReadyStateCheck::ReadyStateCheck() {}

ReadyStateCheck::~ReadyStateCheck() {}

}  // namespace planning
}  // namespace acu
