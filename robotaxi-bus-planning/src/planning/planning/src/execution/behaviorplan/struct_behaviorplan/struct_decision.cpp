#include "struct_decision.h"
#include "common/util/ros_util.h"

namespace acu {
namespace planning {

void StructDecision::Init() {
	//std::string pkg_path;
  //acu::common::util::getPackagePath("planning", pkg_path);
  std::string config_file;
  // Load config parameter of all scenarios
  config_file = "/map/work/config/planning_node_config/cognition_conf/cognition.conf";
  //config_file.assign(acu::common::util::GetAbsolutePath(pkg_path, config_file));
  google::SetCommandLineOption("flagfile", config_file.c_str());

  config_file = "/map/work/config/planning_node_config/behaviorplan_conf/decision.conf";
  //config_file.assign(common::util::GetAbsolutePath(pkg_path, config_file));
  google::SetCommandLineOption("flagfile", config_file.c_str());

  config_file = "/map/work/config/planning_node_config/behaviorplan_conf/cruise_config.config";
  DeciderConfig cruise_config;
  //config_file.assign(common::util::GetAbsolutePath(pkg_path, config_file));
  CHECK(acu::common::util::GetProtoFromFile(config_file, &cruise_config)) << 
  		"failed to load cruise config file " << config_file;
  context_->config_[eScenarioEnum::CRUISE] = cruise_config;
  config_file = "/map/work/config/planning_node_config/behaviorplan_conf/junction_config.config";
  DeciderConfig junction_config;
  //config_file.assign(common::util::GetAbsolutePath(pkg_path, config_file));
  CHECK(acu::common::util::GetProtoFromFile(config_file, &junction_config)) << 
  		"failed to load junction config file " << config_file;
  context_->config_[eScenarioEnum::JUNCTION] = junction_config;
  config_file = "/map/work/config/planning_node_config/behaviorplan_conf/pullover_config.config";
  DeciderConfig pullover_config;
  //config_file.assign(common::util::GetAbsolutePath(pkg_path, config_file));
  CHECK(acu::common::util::GetProtoFromFile(config_file, &pullover_config)) << 
  		"failed to load pullover config file " << config_file;
  context_->config_[eScenarioEnum::PULLOVER] = pullover_config;
  context_->decider_config_ = &context_->config_[eScenarioEnum::CRUISE];
  config_file = "/map/work/config/planning_node_config/motionplan_conf/speed_planning_config.config";
  PlanningConfig speedplan_config;
  //config_file.assign(common::util::GetAbsolutePath(pkg_path, config_file));
  CHECK(acu::common::util::GetProtoFromFile(config_file, &speedplan_config)) << 
  		"failed to load speedplan config file " << config_file;
  context_->speedplan_config_ = speedplan_config.planner_config().dp_st_speed_config();
	context_->cognition_info_ = &DP_->GetMainDataRef().cognition_info.struct_env_info;

}

void StructDecision::ResetData() {
	context_->trafficlight_stop_ = false;
	context_->pull_over_ = false;
	context_->park_road_side_ = false;
	context_->find_destination_ = false;
	context_->departure_counter_ = 0;
	context_->stable_counter_ = 0;
	context_->replan_counter_ = 0;
	context_->decision_result_.giveway_id = -1;
	context_->lc_command = 0;
	context_->lc_command_counter = 0;
	context_->crosswalk_stop_ = false;
	context_->crosswalk_slowdown_ = false;
	context_->conflict_stop_ = false;
	context_->conflict_counter_ = 0;
	context_->crosswalk_stop_counter_ = 0;
	context_->crosswalk_slowdown_counter_ = 0;
	context_->st_data_.clear();
	context_->target_direction_ = 0;
	context_->replan_state_ = eReplanStateEnum::CRUISE;
	context_->pedal_command_ = false;
	context_->decision_result_.speed_limit_point = std::make_pair(0.0, 0.0);
	context_->decision_result_.gap_id = std::make_pair(-2, -1);
	context_->cognition_info_->scenario_info.Reset();
	context_->is_passable_ = true;
	context_->mission_lc_blocked_counter_ = 0;
	context_->cruise_blocked_counter_ = 0;
	context_->passable_lane_ids_.clear();
	context_->black_road_ids_.clear();
	DP_->GetMainDataRef().ready_state = 10;
}

void StructDecision::PullData() {
	context_->planning_config_ = DP_->GetMainDataRef().config_info;
	context_->cognition_info_ = &DP_->GetMainDataRef().cognition_info.struct_env_info;
	context_->trajectory_info_ = DP_->GetMainDataRef().motion_path;
	context_->ego_speed_ = context_->cognition_info_->vehicle_info.localization.loc_velocity;
}

void StructDecision::Process() {
	if (!ReferenceLineInit()) {
		ResetData();
		return;
	}
	AWARN_IF(FLAGS_log_enable) << "---------------scenario---------------";
	ScenarioDispatcher dispatcher;
	dispatcher.GetScenarioType(context_->scenario_type);
	context_->decider_config_ = &context_->config_[context_->scenario_type];
	context_->stop_s_ = context_->speedplan_config_.matrix_dimension_t() / 3.6 * 
			DP_->GetMainDataRef().config_info.speedplan_config.maximum_cruising_speed;
	ScenarioBase* decision_ptr;
	ScenarioFactory factory;
	decision_ptr = factory.GetScenarioPtr(context_->scenario_type);
	decision_ptr->reference_line_ = &context_->cognition_info_->reference_line_info;
	ReplanStateManager();
	UpdateCloudCommand();
	UpdatePedalCommand();
	GetCongestionState();
	CheckEmergencyLevel();
	AWARN_IF(FLAGS_log_enable) << "---------------intention---------------";
	if (context_->cognition_info_->vehicle_info.chassis.drive_mode == 1) {
		ReadyStateCheck checker;
		DP_->GetMainDataRef().ready_state = checker.GetReadyState();
		AERROR << "ready_state = " << DP_->GetMainDataRef().ready_state;
	} else {
		DP_->GetMainDataRef().ready_state = 0;
	}
	if (decision_ptr->MakeIntentionDecision()) {
		AWARN_IF(FLAGS_log_enable) << "---------------lateral---------------";
		decision_ptr->MakeLateralDecision();
	} 
	// TrajectoryDecision decider;
	// decider.MakeMeetingDecision();
	AWARN_IF(FLAGS_log_enable) << "---------------object---------------";
	AINFO_IF(FLAGS_log_enable) <<"reference_line_id : " <<context_->decision_result_.reference_line_id;
	if (context_->decision_result_.reference_line_id > 0) {
		decision_ptr->MakeObjectDecision();
		auto lane = context_->reference_line_map_.at(context_->decision_result_.reference_line_id);
		AINFO_IF(FLAGS_log_enable) <<"lane->mapinfo.front_lane_ids : "<<lane->mapinfo.front_lane_ids.size();
		context_->decision_result_.target_lanes = lane->mapinfo.front_lane_ids;
		for (auto &targ_lane : context_->decision_result_.target_lanes){
			AINFO_IF(FLAGS_log_enable) <<"target lane : " <<targ_lane;
		}
	}
	AWARN_IF(FLAGS_log_enable) << "replan_counter = " << context_->replan_counter_;
	AWARN_IF(FLAGS_log_enable) << "result = " << context_->decision_result_.reference_line_id;
	delete decision_ptr;
	CheckPassableForReplan();
}

void StructDecision::PushData() {
	AERROR<<"StructDecision::PushData ";
	DP_->GetMainDataRef().decision_info = context_->decision_result_;
	DP_->GetMainDataRef().turning = GetTurningDirection();
	DP_->GetMainDataRef().debug_planning_msg.decision.set_turning(DP_->GetMainDataRef().turning);
	AWARN_IF(FLAGS_log_enable) << "turning = " << DP_->GetMainDataRef().turning;
	DP_->GetMainDataRef().impassable_flag = 0;
	auto line_id = context_->decision_result_.reference_line_id;
	if (context_->cognition_info_->vehicle_info.chassis.drive_state > 0 && 
			context_->reference_line_map_.count(line_id) && 
			context_->reference_line_map_[line_id]->dis_to_last_lc < 1e-3 && 
			context_->dis_to_mission_ > FLAGS_pull_over_dis) {
		DP_->GetMainDataRef().impassable_flag = 1;
	AWARN_IF(FLAGS_log_enable) << "impassable_flag = 1";
	}
	AddDebugInfo();
	AddExtraDebugInfo();
}

void StructDecision::ReplanStateManager() { 
	AWARN_IF(FLAGS_log_enable) << "state = " << (int)context_->replan_state_ << 
			" target direction = " << context_->target_direction_;
	// LC finished if all pathpoints in the current line
	context_->decision_result_.is_arrive_target_line_failed = false;	
	if (context_->replan_state_ == eReplanStateEnum::EXCUTE) {
		context_->stable_counter_ = 1;
		int target_id = context_->cognition_info_->reference_line_info.target_line_id;
		if (target_id >= 20 && context_->reference_line_map_.count(target_id) && 
				context_->trajectory_info_.path.size()) {
			auto line = context_->reference_line_map_.at(target_id);
			double left_w, right_w;
  		line->GetWidthToLaneBoundary(left_w, right_w, context_->trajectory_info_.path.back());
  		if (left_w > 0.0 && right_w > 0.0) {
  			return;
  		}
		} else if (!reference_line_->path_in_current) {
			return;
		} 
		if (context_->cognition_info_->reference_line_info.target_line_id >= 20) {
			AWARN_IF(FLAGS_log_enable) << "Failed to arrive target line!";
			context_->cognition_info_->reference_line_info.target_line_id = 
					context_->cognition_info_->reference_line_info.current_line_id;
			context_->decision_result_.is_arrive_target_line_failed = true;		
		}
		AWARN_IF(FLAGS_log_enable) << "Replan finished!";
		context_->replan_state_ = eReplanStateEnum::CRUISE;
	} else {
		context_->stable_counter_++;
	}
	if (context_->replan_state_ == eReplanStateEnum::CRUISE) {
		context_->replan_counter_ = 0;
    context_->target_direction_ = 0; 
    context_->pedal_command_ = false;
    context_->fit_gap_counter_ = 0;
	}
}

int StructDecision::GetTurningDirection() {
	AWARN_IF(FLAGS_log_enable) << "---------------turnlight---------------";
	DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(0);
	int path_direction = GetPathDirection();
	GetTurnDirection();
	if (context_->cognition_info_->vehicle_info.chassis.drive_state == 0 && 
			context_->cognition_info_->vehicle_info.chassis.drive_mode == 0) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(1);
		return 1;	// start 
	} else if (context_->decision_result_.reference_line_id == 0) {
		return 0;	// no reference line
	} else if (GetLaneType() == eLaneType::PARKING_LANE) {
		return 3;
	} else if (turn_.first < FLAGS_turnlight_dis && turn_.second > 0) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(3);
		return turn_.second;	// turning
	} else if (reference_line_->target_line_id >= 20) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(5);
		return (reference_line_->target_line_id < 30) ? 1 : 2;  // lane changing
	} else if (context_->replan_counter_ > FLAGS_decision_frame) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(6);
		return context_->target_direction_;	// prepare
	} else if (path_direction > 0){
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(7);
		return path_direction;	// path
	} else if (context_->scenario_type == eScenarioEnum::PULLOVER) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(2);
		return 2;	// pull over
	} else if (merge_.first < 1e-3 && merge_.second > 0) {
		DP_->GetMainDataRef().debug_planning_msg.decision.set_turning_type(4);
		return merge_.second;	// merge
	}
	return 0;
}

eLaneType StructDecision::GetLaneType() {
	if (current_line_->mapinfo.lane_types.size()) {
		AERROR_IF(FLAGS_log_enable) << "current_lane_type = " << 
  			(int)current_line_->mapinfo.lane_types.front().second;
		return current_line_->mapinfo.lane_types.front().second;
	} else {
		return eLaneType::DEFAULT;
	}
}

void StructDecision::GetTurnDirection() {
	if (context_->decision_result_.reference_line_id == 0) {
		return;
	}
	turn_ = std::make_pair(FLAGS_turnlight_dis, 0);
	merge_ = std::make_pair(FLAGS_turnlight_dis, 0);
	if (current_line_->mapinfo.lane_turns.size() && 
			current_line_->mapinfo.lane_turns.at(0).second > 1) {
		if (current_line_->mapinfo.lane_turns.at(0).second > 4) {
			merge_.first = 0.0;
			merge_.second = current_line_->mapinfo.lane_turns.at(0).second - 4;
		} else {
			turn_.first = 0.0;
			turn_.second = current_line_->mapinfo.lane_turns.at(0).second % 2 ? 2 : 1;
		}
	} else if (current_line_->mapinfo.lane_turns.size() > 1) {
		if (current_line_->mapinfo.lane_turns.at(1).second > 1 && 
				current_line_->mapinfo.lane_turns.at(1).second < 5) {
			turn_.first = current_line_->mapinfo.lane_turns.at(0).first;
			turn_.second = current_line_->mapinfo.lane_turns.at(1).second % 2 ? 2 : 1;
		}
	}
	AWARN_IF(FLAGS_log_enable) << "turn_s = " << turn_.first << " type = " << turn_.second;
	AWARN_IF(FLAGS_log_enable) << "merge_s = " << merge_.first << " type = " << merge_.second;
}

int StructDecision::GetPathDirection() {
	if (context_->decision_result_.reference_line_id == 0 || 
			context_->trajectory_info_.path.empty()) {
		return 0;
	}
	double s, l_1, l_2;
	std::list<geometry::Site>& path = context_->trajectory_info_.path;
	XYToSL(current_line_->mapinfo, path.front(), s, l_1);
	int step = 0;
	for (auto it = path.begin(); it != path.end(); it++) {
		if (it->length < double(step)) {
			continue;
		}
		step++;
		XYToSL(current_line_->mapinfo, *it, s, l_2);
		if (fabs(l_1 - l_2) > 0.5) {
			if (fabs(l_1) > 0.9 || fabs(l_2) > 0.9) {
				return (l_2 > l_1) ? 1 : 2;
			} 
		}
		if (it->length > FLAGS_front_perception_range) {
			break;
		}
	}
	return 0;
}

bool StructDecision::ReferenceLineInit() {
	reference_line_ = &context_->cognition_info_->reference_line_info;
	//context_->cognition_info_->vehicle_info.chassis.drive_state < 1 || 
	if (reference_line_->current_reference_line.empty() || 
			reference_line_->current_line_id < 10 || 
			DP_->GetMainDataRef().task_content.command_info == eCommand::COMMAND_STOP) {
		reference_line_ = nullptr;
		context_->decision_result_.target_lanes.clear();
		context_->decision_result_.reference_line_id = 0;
		AWARN_IF(FLAGS_log_enable) << "Failed to initialize frame!";
		return false;
	}
	context_->decision_result_.st_graph.clear_original_objects();
	context_->decision_result_.st_graph.clear_cognition_objects();
	context_->decision_result_.st_graph.clear_decision_points();
	context_->decision_result_.object_decision.clear();
	context_->decision_result_.path_bound.clear();
	context_->decision_result_.sentence.clear();
	context_->decision_result_.prepare_direction = 0;
	context_->decision_result_.is_congestion = false;
	context_->decision_result_.follow_id = -1;
	context_->decision_result_.borrow_lane_type = 0;
	context_->reference_line_map_.clear();
	context_->best_option_.type = 0;
	context_->stop_type_ = 0;
	context_->fit_gap_speed_ = 100.0;
	context_->is_passable_ = true;
	context_->passable_lane_ids_.clear();
	context_->black_road_ids_.clear();
  for (auto &line : reference_line_->current_reference_line) {
  	if (line.reference_lane_id == reference_line_->current_line_id) {
    	current_line_ = &line;
    }
    if (line.mapinfo.able_driving) {
  		context_->reference_line_map_[line.reference_lane_id] = &line;
  	}
  }
  for (auto &line : reference_line_->left_reference_line) {
  	if (line.mapinfo.able_driving) {
  		context_->reference_line_map_[line.reference_lane_id] = &line;
  	}
  }
  for (auto &line : reference_line_->right_reference_line) {
    if (line.mapinfo.able_driving) {
  		context_->reference_line_map_[line.reference_lane_id] = &line;
  	}
  }
  for (auto& line : context_->reference_line_map_) {
  	AINFO_IF(FLAGS_log_enable)<<"id = " << line.first << " all_lc_time = " << 
        line.second->mapinfo.all_lc_time << " first_lc_time = " << 
        line.second->mapinfo.first_lc_time << " global_cost = " << 
        line.second->mapinfo.global_cost << " dis_to_end = " << 
        line.second->mapinfo.dis_to_end << " dis_to_last_lc = " << 
        line.second->dis_to_last_lc;
  }
  return true;
}

void StructDecision::AddDebugInfo() {
	AERROR<<"AddDebugInfo AddDebugInfo";
	auto& debug = DP_->GetMainDataRef().debug_planning_msg.decision;
  debug.set_target_line(context_->decision_result_.reference_line_id);
	debug.set_giveway_id(context_->decision_result_.giveway_id);
	debug.set_speed_limit(context_->decision_result_.speed_limit);
	debug.set_lateral_decision(0);
	debug.mutable_boundary_s()->Clear();
	debug.mutable_left_boundary()->Clear();
	debug.mutable_right_boundary()->Clear();
	debug.mutable_object_id()->Clear();
	debug.mutable_object_decision()->Clear();
	debug.mutable_sentences()->Clear();
	debug.mutable_trajectory()->mutable_points()->Clear();
	debug.set_is_passable(true);
	debug.mutable_passable_lane_ids()->Clear();
	debug.mutable_black_road_ids()->Clear();
	for (auto& bound : context_->decision_result_.path_bound) {
		debug.add_boundary_s(std::get<0>(bound));
		debug.add_left_boundary(std::get<1>(bound));
		debug.add_right_boundary(std::get<2>(bound));
	}
	auto debug_obj_by_line = &DP_->GetMainDataRef().debug_planning_msg.object_debug_by_line;
	debug_obj_by_line->set_object_decision(0);
	for (auto& decision : context_->decision_result_.object_decision) {
		debug.add_object_id(decision.first);
		debug.add_object_decision((int)decision.second);
		if ( decision.first == FLAGS_debug_id ) {
			debug_obj_by_line->set_object_decision((int)decision.second);
		}
	}
	for (auto& sen : context_->decision_result_.sentence) {
		planning_debug_msgs::Sentence sentence;
		if (sen.action == eActionEnum::START) {
			sentence.set_action("START");
		} else if (sen.action == eActionEnum::STOP) {
			sentence.set_action("STOP");
		} else if (sen.action == eActionEnum::OBSTACLE_AVOID) {
			sentence.set_action("OBSTACLE_AVOID");
		} else if (sen.action == eActionEnum::LANE_CHANGE) {
			sentence.set_action("LANE_CHANGE");
		} else if (sen.action == eActionEnum::CHANGE_OFFSET) {
			sentence.set_action("CHANGE_OFFSET");
		} else if (sen.action == eActionEnum::PULL_OVER) {
			sentence.set_action("PULL_OVER");
		} else {
			sentence.set_action(std::to_string((int)sen.action));
		}
		sentence.set_direction((int)sen.direction);
		sentence.set_dis_to_end(sen.dis_to_end);
		sentence.set_dis_to_boundary(sen.dis_to_boundary);
		sentence.set_xg(sen.point.x);
		sentence.set_yg(sen.point.y);
		sentence.set_heading(sen.point.heading);
		debug.add_sentences()->CopyFrom(sentence);
		if (sentence.action() != "STOP") {
			debug.set_lateral_decision(1);
		}
	}
	*(debug.mutable_st_graph()) = context_->decision_result_.st_graph;
	debug.set_decision_result(context_->decision_result_.object_decision_success);
	debug.set_scenario_type((int)context_->scenario_type);
	debug.set_intention_type((int)context_->replan_reason_);
	debug.set_option_type(context_->best_option_.type);
	debug.set_dis_to_junction(context_->dis_to_junction_);
	debug.set_dis_to_stop(context_->stop_s_);
	debug.set_stop_type(context_->stop_type_);
	debug.set_ready_status(DP_->GetMainDataRef().ready_state);
	debug.set_light_s(0.0);
	debug.set_light_color(0);
	debug.set_light_time(0.0);
	debug.set_left_time(0.0);
	if (context_->reference_line_map_.count(debug.target_line())) {
		auto target_line = context_->reference_line_map_[debug.target_line()];
		debug.set_light_s(target_line->mapinfo.trafficlight.light_s);
		if (target_line->mapinfo.trafficlight.history_light.size()) {
			debug.set_light_color(target_line->mapinfo.trafficlight.history_light.back().color);
			debug.set_light_time(target_line->mapinfo.trafficlight.history_light.back().time);
			debug.set_left_time(target_line->mapinfo.trafficlight.history_light.back().left_time);
		}
	}
	debug_obj_by_line->set_max_p(context_->st_data_.count(FLAGS_debug_id) ?
			context_->st_data_[FLAGS_debug_id].max_p : 0.0);
	double delta_v = context_->st_data_.count(FLAGS_debug_id) 
				&& context_->st_data_[FLAGS_debug_id].max_p > 0.2 ?
			0.8/context_->st_data_[FLAGS_debug_id].max_p : 4.0;
	// debug_obj_by_line->set_limit_v(context_->st_data_.count(FLAGS_debug_id) ?
	//   		debug_obj_by_line->mutable_object_debug(0)->speed() + delta_v : -1.0);

	DP_->GetMainDataRef().debug_planning_msg.stmap.clear_decision_st_points();
	if (context_->cognition_info_->vehicle_info.chassis.drive_state < 1 || 
			reference_line_ == nullptr) {
		 	debug.set_is_passable(context_->is_passable_ ? 1 : 0);
		  //DP_->GetMainDataRef().debug_planning_msg.decision.set_is_passable(context_->is_passable_ ? 1 : 0);
  		for(const auto& lane_id : context_->passable_lane_ids_){
    		debug.add_passable_lane_ids(lane_id);
    		AERROR<< "lane id : "<< lane_id;

  		}
  		for(const auto& lane_id : context_->black_road_ids_){
    		debug.add_black_road_ids(lane_id);
  		}
  		AINFO_IF(FLAGS_log_enable) <<"debug.is_passable:"<<(int)debug.is_passable() 
 														<< " , " <<(int)DP_->GetMainDataRef().debug_planning_msg.decision.is_passable();
		  return;
	}
  for (auto &line : reference_line_->current_reference_line) {
    if (line.reference_lane_id == reference_line_->current_line_id) {
    	auto search_line = reference_line_->path_in_current ? 
    			&line : &reference_line_->local_reference_line;
      const int range_t = search_line->GetSTMapRangeT();
	    const int range_s = search_line->GetSTMapRangeS();
	    auto st_map_ptr = search_line->GetSTMapPtr();
      for (int t = 0; t < range_t; t++) {
	      for(int s = 0; s < range_s; s++) {
	    	   auto& st_point = st_map_ptr[t][s];
           planning_debug_msgs::DebugSTPoint temp_point;
           temp_point.set_p(st_point.p);
           temp_point.set_s(st_point.index.s);
           temp_point.set_t(st_point.index.t);
           for (auto &occ_obj : st_point.objs) {
             temp_point.add_id(occ_obj.second.id);
           }
          DP_->GetMainDataRef().debug_planning_msg.stmap.add_decision_st_points()->CopyFrom(temp_point);
        }
      }
    }
  }
  debug.set_expand_l(context_->decision_result_.expand_l);
  for (auto& slt_point : context_->final_path_) {
  	planning_msgs::TrajectoryPoint point;
  	point.set_xg(slt_point.xg);
  	point.set_yg(slt_point.yg);
  	point.set_velocity(slt_point.v);
  	debug.mutable_trajectory()->add_points()->CopyFrom(point);
  }
  debug.set_is_passable(context_->is_passable_ ? 1 : 0);
  for(const auto& lane_id : context_->passable_lane_ids_){
    debug.add_passable_lane_ids(lane_id);
    AERROR<< "lane id : "<< lane_id;
  }
  for(const auto& lane_id : context_->black_road_ids_){
    debug.add_black_road_ids(lane_id);
  }
  AERROR <<"context_->is_passable_ " << context_->is_passable_ <<" , "<<debug.is_passable();
  AINFO_IF(FLAGS_log_enable) <<"debug.is_passable:"<<debug.is_passable() 
 														<< " , " <<DP_->GetMainDataRef().debug_planning_msg.decision.is_passable();
}

void StructDecision::AddExtraDebugInfo(){
	if(!reference_line_){
		return;
	}
	
}
void StructDecision::UpdateCloudCommand() {
	auto& debug = DP_->GetMainDataRef().debug_planning_msg;
	if (DP_->GetMainDataRef().task_content.direction_command > 0) {
		context_->lc_command = DP_->GetMainDataRef().task_content.direction_command;
		context_->lc_command_counter = 0;
	} else if (context_->lc_command > 0) {
		if (context_->lc_command_counter > 20) {
			context_->lc_command = 0;
			context_->lc_command_counter = 0;			
			debug.decision.set_cmd_feedback("Abandon lc command because of timeout!");
			debug.decision.set_lc_status(-1);
			DP_->GetMainDataRef().task_content.direction_command = 0;
		} else {
			context_->lc_command_counter++;
		}
	} else {
		debug.decision.set_cmd_feedback("");
		debug.decision.set_lc_status(0);
		DP_->GetMainDataRef().task_content.direction_command = 0;
	}
}

void StructDecision::UpdatePedalCommand() {
	if (FLAGS_pedal_enable && 
			context_->cognition_info_->vehicle_info.chassis.on_accpedal && 
			context_->replan_state_ == eReplanStateEnum::PREPARE) {
		context_->pedal_command_ = true;
	} 
}

void StructDecision::GetCongestionState() {
	if (FLAGS_congestion_enable && current_line_->is_congestion && 
      context_->dis_to_junction_ > 0.0) {
		double speed_threshold = FLAGS_speed_threshold;
  	if (current_line_->mapinfo.expected_speeds.size()) {
  	  speed_threshold = 0.6 * current_line_->mapinfo.expected_speeds.front().second;
  	} 
    if (reference_line_->left_reference_line.size() && 
        reference_line_->left_reference_line.back().long_term_speed < speed_threshold && 
        reference_line_->left_reference_line.back().mapinfo.dis_to_merge > 100.0 || 
        reference_line_->right_reference_line.size() && 
        reference_line_->right_reference_line.front().long_term_speed < speed_threshold &&
        reference_line_->right_reference_line.front().mapinfo.dis_to_merge > 100.0) {
      context_->decision_result_.is_congestion = true;
    }
  } else if (current_line_->line_queue) {
  	context_->decision_result_.is_congestion = true;
  }
}

void StructDecision::CheckEmergencyLevel() {
	context_->lc_emergency_level_ = 0;
	double max_speed = current_line_->mapinfo.expected_speeds.size() ? 
			current_line_->mapinfo.expected_speeds.front().second : 
			DP_->GetMainDataRef().config_info.speedplan_config.maximum_cruising_speed;
	if (current_line_->mapinfo.first_lc_time && 6.0 * max_speed > 
			current_line_->dis_to_last_lc / fabs(current_line_->mapinfo.first_lc_time)) {
		AWARN_IF(FLAGS_log_enable) << "LC is urgent because of mission lc distance!";
		context_->lc_emergency_level_ = 1;
	} else if (current_line_->line_slow) {
		AWARN_IF(FLAGS_log_enable) << "LC is urgent because current line is slow!";
		context_->lc_emergency_level_ = 1;
	}
}

void StructDecision::CheckPassableForReplan(){
  AINFO_IF(FLAGS_log_enable) <<"CheckPassableForReplan "<<context_->decision_result_.reference_line_id;
  if(!context_->reference_line_map_.count(context_->decision_result_.reference_line_id)){
  	context_->is_passable_ = true;
  	context_->passable_lane_ids_.clear();
  	AINFO_IF(FLAGS_log_enable) <<"return";
  	return;
  }
  if(context_->dis_to_mission_ < FLAGS_pull_over_dis){
  	return;
  }

  if(!CheckMissionLcPassableForReplan()){
  	CheckCruisePassableForReplan();
  }
}



bool StructDecision::CheckMissionLcPassableForReplan(){
  auto target_line = context_->reference_line_map_.at(context_->decision_result_.reference_line_id);
  AINFO_IF(FLAGS_log_enable) <<"target_line->dis_to_last_lc:"<<target_line->dis_to_last_lc
       <<",context_->mission_lc_blocked_counter_:"<<context_->mission_lc_blocked_counter_;

  if(target_line->dis_to_last_lc < 40.0){
    context_->mission_lc_blocked_counter_++;
  }else{
    context_->mission_lc_blocked_counter_ = 0;
  }

  if(context_->mission_lc_blocked_counter_ > FLAGS_mission_lc_blocked_frame){
    std::vector<int> passable_ref_lane_ids;
    PassablePathSearch passable_path_search;
    passable_path_search.GeneratePath(passable_ref_lane_ids);
    AINFO_IF(FLAGS_log_enable) << "[1] after passable_path_search";
    for(auto iter = passable_ref_lane_ids.begin(); iter != passable_ref_lane_ids.end();){
      int id = *iter;
      auto line = context_->reference_line_map_.at(id);
      double dis_to_junction = line->mapinfo.distance_to_junctions.size()? 
      							line->mapinfo.distance_to_junctions.front().first:
      						   -1000;
      AINFO_IF(FLAGS_log_enable) <<"id:"<<id <<",line.all_lc_time:"<<line->mapinfo.all_lc_time
      	  <<",first_lc_time:"<<line->mapinfo.first_lc_time
          <<",line->dis_to_end:"<<line->mapinfo.dis_to_end
          <<",dis_to_junction:"<<dis_to_junction;
      if(line->mapinfo.all_lc_time < current_line_->mapinfo.all_lc_time){
        iter = passable_ref_lane_ids.erase(iter);
        continue;
      }
      iter++;
    }
    context_->is_passable_ = false;
    AINFO_IF(FLAGS_log_enable) <<"final passable_ref_lane_ids:"<<passable_ref_lane_ids.size();
	for(auto& id: passable_ref_lane_ids){
		AINFO_IF(FLAGS_log_enable) <<" ID:" <<id;
	}
    context_->passable_lane_ids_.clear();
    std::vector<string> suc_junc_lane_ids;
    for(auto& ref_lane_id : passable_ref_lane_ids){
      auto line = context_->reference_line_map_.at(ref_lane_id);
      line->GetSucJunctionRoadId(suc_junc_lane_ids);
      context_->passable_lane_ids_.insert(context_->passable_lane_ids_.end(),
                        suc_junc_lane_ids.begin(),
                        suc_junc_lane_ids.end());
    }
    return true;
  }else{
  	context_->is_passable_ = true;
  	context_->passable_lane_ids_.clear();
  	context_->black_road_ids_.clear();
  }
  return false;
}


bool StructDecision::CheckCruisePassableForReplan(){
  const double dis_to_junction_thres = 100.0;
  const double delta_for_reach_global = 10.0;
  // 路径分叉，但地图没有画出junction的场景，如果某些参考线无法到达终点，dis_to_end比较小
  double min_dis_to_end = 1000.0, max_dis_to_end = 0.0;
  for(auto& line:context_->reference_line_map_){
  	if(line.second->mapinfo.all_lc_time > current_line_->mapinfo.all_lc_time){
  	  if(line.second->mapinfo.dis_to_end < min_dis_to_end){
  		min_dis_to_end = line.second->mapinfo.dis_to_end;
  	  }
  	  if(line.second->mapinfo.dis_to_end > max_dis_to_end){
  		max_dis_to_end = line.second->mapinfo.dis_to_end;
  	  }
  	}
  }

  AINFO_IF(FLAGS_log_enable) <<"min_dis_to_end:"<<min_dis_to_end <<",max_dis_to_end:"<<max_dis_to_end;
  if(fabs(max_dis_to_end - min_dis_to_end) < delta_for_reach_global){
    return true;
  }

  bool has_line_blocked = current_line_->line_blocked;
  for(auto& line : context_->reference_line_map_){
  	if(has_line_blocked){
  		break;
  	}
  	has_line_blocked = (has_line_blocked || line.second->line_blocked);
  }

  AINFO_IF(FLAGS_log_enable) <<"dis_to_junction_:"<<context_->dis_to_junction_ <<",line_blocked:"<<int(has_line_blocked)
  		<<",min_dis_to_end:"<<min_dis_to_end;
  
  // 路口或非路口但参考线无法到达终点，同时路径被堵塞
  if(((context_->dis_to_junction_ < dis_to_junction_thres && context_->dis_to_junction_ >0.0)||
  	  (min_dis_to_end < dis_to_junction_thres + delta_for_reach_global)) && 
  	 has_line_blocked){
  	std::map<int,double> search_s;
    std::vector<int> passable_ref_lane_ids;
    PassablePathSearch passable_path_search;
    bool generate_path_sucessed = passable_path_search.GeneratePath(passable_ref_lane_ids);
    passable_path_search.GetSearchS(search_s);
    AINFO_IF(FLAGS_log_enable) << "[2] after passable_path_search";
    bool need_global_replan = true;
    for(auto iter = passable_ref_lane_ids.begin(); iter != passable_ref_lane_ids.end();){
      int id = *iter;
      auto line = context_->reference_line_map_.at(id);
     
      double dis_to_junction = line->mapinfo.distance_to_junctions.front().first;


       AINFO_IF(FLAGS_log_enable) <<"id:"<<id <<",line.all_lc_time:"<<line->mapinfo.all_lc_time
          <<",current_line_.all_lc_time:"<<current_line_->mapinfo.all_lc_time
          <<",line->dis_to_end:"<<line->mapinfo.dis_to_end
          <<",context->dis_to_junction:"<<context_->dis_to_junction_
          <<",dis_to_junction:"<<dis_to_junction;
      // 代表除当前路线外，有其他可以到达终点的路径能够通行
      if(line->mapinfo.dis_to_end > dis_to_junction){
        need_global_replan = false;
        break;
      }

      // 代表除不能到达终点的路线与当前路线外，有其他可以到达终点的路径能够通行
      if(min_dis_to_end < dis_to_junction_thres + delta_for_reach_global && 
      	 line->mapinfo.dis_to_end > min_dis_to_end + delta_for_reach_global){
      	need_global_replan = false;
      	break;
      }
      iter++;
    }
    AINFO_IF(FLAGS_log_enable) <<"need_global_replan:"<<need_global_replan << ",cruise_blocked_counter:"<< context_->cruise_blocked_counter_;
    int front_lane_size = -1;
    if(need_global_replan && generate_path_sucessed){
      context_->cruise_blocked_counter_++;
      if(context_->cruise_blocked_counter_ > FLAGS_cruise_blocked_frame){
        context_->is_passable_ = false;
        context_->passable_lane_ids_.clear();
        std::vector<string> suc_junc_road_ids;
        for(auto& ref_lane_id : passable_ref_lane_ids){
          auto line = context_->reference_line_map_.at(ref_lane_id);
          front_lane_size = line->mapinfo.front_lane_ids.size();
          line->GetSucJunctionRoadId(suc_junc_road_ids);
          context_->passable_lane_ids_.insert(context_->passable_lane_ids_.end(),
                            suc_junc_road_ids.begin(),
                            suc_junc_road_ids.end());
        }
        AINFO_IF(FLAGS_log_enable) <<"final passable_ref_lane_ids:"<<passable_ref_lane_ids.size();
	    for(auto& id: passable_ref_lane_ids){
	      AINFO_IF(FLAGS_log_enable) <<" ID:" <<id;
	    }

	    CheckNeedAddStopPointOrNot(search_s, min_dis_to_end,dis_to_junction_thres,delta_for_reach_global);
	    AddBlackRoad(front_lane_size, min_dis_to_end,dis_to_junction_thres,delta_for_reach_global);
      }else{
      	context_->is_passable_ = true;
        context_->passable_lane_ids_.clear();
        context_->black_road_ids_.clear();
      }
      
    }else{
      context_->cruise_blocked_counter_ = 0;
      context_->is_passable_ = true;
      context_->passable_lane_ids_.clear();
      context_->black_road_ids_.clear();
    }
  } else{
    context_->cruise_blocked_counter_ = 0;
    context_->is_passable_ = true;
    context_->passable_lane_ids_.clear();
    context_->black_road_ids_.clear();
  }
  return true;
}

void StructDecision::CheckNeedAddStopPointOrNot(std::map<int,double>& search_s, 
												const double min_dis_to_end,
												const double dis_to_junction_thres,
												const double delta_for_reach_global){
	int left_or_right = -1;
    int ref_lane_id = -1;
    AINFO_IF(FLAGS_log_enable) <<"search_s:";
    for(auto& s:search_s){
    	AINFO_IF(FLAGS_log_enable) <<"s:"<<s.first<<","<<s.second;
    }

    for(auto& line:context_->reference_line_map_){
    	double dis_to_junction = line.second->mapinfo.distance_to_junctions.front().first;
    	AINFO_IF(FLAGS_log_enable) <<"line.id:"<<line.first<<",dis_to_end:" <<line.second->mapinfo.dis_to_end<<",dis_to_junction:"<< dis_to_junction
    		  <<",min_dis_to_end:"<<min_dis_to_end;

    	if((line.second->mapinfo.dis_to_end > dis_to_junction  ||
    		(min_dis_to_end < dis_to_junction_thres + delta_for_reach_global && 
  	 		 line.second->mapinfo.dis_to_end > min_dis_to_end + delta_for_reach_global))
    		 && search_s.count(line.first)){
    		AINFO_IF(FLAGS_log_enable) <<"try check line need add stop or not;line.id:"<<line.first
    			  <<",right.size:"<<reference_line_->right_reference_line.size()
    			  <<",left.size:"<<reference_line_->left_reference_line.size();
    		// 是否是右侧车道
    		if(line.first >=30 || line.first < 20 && reference_line_->right_reference_line.empty()){
    			left_or_right = 1;
    			ref_lane_id = line.first;
    			break;
    		}

    		if(line.first >= 20 && line.first <30 || line.first < 20 && reference_line_->left_reference_line.empty()){
    			left_or_right = 0;
    			ref_lane_id = line.first;
    			break;
    		}
    	}
    }
    AINFO_IF(FLAGS_log_enable) <<"left_or_right:"<<left_or_right;
    if(left_or_right >=0){
    	ObjectEvaluation evaluator;
    	evaluator.SetGuaranteeReplanPoint(ref_lane_id,left_or_right,search_s[ref_lane_id]);
    }
}

void StructDecision::AddBlackRoad(int &front_lane_size,
									const double min_dis_to_end,
									const double dis_to_junction_thres,
									const double delta_for_reach_global) {
	context_->black_road_ids_.clear();
	for(auto& line:context_->reference_line_map_){
		double dis_to_junction = line.second->mapinfo.distance_to_junctions.front().first;
		if(line.second->mapinfo.dis_to_end > dis_to_junction  ||
    		(min_dis_to_end < dis_to_junction_thres + delta_for_reach_global && 
  	 		 line.second->mapinfo.dis_to_end > min_dis_to_end + delta_for_reach_global)){
			if (line.second->mapinfo.front_lane_ids.size() > front_lane_size) {
				string lane_id = line.second->mapinfo.front_lane_ids.at(front_lane_size);
				context_->black_road_ids_.push_back(LaneToRoad(lane_id));
			}
		}
	}

}

string StructDecision::LaneToRoad(string lane_id) {
	string road = "";
	for (auto it : lane_id) {
		if (it == '_') break;
		road.push_back(it);
	}
	return road;
}

StructDecision::StructDecision() {}

StructDecision::~StructDecision() {}


}  // namespace planning
}  // namespace acu
