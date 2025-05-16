#include "struct_cognition.h"
#include "common/util/file.h"
 
using geometry::Site;
using namespace acu::common;
using namespace acu::common::util;

namespace acu{
namespace planning {
 
StructCognition::StructCognition() {}

StructCognition::~StructCognition() {}
  
void StructCognition::Init() {
  //std::string planning_pkg_path;
  //acu::common::util::getPackagePath("planning", planning_pkg_path);
  std::string flag_file_path = "/map/work/config/planning_node_config/cognition_conf/cognition.conf";
  //flag_file_path.assign(acu::common::util::GetAbsolutePath(planning_pkg_path, flag_file_path));
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  auto &DP = DataPool::Instance()->GetMainDataRef().config_info;
  cognition_config_ = DP.cognition_config;
  speedplan_config_ = DP.speedplan_config;
  car_model_        = DP.car_model;
  last_current_lanes_.clear();
  last_target_lanes_.clear();
}

void StructCognition::Reset() {// 清理全局数据，处理历史数据
  AERROR<<"Reset StructCognition.";
  auto DP = DataPool::Instance()->GetMainDataPtr();
  DP->cognition_info.struct_env_info.Reset();
  last_current_lanes_.clear();
  last_target_lanes_.clear();
  target_line_ptr_ = nullptr;
  current_line_ptr_ = nullptr;
  allline_objects_.clear();
}

void StructCognition::PullData() { 
  std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
  start_time_ = acu::common::NodeTime::Now().ToSecond();
  auto DP = DataPool::Instance()->GetMainDataPtr();
  perception_        = DP->loc_perception.perception_data;
  prediction_        = DP->prediction_data;
  drivestatus_       = DP->drive_status;
  behavior_          = DP->decision_info;
  motionpath_        = DP->motion_path;
  main_data_ptr_     = &DP->cognition_info.struct_env_info;
  last_target_lanes_ = behavior_.target_lanes;
  AINFO_IF(FLAGS_log_enable) <<"behavior_.target_lanes= "<<behavior_.target_lanes.size(); 
  for (auto &targ_lane : last_target_lanes_){
    AINFO_IF(FLAGS_log_enable) << "target lane "<<targ_lane;
  }
  int map_index = -1, imu_index = -1;
  double map_min_time = std::numeric_limits<double>::max();
  double imu_min_time = std::numeric_limits<double>::max();
  //if(ChooseSyncData(DP->loc_perception.perception_data.timestamp,
  //               DP->mapengine_data_list, DP->loc_perception.loc_data_list,
  //               map_min_time, imu_min_time, map_index, imu_index,
  //               compensate_s_in_unit_m_, compensate_t_in_unit_s_)){
  //DP->mapengine_data.Set(DP->mapengine_data_list.at(map_index));
  //  DP->loc_perception.localization_data = DP->loc_perception.loc_data_list.at(imu_index);
  //}// 找时间同步的定位数据
  {  
    //std::lock_guard<std::mutex> lock(acu::planning::DataPool::Instance()->mutex_);
    if (DP->mapengine_data_list.size() > 0) { 
      DP->mapengine_data.Set(DP->mapengine_data_list.back());
    }
    if (DP->loc_perception.loc_data_list.size() > 0) { 
     DP->loc_perception.localization_data = DP->loc_perception.loc_data_list.back();
    }
    DP->debug_planning_msg.cognition.set_map_delta_time(map_min_time);
    DP->debug_planning_msg.cognition.set_imu_delta_time(imu_min_time);
    mapengine_ = DP->mapengine_data;
    localization_ = DP->loc_perception.localization_data;
  }
  light_matching_.MatchTrafficLights(perception_, localization_, matched_lights_);
  //DP->debug_planning_msg.cell_obstacle.clear();
}

void StructCognition::PushData() {
  auto DP = DataPool::Instance()->GetMainDataPtr();
  if (main_data_ptr_->vehicle_info.chassis.drive_mode == 1
    && drivestatus_.control_mode == 0 && !CheckCurrentPoseEnablePark()) {
    DP->task_exe_result = eTaskExeResult::FAILURE;
  }
  PushVehicleData();
  last_current_lanes_.clear();
  if (current_line_ptr_ == nullptr || current_line_ptr_->mapinfo.path_points.empty()) {
    last_current_lanes_.clear();
    AERROR_IF(FLAGS_log_enable) << "no current line id found "<<
                  main_data_ptr_->reference_line_info.current_line_id;
    return;
  } 
  last_current_lanes_ = current_line_ptr_->mapinfo.front_lane_ids;
  if (last_current_lanes_.empty()) {
    AERROR<<"update history data failed.";
  } 
} 

bool StructCognition::CheckCurrentPoseEnablePark() {
  if (current_line_ptr_ == nullptr || current_line_ptr_->mapinfo.lane_types.empty()) {
    return true;
  }
  if (current_line_ptr_->mapinfo.lane_types.front().second == eLaneType::PARKING_LANE) {
    double s = 0;
    double min_dis = std::numeric_limits<double>::max();
    int min_index = 0;
    Site loc, nearest_point;
    loc.xg = localization_.xg;
    loc.yg = localization_.yg;
    loc.globalangle = localization_.yaw;
    if (current_line_ptr_->GetGlobalNearestPointwithHeading(
        loc, nearest_point, s, min_dis, min_index) ) {
      if (loc.DistanceTo(nearest_point) > 0.2 || 
        NormalizeAngle(loc.globalangle - nearest_point.globalangle) > 30.0 )
      return false;
    }
  }
  return true;
}

void StructCognition::PushVehicleData() {
  int last_state = main_data_ptr_->vehicle_info.chassis.drive_state;
  main_data_ptr_->vehicle_info.Reset(); 
  main_data_ptr_->vehicle_info.localization.Set(localization_);
  main_data_ptr_->vehicle_info.chassis.Set(drivestatus_);
  main_data_ptr_->vehicle_info.chassis.drive_state = last_state;
  if (main_data_ptr_->vehicle_info.chassis.drive_mode == 0 && 
      current_line_ptr_ != nullptr) {
    if (main_data_ptr_->vehicle_info.chassis.velocity < 0.1 &&
        current_line_ptr_->IsRoadSide() &&
        main_data_ptr_->vehicle_info.chassis.drive_state != 1) {
      start_counter_++;
      if (start_counter_ >= 30) {
        main_data_ptr_->vehicle_info.chassis.drive_state = 1;
        start_counter_ = 0;
      } else {
        main_data_ptr_->vehicle_info.chassis.drive_state = 0;
      }
    } else {
      main_data_ptr_->vehicle_info.chassis.drive_state = 1;
      start_counter_ = 0;
    }
  } else {
    main_data_ptr_->vehicle_info.chassis.drive_state = 0;
    start_counter_ = 0;
  }
}

void StructCognition::ProcessData() {
  current_line_ptr_ = nullptr;
  target_line_ptr_ = nullptr;
  int drive_state = main_data_ptr_->vehicle_info.chassis.drive_state;
  AERROR_IF(FLAGS_log_enable)<<" driver mode "<<drivestatus_.control_mode;
  auto &reference_lines = main_data_ptr_->reference_line_info;
  LineProcess correction(mapengine_.map_info_data, last_current_lanes_, 
      last_target_lanes_, motionpath_, drive_state, localization_, car_model_);
  if (!correction.CorrectionProcess(correction_index_, target_index_)) {
    AERROR_IF(FLAGS_log_enable) << "correct failed! correction_index_ is "
                <<correction_index_<<", target_index_ is "<<target_index_;
    return;
  }
  AERROR_IF(FLAGS_log_enable)<<"mapengine_ index is "<<mapengine_.map_info_data.index
                             <<", correction_index_ is "<<correction_index_
                             <<", target_index_ is "<<target_index_;
  correction.LineClassification(reference_lines);
  if (reference_lines.current_reference_line.empty()) {
    AERROR_IF(FLAGS_log_enable) << "LineClassification failed!";
    return; 
  }
  current_line_ptr_ = GetTargetData(reference_lines, reference_lines.current_line_id);
  target_line_ptr_ = GetTargetData(reference_lines, reference_lines.target_line_id);
  if (current_line_ptr_ == nullptr || target_line_ptr_ == nullptr) return;
  AERROR_IF(FLAGS_log_enable) <<"current line id "<<reference_lines.current_line_id
                              <<" target line id "<<reference_lines.target_line_id;
  reference_lines.pathbox_in_current = PathBoxInLine(reference_lines, 
                                                     motionpath_.path, car_model_,
                                                     main_data_ptr_->vehicle_info.chassis.drive_state,
                                                     reference_lines.current_line_id);
  reference_lines.path_in_target = LocalPathInLine(reference_lines,
                                                   motionpath_.path,
                                                   reference_lines.target_line_id, 
                                                   reference_lines.current_line_id,
                                                   main_data_ptr_->vehicle_info.chassis.drive_state);
  reference_lines.path_in_current = LocalPathInLine(reference_lines,
                                                    motionpath_.path,
                                                    reference_lines.current_line_id, 
                                                    reference_lines.target_line_id,
                                                   main_data_ptr_->vehicle_info.chassis.drive_state);
  AERROR_IF(FLAGS_log_enable)<<"path_in_current "<<reference_lines.path_in_current
                             <<" path_in_target "<<reference_lines.path_in_target
                             <<" path box in current "<<reference_lines.pathbox_in_current;

  object_process_.CellsToObjects(perception_);
  object_process_.CompleteObjectInfo(perception_.objects, prediction_, allline_objects_);
  prediction_process_.AddPredictionInfo(perception_.timestamp, allline_objects_, prediction_);
  NewPathInfo local_line;
  bool has_local_flag = local_line.MotionPathToFrame(motionpath_, *target_line_ptr_, *current_line_ptr_,
                    localization_, car_model_, reference_lines.local_reference_line);
  
  if (FLAGS_use_openmp) {
    omp_set_num_threads(3);
    #pragma omp parallel sections
    {
      {// current
        AWARN_IF(FLAGS_log_enable)<<"--------------------current--------------------";
        for (auto &current_line : reference_lines.current_reference_line) {
          CalculateLineInfo(current_line, matched_lights_, allline_objects_, behavior_, 
                            motionpath_, localization_, car_model_, speedplan_config_, 
                            compensate_s_in_unit_m_, compensate_t_in_unit_s_);
        }
      }
      #pragma omp section
      {
        AWARN_IF(FLAGS_log_enable) <<"--------------------left--------------------";
        for (auto &left_line : reference_lines.left_reference_line) {
          CalculateLineInfo(left_line, matched_lights_, allline_objects_, behavior_, 
                            motionpath_, localization_, car_model_, speedplan_config_, 
                            compensate_s_in_unit_m_, compensate_t_in_unit_s_);
        }
        AWARN_IF(FLAGS_log_enable) <<"--------------------right--------------------";
        for (auto &right_line : reference_lines.right_reference_line) {
          CalculateLineInfo(right_line, matched_lights_, allline_objects_, behavior_, 
                            motionpath_, localization_, car_model_, speedplan_config_, 
                            compensate_s_in_unit_m_, compensate_t_in_unit_s_);
        }
      }
      #pragma omp section
      {
        AWARN_IF(FLAGS_log_enable) <<"--------------------local--------------------";
        if (has_local_flag) {
          auto &local_line = reference_lines.local_reference_line;
          AERROR_IF(FLAGS_log_enable)<<"new local size "<<local_line.mapinfo.path_points.size();
          CalculateLineInfo(local_line, matched_lights_, allline_objects_, behavior_, 
                            motionpath_, localization_, car_model_, speedplan_config_, 
                            compensate_s_in_unit_m_, compensate_t_in_unit_s_);
        }
      } 
    }                           
  } else {
    AWARN_IF(FLAGS_log_enable) <<"--------------------current--------------------";
    for (auto &line : reference_lines.current_reference_line) {
      if (!line.mapinfo.path_points.empty()) {
        CalculateLineInfo(line, matched_lights_, allline_objects_, behavior_, 
                          motionpath_, localization_, car_model_, speedplan_config_, 
                          compensate_s_in_unit_m_, compensate_t_in_unit_s_);                          
      }
    }
    AWARN_IF(FLAGS_log_enable) <<"--------------------left--------------------";
    for (auto &line : reference_lines.left_reference_line) {
      CalculateLineInfo(line, matched_lights_, allline_objects_, behavior_,  
                        motionpath_, localization_, car_model_, speedplan_config_, 
                        compensate_s_in_unit_m_, compensate_t_in_unit_s_);
    }
    AWARN_IF(FLAGS_log_enable) <<"--------------------right--------------------";
    for (auto &line : reference_lines.right_reference_line) {
      CalculateLineInfo(line, matched_lights_, allline_objects_, 
                          behavior_, motionpath_, localization_,
                          car_model_, speedplan_config_, 
                          compensate_s_in_unit_m_, compensate_t_in_unit_s_);
    }
    AWARN_IF(FLAGS_log_enable) <<"--------------------local--------------------";
    if (has_local_flag) {
      auto &local_line = reference_lines.local_reference_line;
      AERROR_IF(FLAGS_log_enable)<<"new local size "<<local_line.mapinfo.path_points.size();
      CalculateLineInfo(local_line, matched_lights_, allline_objects_, 
                        behavior_, motionpath_, localization_,
                        car_model_, speedplan_config_, 
                        compensate_s_in_unit_m_, compensate_t_in_unit_s_);
    }
  }

  double local_path_length = 1000.0;
  if (motionpath_.path.size()) {
    local_path_length = motionpath_.path.back().length;
  } 
  CalculateRevLineInfo(reference_lines.reverse_reference_line, allline_objects_, local_path_length, 
                       localization_,car_model_);
  MergeCurrentToLocal(*current_line_ptr_, reference_lines.local_reference_line, has_local_flag);
  reference_lines.task_change = mapengine_.map_info_data.task_change;
  AERROR_IF(FLAGS_log_enable)<<"reference_lines.task_change = "<<reference_lines.task_change;
  ScenarioEvaluation scenario;
  scenario.EvaluateScenario();
  DebugMsg();
}

void StructCognition::DebugMsg() {
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->debug_planning_msg.cognition.set_drive_state(drivestatus_.control_mode);
  DP->debug_planning_msg.cognition.mutable_reference_lane_ids()->Clear();
  DP->debug_planning_msg.cognition.mutable_reference_target_ids()->Clear();
  for(const auto& lane : last_current_lanes_){
    DP->debug_planning_msg.cognition.add_reference_lane_ids(lane);
  }
  for(const auto& lane : last_target_lanes_){
    DP->debug_planning_msg.cognition.add_reference_target_ids(lane);
  }
  DP->debug_planning_msg.cognition.set_mapengine_index(mapengine_.map_info_data.index);
  DP->debug_planning_msg.cognition.set_correction_index(correction_index_);
  DP->debug_planning_msg.cognition.set_target_index(target_index_);
  DP->debug_planning_msg.cognition.mutable_key_line_id()->Clear();
  DP->debug_planning_msg.cognition.add_key_line_id(main_data_ptr_->reference_line_info.current_line_id);
  DP->debug_planning_msg.cognition.add_key_line_id(main_data_ptr_->reference_line_info.target_line_id);
  DP->debug_planning_msg.cognition.set_in_current(main_data_ptr_->reference_line_info.path_in_current);
  DP->debug_planning_msg.cognition.set_in_target(main_data_ptr_->reference_line_info.path_in_target);
  DP->debug_planning_msg.cognition.mutable_current_line()->Clear();
  DP->debug_planning_msg.cognition.mutable_left_line()->Clear(); 
  DP->debug_planning_msg.cognition.mutable_right_line()->Clear();
  DP->debug_planning_msg.cognition.mutable_local_line()->Clear();
  DP->debug_planning_msg.stmap.mutable_st_points()->Clear();
  DP->debug_planning_msg.stmap.mutable_decision_st_points()->Clear();
  for (auto &line : main_data_ptr_->reference_line_info.current_reference_line) {
    planning_debug_msgs::DebugLine line_msg;
    LineDebug(line, line_msg);
    DP->debug_planning_msg.cognition.add_current_line()->CopyFrom(line_msg);
    if (line.reference_lane_id == main_data_ptr_->reference_line_info.current_line_id &&
        DP->debug_planning_msg.cognition.in_current()) {
      //for (const auto &t_points : line.GetSTMap()) {
        //for (const auto &st_point : t_points) {
      const int range_t = line.GetSTMapRangeT();
      const int range_s = line.GetSTMapRangeS();
      auto st_map_ptr = line.GetSTMapPtr();
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
          DP->debug_planning_msg.stmap.add_st_points()->CopyFrom(temp_point);
        }
      }
    }
  }

  for (auto &line : main_data_ptr_->reference_line_info.left_reference_line) {
    planning_debug_msgs::DebugLine line_msg;
    LineDebug(line, line_msg);
    DP->debug_planning_msg.cognition.add_left_line()->CopyFrom(line_msg);
  }
  for (auto &line : main_data_ptr_->reference_line_info.right_reference_line) {
    planning_debug_msgs::DebugLine line_msg;
    LineDebug(line, line_msg);
    DP->debug_planning_msg.cognition.add_right_line()->CopyFrom(line_msg);
  }
  if (!main_data_ptr_->reference_line_info.local_reference_line.mapinfo.path_points.empty()) {
    planning_debug_msgs::DebugLine line_msg;
    auto &local_line = main_data_ptr_->reference_line_info.local_reference_line;
    LineDebug(local_line, line_msg);
    DP->debug_planning_msg.cognition.add_local_line()->CopyFrom(line_msg);
    if (!DP->debug_planning_msg.cognition.in_current()) {
      // for (const auto &t_points : local_line.GetSTMap()) {
      //   for (const auto &st_point : t_points) {
      const int range_t = local_line.GetSTMapRangeT();
      const int range_s = local_line.GetSTMapRangeS();
      auto st_map_ptr = local_line.GetSTMapPtr();
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
          DP->debug_planning_msg.stmap.add_st_points()->CopyFrom(temp_point);
        }
      }
    }
  }

  DebugObjMsg object_local;
  if(!main_data_ptr_->reference_line_info.local_reference_line.mapinfo.path_points.empty()) {
    auto &local_line = main_data_ptr_->reference_line_info.local_reference_line;
    ObjectDebug(local_line, object_local, FLAGS_debug_id);
  }
  //DP->debug_planning_msg.object_debug_by_line.mutable_object_debug(0)->CopyFrom(object_local);

  DebugObjMsg object_current;
  if (!main_data_ptr_->reference_line_info.current_reference_line.empty()) {
    auto &line = main_data_ptr_->reference_line_info.current_reference_line.front();
    ObjectDebug(line, object_current, FLAGS_debug_id);
  }
  //DP->debug_planning_msg.object_debug_by_line.mutable_object_debug(1)->CopyFrom(object_current);
  DP->debug_planning_msg.cognition.set_time_cost_ms((acu::common::NodeTime::Now().ToSecond() - start_time_)*1000.0);
}

std::vector<DebugObjMsg> StructCognition::ObjectDebug(const ReferenceLineFrame &line,
    DebugObjMsg& debug_obj, int debug_id) {
  std::vector<DebugObjMsg> objects;
  for (auto &it : line.objects_) {
    auto &obj = it.second;
    DebugObjMsg obj_msg;
    obj_msg.set_id(obj.id);
    obj_msg.set_lane_id(obj.obj_lane_id);
    obj_msg.set_acc(obj.acc);
    obj_msg.set_speed(obj.speed);
    obj_msg.set_is_static(obj.is_static);
    obj_msg.set_was_dynamic(obj.was_dynamic);
    obj_msg.set_need_focus(obj.need_focus);
    obj_msg.set_is_reverse_traveling(obj.is_reverse_traveling);
    obj_msg.set_dis_to_junction(obj.dis_to_junction);
    obj_msg.set_cell_num(obj.cells.size());
    obj_msg.set_s_l_min_s(obj.sl_boundary.min_s);
    obj_msg.set_s_l_max_s(obj.sl_boundary.max_s);
    obj_msg.set_s_l_min_l(obj.sl_boundary.min_l);
    obj_msg.set_s_l_max_l(obj.sl_boundary.max_l);
    obj_msg.set_nearest_xg(obj.nearest_xg);
    obj_msg.set_nearest_yg(obj.nearest_yg);
    obj_msg.mutable_pd_objs()->Clear();
    int index = 0;
    for (auto &pd_line : obj.prediction.trajectories) {
      planning_debug_msgs::DebugPdObject pd_obj_msg;
      for(const auto& lane_id : pd_line.lane_ids){
        pd_obj_msg.add_lane_ids(lane_id);
      }
      pd_obj_msg.mutable_st()->Clear();
      pd_obj_msg.mutable_range_a()->Clear();
      pd_obj_msg.mutable_range_pd_s()->Clear();
      for (auto &pd_obj : obj.st_area.pd_objs) {
        if (pd_obj.prediction_index == index) {
          pd_obj_msg.set_conflict_type(pd_obj.conflict_type);
          pd_obj_msg.set_right_of_way(int(pd_obj.right_of_way));
          planning_debug_msgs::STpair pair_st;
          for (auto st_bd : pd_line.st_boundary) {
            pair_st.set_t(st_bd.first.y());
            pair_st.set_min_s(st_bd.first.x());
            pair_st.set_max_s(st_bd.second.x());
            pd_obj_msg.add_st()->CopyFrom(pair_st);
          } 
        }
      }
      obj_msg.add_pd_objs()->CopyFrom(pd_obj_msg);
      index++;
    }
    if(debug_id != -1 && obj.id == debug_id) debug_obj = obj_msg;
    objects.push_back(obj_msg);
  }
  return objects;
}

void StructCognition::LineDebug(const ReferenceLineFrame &line, planning_debug_msgs::DebugLine& line_msg) {
for(const auto& front_lane_id : line.mapinfo.front_lane_ids){
    line_msg.add_lane_ids(front_lane_id);
  }
  if (!line.mapinfo.expected_speeds.empty()) {
    line_msg.set_speed_limit(line.mapinfo.expected_speeds.front().second);
  }
  line_msg.set_dis2line(line.mapinfo.dis2line);
  line_msg.set_distance_to_mission_point(line.mapinfo.dis2missionpoint);
  line_msg.mutable_block_state()->set_str((line.line_blocked)? "block!" : "no block.");
  line_msg.mutable_block_state()->set_value(line.block_counter);
  line_msg.mutable_slow_state()->set_str((line.line_slow)? "slow!" : "no block.");
  line_msg.mutable_slow_state()->set_value(line.slow_counter);
  line_msg.set_global_cost(line.mapinfo.global_cost);
  line_msg.set_line_queue(line.line_queue);
  line_msg.set_speed_cost(line.speed_cost);
  line_msg.set_is_congestion(line.is_congestion);
  line_msg.set_long_term_speed(line.long_term_speed);
  line_msg.set_short_term_speed(line.short_term_speed);
  line_msg.mutable_gaps()->Clear();
  for (auto &gap : line.line_gap) {
    planning_debug_msgs::DebugGap gap_msg;
     gap_msg.set_start_id(gap.start_id);
    gap_msg.set_end_id(gap.end_id);
    gap_msg.set_aim_min_s(gap.aim_min_s);
    gap_msg.set_aim_max_s(gap.aim_max_s);
    gap_msg.set_allow_min_v(gap.allow_min_v);
    gap_msg.set_allow_max_v(gap.allow_max_v);
    gap_msg.set_allow_min_t(gap.allow_min_t);
    gap_msg.set_feasibility_level(gap.feasibility_level);
    gap_msg.set_safety_level(gap.safety_level);
    line_msg.add_gaps()->CopyFrom(gap_msg);
  }
  DebugObjMsg debug_obj;
  for(const auto& obj : ObjectDebug(line, debug_obj)){
    line_msg.add_objs()->CopyFrom(obj);
  }
}




} // namespace planning
} // namespace acu
