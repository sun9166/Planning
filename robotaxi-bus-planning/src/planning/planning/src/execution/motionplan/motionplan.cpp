/**
 * @file motionplan.cpp
 **/
#include <iostream>
#include <future>
#include <memory>

#include "motionplan.h"
#include "common/util/ros_util.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "common/base/time/include/rate.h"
#include "common/common_define/error_code_define.h"
//#include "common/common_header/fault_vector_handler.h"
#include "common/base/thread_pool/include/thread_pool.h"
#include "common/mapcheck/geotool/include/coordtransform.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "datapool/include/data_pool.h"
#include "gflags/gflags.h"
#include "src/execution/cognition/struct_cognition/conf/cognition_gflags.h"

namespace acu {
namespace planning {

MotionPlan::MotionPlan() {

}

MotionPlan::~MotionPlan() {
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
  Reset();
}

int MotionPlan::Init() {
  //std::string planning_pkg_path;
  //acu::common::util::getPackagePath("planning", planning_pkg_path);
  std::string flag_file_path = "/map/work/config/planning_node_config/motionplan_conf/motionplan.conf";
  //flag_file_path.assign(common::util::GetAbsolutePath(planning_pkg_path, flag_file_path));
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  flag_file_path = "/map/work/config/planning_node_config/cognition_conf/cognition.conf";
  //flag_file_path.assign(common::util::GetAbsolutePath(planning_pkg_path, flag_file_path));
  google::SetCommandLineOption("flagfile", flag_file_path.c_str());

  path_planner_ = std::unique_ptr<PathPlanner>(new PathPlanner());
  speed_planner_ = std::unique_ptr<SpeedPlanner>(new SpeedPlanner());
  reference_line_provider_ = std::unique_ptr<ReferenceLineProvider>(new ReferenceLineProvider());
  reference_line_provider_->Start();
  seq_num_ = 0 ;
  std::string planning_config_file = "/map/work/config/planning_node_config/motionplan_conf/path_planning_config.config";
  PlanningConfig path_plan_config;
  //planning_config_file.assign(common::util::GetAbsolutePath(planning_pkg_path, planning_config_file));
  CHECK(acu::common::util::GetProtoFromFile(planning_config_file,
        &path_plan_config))
      << "failed to load planning config file " << planning_config_file;
  path_planner_->Init(path_plan_config);
  planning_config_file = "/map/work/config/planning_node_config/motionplan_conf/speed_planning_config.config";
  PlanningConfig speed_plan_config;
  //planning_config_file.assign(common::util::GetAbsolutePath(planning_pkg_path, planning_config_file));
  CHECK(acu::common::util::GetProtoFromFile(planning_config_file,
        &speed_plan_config))
      << "failed to load planning config file " << planning_config_file;

  speed_planner_->Init(speed_plan_config);

  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  EgoInfo::instance()->SetVehicleParams(DP->config_info.car_model);
  return 0;
}

void MotionPlan::Process() {
  AINFO<<"------------[MotionplanProcess] start------------------";
  const double planning_start_timestamp = acu::common::NodeTime::Now().ToSecond();
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason("");
  DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_target_dis(0.0);
  DP_ptr->debug_planning_msg.motionplan_debug.set_jerk_weight_factor(10);
  DP_ptr->debug_planning_msg.motionplan_debug.set_acc_dcc_mode(-1);
  DP_ptr->debug_planning_msg.motionplan_debug.set_follow_dis_error(-1);
  DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason("");
  DP_ptr->debug_planning_msg.motionplan_debug.set_cog_linear_velocity(-1);
  DP_ptr->debug_planning_msg.motionplan_debug.set_dis_to_virtual_wall(-1);
  double timestamp_s = DP_ptr->cognition_info.struct_env_info.vehicle_info.localization.time_stamp;
  
  //1.update vehicle state
  StructLocalizationInfo newest_loc;
  StructVehicleInfo vehicle_info = DP_ptr->cognition_info.struct_env_info.vehicle_info;
  int new_imu_index = DP_ptr->loc_perception.new_imu_index;
  if (!DP_ptr->loc_perception.loc_data_list.empty()
      && DP_ptr->loc_perception.loc_data_list.size() > new_imu_index){
    newest_loc.time_stamp = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).time;
    newest_loc.xg = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).xg;
    newest_loc.yg = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).yg;
    newest_loc.global_angle = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).yaw;
    newest_loc.dr_x = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).loc_xg_dr;
    newest_loc.dr_y = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).loc_yg_dr;
    newest_loc.dr_angle = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).loc_yaw_dr;
    newest_loc.loc_velocity = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).velocity;
    newest_loc.angular_velocity = DP_ptr->loc_perception.loc_data_list.at(new_imu_index).yawrate;
  } else {
    newest_loc = DP_ptr->cognition_info.struct_env_info.vehicle_info.localization;
  }
  if (!EgoInfo::instance()->Update(vehicle_info, newest_loc)) {
    string stop_reason_msg = "update EgoInfo failed, GenerateStopTrajectory!";
    AERROR<<stop_reason_msg;
    DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason(stop_reason_msg);
    GenerateStopTrajectory();
    return;
  }

  VehicleState vehicle_state = EgoInfo::instance()->vehicle_state();
  if (vehicle_state.driving_mode == 1) {
    ManualDriveReset();
  }

  if (vehicle_state.driving_mode == 1 && false) {//manual drive mode
    Reset();
    if (FLAGS_enable_generate_trajectory_manual_drive_mode) {
      return GenerateTrajectoryBasedCurrentReferenceLine();
    } else {
      string stop_reason_msg = "Manual drive mode, GenerateStopTrajectory!";
      AERROR<<stop_reason_msg;
      DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason(stop_reason_msg);
      return GenerateStopTrajectory();
    } 
  }
  
  //2.behavior_sentence Parser
  const auto& DP = acu::planning::DataPool::Instance()->GetMainDataRef();
  if (!BehaviorParser::instance()->Update(
        DP.decision_info, &DP.cognition_info.struct_env_info.reference_line_info)) {
    string stop_reason_msg = BehaviorParser::instance()->error_msg() + ", GenerateStopTrajectory!";
    AERROR<<stop_reason_msg;
    DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason(stop_reason_msg);
    GenerateStopTrajectory();
    if (reference_line_provider_ 
      && BehaviorParser::instance()->error_msg() == "cannot find target reference line.") {
      reference_line_provider_->Stop();
    }
    return;
  }

  DP_ptr->debug_planning_msg.motionplan_debug.set_behavior_parser_time_spend( 
                      1000 * (acu::common::NodeTime::Now().ToSecond() - planning_start_timestamp));
  const double trajectorystitcher_start_timestamp = acu::common::NodeTime::Now().ToSecond();

  bool reverse_flag = !BehaviorParser::instance()->reference_lines_data().empty()
    && !BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.empty()
    && BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.front().reverse;
  if ( !reverse_flag == reverse_flag_ ) {
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
  }
  AERROR_IF(FLAGS_enable_debug_motion)<<"reverse_flag "<<reverse_flag<<" reverse_flag_ "<<reverse_flag_;
  reverse_flag_ = reverse_flag;
  //3.calculate stitching_trajectory
  double planning_period = 0.1;
  int replan_reason = 0;
  VehicleState vehicle_state_new = EgoInfo::instance()->vehicle_state_new();
  std::vector<common::TrajectoryPoint> stitching_trajectory;
  std::vector<common::TrajectoryPoint> stitching_trajectory_for_speedplan;
  size_t speedplan_stitching_trajectory_index = 0;
  local_reference_line_ = DP.cognition_info.struct_env_info.reference_line_info.local_reference_line;
  if (FLAGS_enable_two_stitching_trajectory) {
    stitching_trajectory_for_speedplan = trajectory_stitcher::ComputeStitchingTrajectoryForSpeedPlan(
                                      vehicle_state, vehicle_state_new, timestamp_s, planning_period,
                                      last_publishable_trajectory_.get(), &replan_reason);
    if ((BehaviorParser::instance()->has_lateral_behavior() 
        || !NewPathManager::instance()->stitch_state()
        || vehicle_state_new.brake_state == 1)
        && FLAGS_enable_const_length_stitching_trajectory) {
      stitching_trajectory = trajectory_stitcher::ComputeStitchingTrajectoryForPathPlan(
                                      vehicle_state, vehicle_state_new, timestamp_s, planning_period,
                                      last_publishable_trajectory_.get(),local_reference_line_, &replan_reason);
    } else {
      stitching_trajectory = stitching_trajectory_for_speedplan;
    }
    speedplan_stitching_trajectory_index = stitching_trajectory_for_speedplan.size();
  } else {
    stitching_trajectory = trajectory_stitcher::ComputeStitchingTrajectory(
                                      vehicle_state, vehicle_state_new, timestamp_s, planning_period,
                                      last_publishable_trajectory_.get(), &replan_reason);
    speedplan_stitching_trajectory_index = stitching_trajectory.size();
    stitching_trajectory_for_speedplan = stitching_trajectory;
  }
  if (FLAGS_generate_DRpath_type != 0 ) {
    trajectory_stitcher::FixStitchingTrajectoryByDR(vehicle_state, vehicle_state_new, stitching_trajectory_for_speedplan);
    trajectory_stitcher::FixStitchingTrajectoryByDR(vehicle_state, vehicle_state_new, stitching_trajectory);
  }
  
  DP_ptr->debug_planning_msg.motionplan_debug.set_trajectory_stitcher_time_spend( 
                      1000 * (acu::common::NodeTime::Now().ToSecond() - trajectorystitcher_start_timestamp));
  const double frame_start_timestamp = acu::common::NodeTime::Now().ToSecond();                    

  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);
  
  BehaviorParser::instance()->GetDecisionSpeedlimit(
        DP.decision_info.giveway_id, DP.decision_info.speed_limit, stitching_trajectory.back().v());
  BehaviorParser::instance()->GetRemoteDriveInfo(stitching_trajectory.back());

  //4.ego_info update
  bool update_ego_info =
    EgoInfo::instance()->Update(stitching_trajectory.back());

  //5.Frameinit
  reference_line_provider_->UpdateVehicleState(vehicle_state);
  auto init_status = InitMotionPlanFrame(frame_num, stitching_trajectory, stitching_trajectory_for_speedplan,
                     timestamp_s,vehicle_state);
  if (!init_status.ok()) {
    string stop_reason_msg = init_status.error_message() + ", GenerateStopTrajectory";
    AERROR<<stop_reason_msg;
    DP_ptr->debug_planning_msg.motionplan_debug.set_generate_stop_trajectory_reason(stop_reason_msg);
    GenerateStopTrajectory();
    const uint32_t n = frame_->SequenceNum();
    FrameHistory::instance()->Add(n, std::move(frame_));
    AINFO_IF(FLAGS_enable_debug_motion)<<"------------[MotionplanProcess] end for Init failed Frame------------------";
    return ;
  }

  DP_ptr->debug_planning_msg.motionplan_debug.set_frame_time_spend(
    1000 * (acu::common::NodeTime::Now().ToSecond() - frame_start_timestamp));
                        
  //6.Plan
  if (FLAGS_enable_two_stitching_trajectory) {
    auto pathplan_status = path_planner_->Plan(stitching_trajectory_for_speedplan.back(), frame_.get());
    auto speedplan_status = speed_planner_->Plan(stitching_trajectory_for_speedplan.back(), frame_.get());
  } else {
    auto pathplan_status = path_planner_->Plan(stitching_trajectory.back(), frame_.get());
    auto speedplan_status = speed_planner_->Plan(stitching_trajectory.back(), frame_.get());
  }
  
  const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
  if (!best_ref_info) { 
    std::string msg("planner failed to make a driving plan");
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    if (last_publishable_trajectory_) {
      last_publishable_trajectory_->Clear();
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"------------[MotionplanProcess] end for not find ReferenceLineInfo------------------";
    return ;
  }

  //7.save trajectory as last_publishable_trajectory_;
  last_publishable_trajectory_.reset(new PublishableTrajectory(
                                       timestamp_s, best_ref_info->trajectory()));
  if (FLAGS_enable_two_stitching_trajectory) {
    last_publishable_trajectory_->PrependTrajectoryPoints(
      std::vector<common::TrajectoryPoint>(stitching_trajectory_for_speedplan.begin(),
                                   stitching_trajectory_for_speedplan.end() - 1));
  } else {
    last_publishable_trajectory_->PrependTrajectoryPoints(
      std::vector<common::TrajectoryPoint>(stitching_trajectory.begin(),
                                   stitching_trajectory.end() - 1));
  }
  
  last_publishable_trajectory_->set_trajectory_type(best_ref_info->trajectory_type());
  frame_->set_current_publishable_trajectory(*(last_publishable_trajectory_.get()));

  //8.generate plan info
  
  double start_generate_trajectory = acu::common::NodeTime::Now().ToSecond();
  GenerateTrajectory();

  DP_ptr->debug_planning_msg.motionplan_debug.set_generate_trajectory_time_spend(
                       1000 * (acu::common::NodeTime::Now().ToSecond() - start_generate_trajectory));  
  
  // 9.Add debug info
  AddDebugInfo(replan_reason);
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"------------[MotionplanProcess] end------------------";
  const uint32_t n = frame_->SequenceNum();
  FrameHistory::instance()->Add(n, std::move(frame_));

  DP_ptr->debug_planning_msg.motionplan_debug.set_total_time  
                  (1000 * (acu::common::NodeTime::Now().ToSecond() - planning_start_timestamp)) ;
}

acu::common::Status MotionPlan::InitMotionPlanFrame(const uint32_t sequence_num,
                                   const std::vector<common::TrajectoryPoint>& stitching_trajectory,
                                   const std::vector<common::TrajectoryPoint>& stitching_trajectory_speedplan,
                                   const double start_time,
                                   const VehicleState& vehicle_state) {
  double time_stamp_1 = acu::common::NodeTime::Now().ToSecond(); 
  if (FLAGS_enable_two_stitching_trajectory) {
    frame_.reset(new Frame(sequence_num, stitching_trajectory,stitching_trajectory_speedplan,
                         last_publishable_trajectory_type_,
                         start_time, vehicle_state));
  } else {
    frame_.reset(new Frame(sequence_num, stitching_trajectory,stitching_trajectory,
                         last_publishable_trajectory_type_,
                         start_time, vehicle_state));
  }
  
  if (frame_ == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Fail to init frame ";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "frame_ is nullptr, reset failed.");
  }

  if (BehaviorParser::instance()->reference_lines_data().size() < 1 ) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"refrence paths size less than 1";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "refrence paths size less than one.");
  } else if (BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.size() < 3) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"points of refrence path size less than 3";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "points of refrence path size less than 3.");
  }
  AERROR<<"IsNeedExtend????????? "<<BehaviorParser::instance()->target_reference_line().mapinfo.IsNeedExtend();
  if (FLAGS_enable_smooth_reference_line && !BehaviorParser::instance()->target_reference_line().mapinfo.IsNeedExtend() ) {
    std::list<ReferenceLine> reference_lines;
    bool pathplan_status_last = true;
    const auto* last_frame = FrameHistory::instance()->Latest();//
    if (!last_frame) {
      pathplan_status_last = true;
    } else {
      const ReferenceLineInfo* last_reference_line_info =
                 last_frame->DriveReferenceLineInfo();
      if (!last_reference_line_info) {
        pathplan_status_last = true;
      } else {
        pathplan_status_last = last_reference_line_info->path_fallback_reason().empty()? true : false;// failed : false
      AINFO_IF(FLAGS_enable_debug_motion)<<"pathplan_status_last = "<<pathplan_status_last;
      }
    }
    
    if (!reference_line_provider_->GetReferenceLines(&reference_lines,
                            &BehaviorParser::instance()->reference_lines_data().front(), pathplan_status_last)) {
      const std::string msg = "Failed to create reference line";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
    auto status = frame_->Init(reference_lines, BehaviorParser::instance()->reference_lines_data());
    if (!status) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"failed to init frame"  ;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "failed to init frame");
    }
    return Status::OK();
  }
  

  auto status = frame_->Init(BehaviorParser::instance()->reference_lines_data());
  if (!status) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"failed to init frame"  ;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "failed to init frame");
  }
  return Status::OK();
}

void MotionPlan::GenerateStopTrajectory() {
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->motion_path.path.clear();
  const auto& vehicle_state = EgoInfo::instance()->vehicle_state();

  PathData trajectory;
  trajectory.time_stamp = acu::common::NodeTime::Now().ToSecond();
  trajectory.is_new = false;
  trajectory.current_id = 
      BehaviorParser::instance()->current_reference_line_id();//TODO   
  trajectory.target_id = 
      BehaviorParser::instance()->target_reference_line_id(); //TODO  
  trajectory.is_blocked = false ;  //TODO

  geometry::Site point;
  point.x = 0;
  point.y = 0;
  point.angle = 0;
  point.xg = vehicle_state.x;
  point.yg = vehicle_state.y;
  point.globalangle = vehicle_state.heading * kRAD_TO_DEG;
  point.velocity = 0.0;
  point.index = 0;
  point.origin_index = 0;
  point.length = 0.0;
  point.reverse = reverse_flag_;

  trajectory.path.push_back(point);

  // pull result trajectory to datapool
  DP->motion_path.path.push_back(point); 
  last_publishable_trajectory_type_ = 0;
}

void MotionPlan::GenerateTrajectoryBasedCurrentReferenceLine() {
  auto DP = DataPool::Instance()->GetMainDataPtr();
  DP->motion_path.path.clear();
  if (DP->cognition_info.struct_env_info.reference_line_info.current_reference_line.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"cognition current_reference_line is empty. ";
    return;
  }
  for (auto& point : DP->cognition_info.struct_env_info.reference_line_info.current_reference_line.front().mapinfo.path_points) {
    geometry::Site trajectory_point;
    trajectory_point = point;
    trajectory_point.velocity = 0.0;
    DP->motion_path.path.push_back(trajectory_point); 
  }
  last_publishable_trajectory_type_ = 0;
}

void MotionPlan::GenerateTrajectory() {
  auto best_ref_info = frame_->DriveReferenceLineInfo();
  if (best_ref_info == nullptr) {
    AERROR<<"No best reference_line !! ";
    return;
  }

  std::vector<std::string> target_lane_ids = best_ref_info->TargetLaneId();
  double property_point_length = best_ref_info->trajectory().property_length();
  double offset_property_length = best_ref_info->trajectory().offset_property_length();
  bool is_new_path = best_ref_info->path_data().is_new();
  AINFO_IF(FLAGS_enable_debug_motion)<<" property_point_length : "<< property_point_length;
  
  auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();
  DP->motion_path.path.clear();
  bool is_parking = reverse_flag_;
  PathData output_trajectory;
  geometry::SiteVec trajectory_new;
  output_trajectory.is_new = is_new_path;
  output_trajectory.current_id = 
      BehaviorParser::instance()->current_reference_line_id();   
  output_trajectory.target_id = 
      BehaviorParser::instance()->target_reference_line_id();   
  output_trajectory.is_blocked = false ;  
  output_trajectory.lane_ids = target_lane_ids;//for predicition
  geometry::Site trajectory_point;
  geometry::Site localization;
  localization.xg = EgoInfo::instance()->vehicle_state().x;
  localization.yg = EgoInfo::instance()->vehicle_state().y;
  localization.globalangle = EgoInfo::instance()->vehicle_state().heading * kRAD_TO_DEG;
  common::math::Vec2d last_point(0.0, 0.0); //TODO 
  int count = 0;
  int count_point_num = 0;
  double zero_s = 0;
  bool is_need_point_property = property_point_length > 0 ? true : false;
  bool is_need_offset_property = offset_property_length >= 0 ? true : false;
  AINFO_IF(FLAGS_enable_debug_motion)<<" is_need_offset_property ?"<<is_need_offset_property<<", offset_property_length"<<offset_property_length;
  std::vector<std::pair<double ,double>> kappa_t_pairs;
  double kappa_t_pairs_start_t = 0.8;
  bool need_accuracy_control = false;
  if (!BehaviorParser::instance()->reference_lines_data().empty()) {
    const auto& ref_path = BehaviorParser::instance()->reference_lines_data().front();
     need_accuracy_control = !ref_path.mapinfo.lane_types.empty()
            && ref_path.mapinfo.lane_types.front().second == eLaneType::PARKING_LANE
              || ref_path.mapinfo.dis_to_end < 30.0
                 &&(ref_path.mapinfo.dis2missionpoint - ref_path.mapinfo.dis_to_end) > 2.5;
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<" is_need_offset_property ?"
    <<is_need_offset_property<<", offset_property_length"<<offset_property_length
    <<" need_accuracy_control "<<need_accuracy_control;
  bool has_zero_point = false;
  for (const auto& point : *last_publishable_trajectory_) {
    trajectory_point.xg = point.path_point().x();
    trajectory_point.yg = point.path_point().y();
    trajectory_point.globalangle = point.path_point().theta() * kRAD_TO_DEG; //angleglobal为角度，需要进行弧度转角度的转化
    double x = 0.0, y = 0.0;
    common::math::Vec2d current_point(trajectory_point.xg, trajectory_point.yg);
    CoordTransform *coor_transform = CoordTransform::Instance(); 
    geometry::Site point_vcs;
    coor_transform->GCCS2VCS(localization, trajectory_point, point_vcs);
    if ( (current_point - last_point).Length() < 0.05) { //不pub密度特别小的点
      if (point.v() <= 0.01 && !has_zero_point) {
        AERROR_IF(FLAGS_enable_debug_motion)<<"zero point!!! s = "<<point.path_point().s();
      } else {
        continue;
      }
    }
    if (count <= 0) {
      trajectory_point.length = 0;
      zero_s = point.path_point().s();
      count++;
    } else {
      trajectory_point.length = point.path_point().s() - zero_s;
    }
    last_point = current_point;

    if (is_need_point_property) {
      trajectory_point.property = 
            trajectory_point.length > property_point_length ? 0 : 4;
    }
    if (is_need_offset_property) {
      trajectory_point.offset_property = 
            trajectory_point.length >= offset_property_length ? 2 : 1;
    }
    trajectory_point.x = point_vcs.x;
    trajectory_point.y = point_vcs.y;
    trajectory_point.angle = point_vcs.angle;
    trajectory_point.dr_x = point.path_point().dr_x();
    trajectory_point.dr_y = point.path_point().dr_y();
    trajectory_point.dr_angle = point.path_point().dr_theta() * 180.0/M_PI;
    trajectory_point.velocity = point.v() <= 0.01 || (has_zero_point && point.v() < 0.2) ? 0.00: point.v();//
    if (!has_zero_point && trajectory_point.velocity <= 0.01) {//debug
      has_zero_point = true;
    }
    trajectory_point.a = point.a();
    trajectory_point.t = point.relative_time();
    trajectory_point.curvature = point.path_point().kappa();
    trajectory_point.reverse = is_parking;
    output_trajectory.path.push_back(trajectory_point);
    if (trajectory_point.t >= kappa_t_pairs_start_t && kappa_t_pairs.size() < 4) {
      kappa_t_pairs_start_t += 0.1;
      kappa_t_pairs.push_back(std::make_pair(trajectory_point.curvature,trajectory_point.t));
    } 
    if (is_new_path || property_point_length > 0) {
      trajectory_new.push_back(trajectory_point);
    }
    ++count_point_num;
  }
  if (output_trajectory.path.size()) {
    EgoInfo::instance()->UpdateLastPathLength(output_trajectory.path.back().length);
    AWARN_IF(FLAGS_enable_debug_motion)<<"last_length "<<output_trajectory.path.back().length;
  }
  //TODO avoid stop->start scenario cannot start
  bool is_stop_scenario = true;
  if (!output_trajectory.path.empty()) {
    if (output_trajectory.path.back().length > 1) {
      std::list<geometry::Site>::iterator target_index;
      for (target_index = output_trajectory.path.begin(); ;++target_index) {
        if ((*target_index).velocity > 0.1 && (*target_index).length <= 1) {
          is_stop_scenario = false;
          break;
        }
        if ((*target_index).length > 1) break;
      }

      if (!is_stop_scenario && target_index != output_trajectory.path.begin()) {
        std::list<geometry::Site>::reverse_iterator it(++target_index);
        for (; it != output_trajectory.path.rend(); ++it) {
          if ((*it).velocity < 0.1 && (*it).length <= 1) {
            (*it).velocity = 0.1;
          }
        }
      }
    }
  }

  if (!has_zero_point) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"there are no zero point!! is_stop_scenario = "<<is_stop_scenario;
  }

  count_point_num = 0;
  for(auto& point : output_trajectory.path){
    point.velocity = is_parking? std::fmin(point.velocity, 3.0/3.6) : point.velocity;
    point.reverse = is_parking;
    AINFO_IF(FLAGS_enable_debug_motion && count_point_num < 10)<<std::fixed
      <<"vel["<<count_point_num<<"] "<<point.velocity;
      //<<" len "<<point.length
      //<<" xg "<<point.xg
      //<<" yg "<<point.yg
      //<<" angle "<<point.angle
      //<<" globalangle "<<point.globalangle
      //<<" t "<<point.t
      //<<" a "<<point.a;
    ++count_point_num;
  }
  DP->motion_path.time_stamp = last_publishable_trajectory_->header_time();
  DP->motion_path.path.assign(output_trajectory.path.begin(), output_trajectory.path.end());
  DP->motion_path.lane_ids = output_trajectory.lane_ids;
  DP->motion_path.is_new = output_trajectory.is_new ;
  DP->motion_path.current_id = output_trajectory.current_id ;  
  DP->motion_path.target_id = output_trajectory.target_id ;     
  DP->motion_path.is_blocked = output_trajectory.is_blocked;  
  AERROR_IF(FLAGS_enable_debug_motion)<<"kappa_t_pairs size = "<<kappa_t_pairs.size();
  DP->motion_path.steeringangle_rate_max = std::fmin(400, CalculateSteeringAngleRateMax(kappa_t_pairs)); 
  if (is_parking || frame_->DriveReferenceLineInfo()->reference_line().GetMapPath().is_parking_path()
    || need_accuracy_control) {
    DP->motion_path.control_accuracy = FLAGS_parking_control_accuracy;
  } else {
    DP->motion_path.control_accuracy = FLAGS_common_control_accuracy;
  }
  std::pair<double, double> u_turn_interval = std::make_pair(0.0,0.0);
  bool is_u_turn = BehaviorParser::instance()->target_reference_line().IsUturn(u_turn_interval);
  std::string path_type = (is_u_turn && u_turn_interval.first <= 11.0) ? "UTurn" : "straight";
  DP->motion_path.path_type = path_type;
  last_publishable_trajectory_type_= 
               (is_need_point_property || is_need_offset_property) ? 1:0;
  if (is_new_path && !trajectory_new.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"set new local path.";
    NewPathManager::instance()->SetNewPath(trajectory_new, BehaviorParser::instance()->local_path_type());
  } else if (!trajectory_new.empty() && 
    (property_point_length > 0 && (int)best_ref_info->trajectory_type() == 2)) {
    NewPathManager::instance()->SetNewPath(trajectory_new);//不再set局部路径的类型，保持上一帧的类型。
  }
  
  if(FLAGS_enable_debug_motion) DebugTrajectory(output_trajectory);
}

void MotionPlan::AddDebugInfo(const int replan_reason) const {
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  auto best_ref_info = frame_->DriveReferenceLineInfo();

  DP_ptr->debug_planning_msg.sl_boundaries.clear();
  DP_ptr->debug_planning_msg.xt_bounds.clear();

  for(const auto& boundary : frame_->reference_line_info().GetCandidatePathBoundaries()){
    std::vector<std::pair<double, double>> boundary_;
    //不满足多条reference_line_info的场景，目前只有一条
    auto cardesain_boundary = boundary.TransformBoundaryToMsg(frame_->reference_line_info().GetModifiedPathBoundaries());
    if(0 == frame_->reference_line_info().AssaignCartesianValues(cardesain_boundary)){
      DP_ptr->debug_planning_msg.sl_boundaries.push_back(std::move(cardesain_boundary));
    }else{
      AWARN_IF(FLAGS_enable_debug_motion)<<"AssaignCartesianValues failed";
    }
  }
  //should be only one st_graph of single reference_line_info?
  //2 st_graphs: including DP and QP
  if(!frame_->reference_line_info().debug().planning_data().st_graph().empty()){
    auto single_st_graph = *(frame_->reference_line_info().debug().planning_data().st_graph().rbegin());
    planning_debug_msgs::XTBounds single_xt_bounds;
    //s_constraint
    CHECK_EQ(single_st_graph.s_constraint().t().size(), single_st_graph.s_constraint().lower_bound().size());
    CHECK_EQ(single_st_graph.s_constraint().upper_bound().size(), single_st_graph.s_constraint().lower_bound().size());
    for(int i = 0; i < single_st_graph.s_constraint().t().size(); ++ i){
      planning_debug_msgs::DebugXTBound single_st_bound;
        single_st_bound.set_t(single_st_graph.s_constraint().t()[i]);
        single_st_bound.set_min_bound(single_st_graph.s_constraint().lower_bound()[i]);
        single_st_bound.set_max_bound(single_st_graph.s_constraint().upper_bound()[i]);
        single_xt_bounds.add_st_bounds()->CopyFrom(std::move(single_st_bound));
    }
    //speed_constraint
    CHECK_EQ(single_st_graph.speed_constraint().t().size(), single_st_graph.speed_constraint().lower_bound().size());
    CHECK_EQ(single_st_graph.speed_constraint().upper_bound().size(), single_st_graph.speed_constraint().lower_bound().size());
    for(int i = 0; i < single_st_graph.speed_constraint().t().size(); ++ i){
      planning_debug_msgs::DebugXTBound single_st_bound;
      single_st_bound.set_t(single_st_graph.speed_constraint().t()[i]);
      single_st_bound.set_min_bound(single_st_graph.speed_constraint().lower_bound()[i]);
      single_st_bound.set_max_bound(single_st_graph.speed_constraint().upper_bound()[i]);
      single_xt_bounds.add_dst_bounds()->CopyFrom(std::move(single_st_bound));
    }
    //a_constraint
    CHECK_EQ(single_st_graph.a_constraint().t().size(), single_st_graph.a_constraint().lower_bound().size());
    CHECK_EQ(single_st_graph.a_constraint().upper_bound().size(), single_st_graph.a_constraint().lower_bound().size());
    for(int i = 0; i < single_st_graph.a_constraint().t().size(); ++ i){
      planning_debug_msgs::DebugXTBound single_st_bound;
      single_st_bound.set_t(single_st_graph.a_constraint().t()[i]);
      single_st_bound.set_min_bound(single_st_graph.a_constraint().lower_bound()[i]);
      single_st_bound.set_max_bound(single_st_graph.a_constraint().upper_bound()[i]);
      single_xt_bounds.add_ddst_bounds()->CopyFrom(std::move(single_st_bound));
    }
    DP_ptr->debug_planning_msg.xt_bounds.push_back(std::move(single_xt_bounds));
  }

  DP_ptr->debug_planning_msg.motion_obstacle_vec_msg.mutable_motion_obstacles()->Clear();
  for(const auto& o : frame_->GetObstacleList()->Items()){
    if(!o->IsVirtual()){continue;}
      auto obs_msg = o->TransformToMontionObsMsg();
      DP_ptr->debug_planning_msg.motion_obstacle_vec_msg.add_motion_obstacles()->CopyFrom(obs_msg);
  }
  if (best_ref_info == nullptr) {
    planning_debug_msgs::DebugMotionPlan reset_debugs_msgs;
    DP_ptr->debug_planning_msg.motionplan_debug = reset_debugs_msgs;
    return;
  }
  DP_ptr->debug_planning_msg.motionplan_debug.set_locpos_s(best_ref_info->car_sl().s());
  DP_ptr->debug_planning_msg.motionplan_debug.set_is_new_path(best_ref_info->path_data().is_new());
  DP_ptr->debug_planning_msg.motionplan_debug.set_cruise_speed_max(best_ref_info->reference_line().cruise_speed());
  if (best_ref_info->trajectory_type() == ADCTrajectory::PATH_FALLBACK ||
        best_ref_info->trajectory_type() ==
            ADCTrajectory::SPEED_FALLBACK) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_is_fallback_trajectory(true);
  } else {
    DP_ptr->debug_planning_msg.motionplan_debug.set_is_fallback_trajectory(false);
  }
  if (best_ref_info->path_decision().MainFocusObstacle() != nullptr) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_id( 
            best_ref_info->path_decision().MainFocusObstacle()->PerceptionId());
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_v(
            best_ref_info->path_decision().MainFocusObstacle()->speed());
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_dis(
            best_ref_info->path_decision().MainFocusObstacle()->PerceptionSLBoundary().start_s());
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_a(
            best_ref_info->path_decision().MainFocusObstacle()->acc()); 
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_min_s(
            best_ref_info->path_decision().MainFocusObstacle()->st_boundary().min_s());
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_min_t(
            best_ref_info->path_decision().MainFocusObstacle()->st_boundary().min_t());                                
  } else {
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_id(0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_v(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_dis(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_min_s(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_following_object_min_t(0.0);
  }

  DP_ptr->debug_planning_msg.motionplan_debug.set_path_fallback_reason(best_ref_info->path_fallback_reason());
  DP_ptr->debug_planning_msg.motionplan_debug.set_speed_fallback_reason(best_ref_info->speed_fallback_reason());
  DP_ptr->debug_planning_msg.motionplan_debug.set_trajectory_type((int)best_ref_info->trajectory_type());
  if (best_ref_info->path_decision().MainYieldObstacle() != nullptr) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_id(
            best_ref_info->path_decision().MainYieldObstacle()->PerceptionId());
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_v(
            best_ref_info->path_decision().MainYieldObstacle()->speed());
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_current_s( 
            best_ref_info->path_decision().MainYieldObstacle()->PerceptionSLBoundary().start_s());
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_collision_min_s( 
            best_ref_info->path_decision().MainYieldObstacle()->st_boundary().min_s());
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_collision_min_t(
            best_ref_info->path_decision().MainYieldObstacle()->st_boundary().min_t());                
  } else {
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_id(0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_v(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_current_s(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_collision_min_s(0.0);
    DP_ptr->debug_planning_msg.motionplan_debug.set_yield_object_collision_min_t(0.0);
  }
  DP_ptr->debug_planning_msg.motionplan_debug.set_is_replan(replan_reason > 0? true:false);
  DP_ptr->debug_planning_msg.motionplan_debug.set_replan_reason(replan_reason);
  if (!DP_ptr->motion_path.path.empty()) {
    common::math::Vec2d latest_position(EgoInfo::instance()->vehicle_state_new().x, EgoInfo::instance()->vehicle_state_new().y);
    auto index = last_publishable_trajectory_->QueryNearestPointWithBuffer(latest_position,1.0e-6);
    auto latest_point = last_publishable_trajectory_->TrajectoryPointAt(index);
    DP_ptr->debug_planning_msg.motionplan_debug.set_v(latest_point.v());
    DP_ptr->debug_planning_msg.motionplan_debug.set_kappa(latest_point.path_point().kappa());
    DP_ptr->debug_planning_msg.motionplan_debug.set_a(latest_point.a());
  }
  DP_ptr->debug_planning_msg.motionplan_debug.set_ego_speed(EgoInfo::instance()->vehicle_state().linear_velocity);
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_path_data()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_speed_data()->Clear();
  auto frenet_frame_path = best_ref_info->path_data().frenet_frame_path();
  if (frenet_frame_path.size() > 1) {
    double s = frenet_frame_path.front().s();
    const double total_length = std::fmin(100.0,frenet_frame_path.back().s());
    const double delta_s = 2.0;
    while (s <= total_length) {
      auto sl = frenet_frame_path.EvaluateByS(s);
      planning_debug_msgs::SLPoint sl_point;
      sl_point.set_s(sl.s());
      sl_point.set_l(sl.l());
      DP_ptr->debug_planning_msg.motionplan_debug.add_path_data()->CopyFrom(sl_point);
      s += delta_s; 
    }
  }
  auto speed_data = best_ref_info->speed_data(); 
  if (!speed_data.empty()) {
    double t = 0.0;
    const double delta_t = 0.1;
    const double kEpsilon = 1e-6;
    while (t <= speed_data.TotalTime() + kEpsilon) {
      common::SpeedPoint speed_point;
      speed_data.EvaluateByTime(t,&speed_point);
      planning_debug_msgs::SpeedPoint st_point;
      st_point.set_t(speed_point.t());
      st_point.set_s(speed_point.s());
      st_point.set_v(speed_point.v());
      st_point.set_a(speed_point.a());
      st_point.set_da(speed_point.da());
      DP_ptr->debug_planning_msg.motionplan_debug.add_speed_data()->CopyFrom(st_point);
      t += delta_t; 
    }
  }
  bool has_pathplan_time_speend = false;
  bool has_speedplan_time_speend = false;
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_speedplan_task()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_speedplan_time()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_speed_upper_constraint()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_speed_lower_constraint()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_s_upper_constraint()->Clear();
  DP_ptr->debug_planning_msg.motionplan_debug.mutable_s_lower_constraint()->Clear();
  for (const auto& task_stat : best_ref_info->latency_stats().task_stats()) {
    if (task_stat.name() == "pathplan") {
      DP_ptr->debug_planning_msg.motionplan_debug.set_pathplan_total_time( 
                  task_stat.time_ms() );
      has_pathplan_time_speend = true;
    } else if (task_stat.name() == "speedplan") {
      DP_ptr->debug_planning_msg.motionplan_debug.set_speedplan_total_time( 
                  task_stat.time_ms());
      has_speedplan_time_speend = true;            
    } else {//添加速度规划内部耗时debug消息
      DP_ptr->debug_planning_msg.motionplan_debug.add_speedplan_task(task_stat.name());
      DP_ptr->debug_planning_msg.motionplan_debug.add_speedplan_time(task_stat.time_ms());
    } 
  }
  
  if (!has_pathplan_time_speend) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_pathplan_total_time(0.0);
  } 
  if (!has_speedplan_time_speend) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_speedplan_total_time(0.0);
  }
  
  if ((int)best_ref_info->trajectory_type() > 0
     && best_ref_info->debug().planning_data().st_graph_size() >= 2) {//速度规划失败时添加这些debug消息
    for (const auto& speed_upper : 
        best_ref_info->debug().planning_data().st_graph(1).speed_constraint().upper_bound()) {
      DP_ptr->debug_planning_msg.motionplan_debug.add_speed_upper_constraint(speed_upper);
    }
    for (const auto& speed_lower : 
        best_ref_info->debug().planning_data().st_graph(1).speed_constraint().lower_bound()) {
      DP_ptr->debug_planning_msg.motionplan_debug.add_speed_lower_constraint(speed_lower);
    }
    for (const auto& s_upper : best_ref_info->debug().planning_data().st_graph(1).s_constraint().upper_bound()) {
      DP_ptr->debug_planning_msg.motionplan_debug.add_s_upper_constraint(s_upper);
    }
    for (const auto& s_lower : best_ref_info->debug().planning_data().st_graph(1).s_constraint().lower_bound()) {
      DP_ptr->debug_planning_msg.motionplan_debug.add_s_lower_constraint(s_lower);
    }
  } 

  DP_ptr->debug_planning_msg.target_reference_line.mutable_points()->Clear();
  double min_kappa = 0;
  double max_kappa = 0;
  int max_index = 0;
  int min_index = 0;
  int index = 0;
  for (const auto& point : best_ref_info->reference_line().reference_points()) {
    planning_msgs::TrajectoryPoint point_tmp;
    point_tmp.set_xg(point.x());
    point_tmp.set_yg(point.y());
    point_tmp.set_curvature(point.kappa());
    if (point.kappa() > 0) {
      if (point.kappa() > max_kappa) {
        max_kappa = point.kappa();
        max_index = index;
      }
    } else if (point.kappa() < 0 ) {
      if (point.kappa() < min_kappa) {
        min_kappa = point.kappa();
        min_index = index;
      }
    }
    index++;
    DP_ptr->debug_planning_msg.target_reference_line.add_points()->CopyFrom(point_tmp);
  }
  DP_ptr->debug_planning_msg.motionplan_debug.set_min_kappa_index(min_index);
  DP_ptr->debug_planning_msg.motionplan_debug.set_max_kappa_index(max_index);
  DP_ptr->debug_planning_msg.motionplan_debug.set_kappa_min(min_kappa);
  DP_ptr->debug_planning_msg.motionplan_debug.set_kappa_max(max_kappa);
  DP_ptr->debug_planning_msg.motionplan_debug.set_borrow_lane_type(DP_ptr->decision_info.borrow_lane_type);
  if (reference_line_provider_) {
    DP_ptr->debug_planning_msg.motionplan_debug.set_reference_line_smoothed_status( 
            reference_line_provider_->smoothed_status());
  }
}

void MotionPlan::DebugTrajectory(const PathData& output_trajectory) {
  const auto DP = acu::planning::DataPool::Instance()->GetMainDataPtr();

  bool log_trigger = false;
  if (log_trigger) {
    double timestamp_s = acu::common::NodeTime::Now().ToSecond();
    std::ofstream ofs_trajectory;
    std::string filename = "/home/calmcar/work/trajectory_"+std::to_string(timestamp_s)+".csv";
    fstream file;
    file.open(filename.c_str(), ios::in);
    if (!file) {
      ofs_trajectory.open(filename.c_str(), std::ios::app);
      if (!ofs_trajectory.is_open()) {
        AWARN <<" failed to open file: " << filename;
      }
      ofs_trajectory << "globalangle"
                     << ", " 
                     << "angle"
                     << ", "
                     << "curvature"<< std::endl;
    } else {
      ofs_trajectory.open(filename.c_str(), std::ios::app);
      if (!ofs_trajectory.is_open()) {
         AWARN <<" failed to open file: " << filename;
      }
    }
    for (auto& point : output_trajectory.path) {
      ofs_trajectory << fixed << setprecision(4)
        << point.globalangle
        << ","
        << point.angle
        << "," 
        << point.curvature
        << std::endl;   
    }
  } 
}

void MotionPlan::ManualDriveReset() {
  AWARN_IF(FLAGS_enable_debug_motion)<<"manual drive , reset same param!!!";
  NewPathManager::instance()->Clear();
  BehaviorParser::instance()->ResetDecisionHistory();
  last_publishable_trajectory_.reset(nullptr);
  last_publishable_trajectory_type_ = 0;
  ScenarioContext::instance()->Clear();
  FrameHistory::instance()->Clear();
}

void MotionPlan::Reset() {
  NewPathManager::instance()->Clear();
  BehaviorParser::instance()->Reset();
  last_publishable_trajectory_.reset(nullptr);
  frame_.reset(nullptr);
  FrameHistory::instance()->Clear();
  last_publishable_trajectory_type_ = 0;
  ScenarioContext::instance()->Clear();
  if (reference_line_provider_) {
    reference_line_provider_->Stop();
  }
}

double MotionPlan::CalculateSteeringAngleRateMax(const std::vector<std::pair<double ,double>>& kappa_t_pairs) {
  double steering_angle_max = 500;  //degree/s
  if (kappa_t_pairs.size() < 2) return steering_angle_max;
  const double wheelbase = EgoInfo::instance()->vehicle_wheelbase();
  const double eps_transmission_ratio = EgoInfo::instance()->vehicle_param().eps_transmission_ratio;

  const double front_wheel_angle_1 = atan(kappa_t_pairs.at(0).first * wheelbase);
  const double front_wheel_angle_2 = atan(kappa_t_pairs.at(1).first * wheelbase);
  const double delta_t_1 = fabs(kappa_t_pairs.at(1).second - kappa_t_pairs.at(0).second);

  const double steering_angle_rate_1 = fabs(front_wheel_angle_1 - front_wheel_angle_2) / delta_t_1 * kRAD_TO_DEG * eps_transmission_ratio;
  AERROR_IF(FLAGS_enable_debug_motion)<<"CalculateSteeringAngleRateMax debug 1 = "<<steering_angle_rate_1;
  AERROR_IF(FLAGS_enable_debug_motion)<<" kappa = ("<<kappa_t_pairs.at(0).first<< ", "<<kappa_t_pairs.at(1).first<<") .";
  AERROR_IF(FLAGS_enable_debug_motion)<<" t = "<<delta_t_1;
  if (kappa_t_pairs.size() < 3) return steering_angle_rate_1;

  double delta_t_2 = 0;
  double front_wheel_angle_3 = 0;
  double front_wheel_angle_4 = 0;
  if (kappa_t_pairs.size() > 3) { // size >= 4
    front_wheel_angle_3 = atan(kappa_t_pairs.at(2).first * wheelbase);
    front_wheel_angle_4 = atan(kappa_t_pairs.at(3).first * wheelbase);
    delta_t_2 = fabs(kappa_t_pairs.at(3).second - kappa_t_pairs.at(2).second);
  } else { //size = 3
    front_wheel_angle_3 = front_wheel_angle_2;
    front_wheel_angle_4 = atan(kappa_t_pairs.at(2).first * wheelbase);
    delta_t_2 = fabs(kappa_t_pairs.at(2).second - kappa_t_pairs.at(1).second);
  }

  const double steering_angle_rate_2 = fabs(front_wheel_angle_4 - front_wheel_angle_3) / delta_t_2 * kRAD_TO_DEG * eps_transmission_ratio;

  steering_angle_max = std::fmax(steering_angle_rate_1, steering_angle_rate_2); 
  AERROR_IF(FLAGS_enable_debug_motion)<<"CalculateSteeringAngleRateMax debug 2 = "<< steering_angle_rate_2;
  AERROR_IF(kappa_t_pairs.size() > 3&&FLAGS_enable_debug_motion)<<" kappa = ("<<kappa_t_pairs.at(2).first<<", "<<kappa_t_pairs.at(3).first<<") .";
  AERROR_IF(FLAGS_enable_debug_motion)<<" t = "<<delta_t_2;

  return steering_angle_max;
}

} // namespace planning
} // namespace acu
