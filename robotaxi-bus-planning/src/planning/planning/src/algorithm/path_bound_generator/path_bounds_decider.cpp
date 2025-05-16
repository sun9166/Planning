/**
 * @file path_bounds_decider.cpp
 **/
#include "path_bounds_decider.h"
#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <set>
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "common/util/string_util.h"
#include "lane_change_generator/lane_change_bound_generator.h"
#include "fallback_generator/fallback_bound_generator.h"
#include "pullover_generator/pullover_bound_generator.h"
#include "regular_generator/regular_bound_generator.h"
#include "lite_coupling_generator/lite_coupling_generator.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

PathBoundsDecider::PathBoundsDecider()
    :Procedure("PathBoundsDecider") {}
bool PathBoundsDecider::Init(const PlanningConfig &config) {
  config_ = config.planner_config().path_bounds_decider_config();
  is_init_ = true;
  return true;
}   

acu::common::Status PathBoundsDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  Procedure::Execute(frame, reference_line_info);
  return Process(frame,reference_line_info);
} 

Status PathBoundsDecider::Process(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  //1.校验是否可以跳过PathBoundsDecider处理
  // skip piecewise_jerk_path_optimizer if reused path
  bool path_reusable =
       reference_line_info_->LaterAction().first == eActionEnum::DEFAULT_VALUE? true:false;
  if(EgoInfo::instance()->dis_to_virtual_wall()<10&&EgoInfo::instance()->vehicle_state().linear_velocity<0.25){
    path_reusable=true;
  }
  bool is_start_from_road_side = false;
  if (FLAGS_enable_skip_path_tasks 
    && path_reusable
    && EgoInfo::instance()->vehicle_state().driving_mode == 0) {
    common::SLPoint init_sl_point = reference_line_info_->init_sl_point();
    if ( (fabs(init_sl_point.l()) < FLAGS_replan_lateral_distance_threshold && NewPathManager::instance()->stitch_state())
      || reference_line_info->IsReverse()
      || NewPathManager::instance()->is_pull_over_path()) {
      return Status::OK();
    } else {
      if (EgoInfo::instance()->vehicle_state().linear_velocity < 0.2) {
        is_start_from_road_side = true;
      } 
      AINFO_IF(FLAGS_enable_debug_motion)<< "car is far away from reference line, need a new path. ";
    }
  }

  //2.初始化边界
  InitPathBoundsDecider(*frame, *reference_line_info);
  std::vector<PathBoundary> candidate_path_boundaries;

  //3.生成FallBack边界
  if (!FLAGS_use_last_publishable_trajectory_for_fallback_path) {
    FallbackBoundGenerator generator(frame_, reference_line_info_, start_frenet_state_);
    Status ret = generator.Generate(candidate_path_boundaries);
    if (!ret.ok()) {
      AINFO << "Cannot generate a fallback path bound.";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, ret.error_message());
    }
  }
  
  //3.生成PullOver边界
  AINFO_IF(FLAGS_enable_debug_motion)<<"LaterAction: "<<(int)reference_line_info_->LaterAction().first
      <<", has_pull_over_request: "<<BehaviorParser::instance()->has_pull_over_request();
  const bool plan_pull_over_path =
     reference_line_info_->LaterAction().first == eActionEnum::PULL_OVER;
      //  && BehaviorParser::instance()->has_pull_over_request()? true:false;
  if (plan_pull_over_path) {
    PullOverBoundGenerator generator(frame_, reference_line_info_, start_frenet_state_);
    Status ret = generator.Generate(candidate_path_boundaries);
    if (!ret.ok()) {
      AWARN_IF(FLAGS_enable_debug_motion) 
          << "Cannot generate a pullover path bound, do regular planning.";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, ret.error_message());
    }
    reference_line_info_->SetCandidatePathBoundaries(std::move(candidate_path_boundaries));
    return Status::OK();
  }

  //4.1校验换道决策边界的合理性
  if (reference_line_info->DecisionPathBound().size() > 1 && reference_line_info->IsChangeLanePath()) {
    const auto car_sl = reference_line_info->car_sl();
    const double car_l = car_sl.l();
    const double theta = reference_line_info->speed_init_point().path_point().theta();
    AERROR_IF(FLAGS_enable_debug_motion)<< "!!!!!!VehicleSlBoundary , car_l:" << car_l << ", end_l: "
        << ", theta = " << theta;
    //目标参考线在右边
    const int target_reference_line_id = BehaviorParser::instance()->target_reference_line_id();
    double lanechange_length_min = 1.5 * EgoInfo::instance()->vehicle_length();
    const double ratio = 1.5 * EgoInfo::instance()->vehicle_length() / 3.5;
    if (target_reference_line_id == 30) {
      // 横向和纵向比，根据偏角适当调整长度（偏角45°时缩减最多）
      lanechange_length_min = std::fabs(car_l)*ratio + std::max(theta*1.0, -0.25*3.14159);
    } else if (target_reference_line_id == 20) {
      lanechange_length_min = std::fabs(car_l)*ratio - std::max(theta*1.0, 0.25*3.14159);
    }
    lanechange_length_min = 
       std::fmax(FLAGS_lane_change_time_buffer_min * EgoInfo::instance()->vehicle_state_new().linear_velocity, lanechange_length_min);
    AERROR_IF(FLAGS_enable_debug_motion)<< "!!!!!!lanechange_length_min:" << lanechange_length_min;
    auto first_bound = reference_line_info->DecisionPathBound().at(0);
    auto second_bound = reference_line_info->DecisionPathBound().at(1);
    if (get<0>(first_bound) < lanechange_length_min 
            && get<1>(first_bound) != get<2>(first_bound) 
            && get<1>(second_bound) == get<2>(second_bound)){
      AERROR_IF(FLAGS_enable_debug_motion)<<"decision path bound error !! lanechange is not safe "
        <<", first bound s = "<<get<0>(first_bound)<<", lc_l_min = "<<lanechange_length_min;
      return  Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "decision path bound error"); 
    }
  }

  //4.2生成LanceChange边界
  if (FLAGS_enable_smarter_lane_change &&
      reference_line_info->IsChangeLanePath() || is_start_from_road_side) {
    LaneChangeBoundGenerator generator(frame_, reference_line_info_, start_frenet_state_);
    Status ret = generator.Generate(candidate_path_boundaries);
    if (!ret.ok()) {
      AINFO_IF(FLAGS_enable_debug_motion) << "Cannot generate a lane-change path bound.";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, ret.error_message());
    }
    reference_line_info_->SetCandidatePathBoundaries(std::move(candidate_path_boundaries));
    return Status::OK();
  }

  //5.1生成弱耦合边界，会内部判断是否有相关输入
  LiteCouplingBoundGenerator lite_coupling_generator(frame_, reference_line_info_, start_frenet_state_);
  Status ret = lite_coupling_generator.Generate(candidate_path_boundaries);
  if (!ret.ok()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Cannot generate a lite_coupling path bound, continue regular.";
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"candidate_path_boundaries: "<<candidate_path_boundaries.size();
  //5.2在弱耦合边界基础上，补充Regular边界
  RegularBoundGenerator generator(frame_, reference_line_info_, start_frenet_state_);
  ret = generator.Generate(candidate_path_boundaries);
  if (!ret.ok()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Cannot generate a regular path bound.";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, ret.error_message());
  }

  reference_line_info_->SetCandidatePathBoundaries(std::move(candidate_path_boundaries));
  return Status::OK();
}

void PathBoundsDecider::InitPathBoundsDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  common::TrajectoryPoint planning_start_point = frame.PlanningStartPoint();
  if (FLAGS_enable_two_stitching_trajectory) {
    planning_start_point = reference_line_info.pathplan_init_point();
  }
  AINFO_IF(FLAGS_enable_debug_motion) << "Plan at the starting point: x = "
         << fixed << setprecision(4) << planning_start_point.path_point().x()
         << ", y = " << planning_start_point.path_point().y()
         << ", and angle = " << planning_start_point.path_point().theta();
  start_frenet_state_ = reference_line.ToFrenetFrame(planning_start_point);
}

}  // namespace planning
}  // namespace acu
