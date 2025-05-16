/**
 * @file speed_planner.cpp
 **/

#include <fstream>
#include <limits>
#include <utility>

//#include "ros/ros.h"
#include "speed_planner.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "speed_profile_generator.h"
#include "fallback_path_generator.h"
#include "src/execution/motionplan/planner/speed_decider.h"
#include "src/algorithm/dp_st_plan/dp_st_speed_plan.h"
#include "src/algorithm/curve/quartic_polynomial_curve1d.h"
#include "src/algorithm/optimizer/qp_spline_st_speed_optimizer.h"

namespace acu {
namespace planning {

using common::ErrorCode;
using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::math::Vec2d;

namespace {
constexpr double kPathOptimizationFallbackCost = 2e4;
constexpr double kSpeedOptimizationFallbackCost = 2e4;
constexpr double kStraightForwardLineCost = 10.0;
}  // namespace

void SpeedPlanner::RegisterTasks() { 
  task_factory_.Register(DP_ST_SPEED_OPTIMIZER,
                         []() -> Procedure* { return new DpStSpeedPlan(); });
  task_factory_.Register(SPEED_DECIDER,
                         []() -> Procedure* { return new SpeedDecider(); });
  task_factory_.Register(QP_SPLINE_ST_SPEED_OPTIMIZER, 
                         []() -> Procedure* { return new QpSplineStSpeedOptimizer();
  });
}

Status SpeedPlanner::Init(const PlanningConfig& config) {
  RegisterTasks();
  for (const auto task : config.planner_config().task()) {
    tasks_.emplace_back(
      task_factory_.CreateObject(static_cast<TaskType>(task)));
  }
  for (auto& task : tasks_) {
    if (!task->Init(config)) {
      std::string msg(
        common::util::StrCat("Init task[", task->Name(), "] failed."));
      AWARN_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
    }
  }
  return Status::OK();
}

Status SpeedPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame) {//frame是可改变的
  auto status = Status::OK();
  status = PlanOnReferenceLine(planning_start_point, frame,
                                 frame->mutable_reference_line_info().get());
  if (status != Status::OK()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "planner failed to make a driving plan . ";
  }

  return status;
}

Status SpeedPlanner::PlanOnReferenceLine(
  const TrajectoryPoint& planning_start_point, Frame* frame,
  ReferenceLineInfo* reference_line_info) {
  const double speedplan_start_timestamp = acu::common::NodeTime::Now().ToSecond();
  if (!reference_line_info->IsInited()) {
    if (!reference_line_info->Init(frame->obstacles())) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to init reference line";
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "Init reference line failed");
    }
  }

  if (reference_line_info->speed_data().empty()) { 
    auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
    auto speed_profile =
      speed_profile_generator::GenerateInitSpeedProfile(planning_start_point, reference_line_info);
    if (speed_profile.empty()) {
      speed_profile = speed_profile_generator::GenerateSpeedHotStart(planning_start_point,
                     reference_line_info->planning_target().cruise_speed());
    }
    *heuristic_speed_data = SpeedInfo(speed_profile);
  }
  
  auto ret = Status::OK();

  // path fallback didn't lead to speed fallback any more
  for (auto& optimizer : tasks_) {
    const double start_timestamp = acu::common::NodeTime::Now().ToSecond();
    ret = optimizer->Execute(frame, reference_line_info);
    const double end_timestamp = acu::common::NodeTime::Now().ToSecond();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    AINFO_IF(FLAGS_enable_debug_motion) << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";
    auto latency_stats = reference_line_info->mutable_latency_stats();
    auto time_latency = latency_stats->add_task_stats();
    time_latency->set_name(optimizer->Name()); 
    AINFO_IF(FLAGS_log_enable)<<"Add "<<optimizer->Name()<<" time_latency===================================";
    time_latency->set_time_ms(time_diff_ms);

    if (!ret.ok()) {
      reference_line_info->AddCost(std::numeric_limits<double>::infinity());
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to run procedure[" << optimizer->Name()
       << " Error message: " << ret.error_message();
      break;
    }  
  }
  
  if (!ret.ok() || reference_line_info->speed_data().empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Speed fallback.";
    reference_line_info->SetSpeedFallbackReason(ret.error_message());
    *reference_line_info->mutable_speed_data() =
      speed_profile_generator::GenerateFallbackSpeedProfile(reference_line_info);
    reference_line_info->AddCost(kSpeedOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
  }

  if (!(reference_line_info->trajectory_type() ==
        ADCTrajectory::PATH_FALLBACK ||
        reference_line_info->trajectory_type() ==
        ADCTrajectory::SPEED_FALLBACK)) {
    reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  }

  double speed_plan_total_time = 1000*(acu::common::NodeTime::Now().ToSecond() - speedplan_start_timestamp);
  auto latency_stats = reference_line_info->mutable_latency_stats();
  auto time_latency = latency_stats->add_task_stats();
  time_latency->set_name("speedplan"); 
  time_latency->set_time_ms(speed_plan_total_time);
 
  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
        planning_start_point.relative_time(),
        planning_start_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AWARN_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  if ( FLAGS_generate_DRpath_type == 1 ) {
    const VehicleState* vehicle_state_ptr = &frame->vehicle_state();
    GetDRTrajectory(&trajectory, vehicle_state_ptr);
  }

  reference_line_info->SetTrajectory(trajectory);
  if (ret == Status::OK()) {  // vehicle can drive on this reference line.
    reference_line_info->SetDrivable(true);
  }
  return ret;
}

void SpeedPlanner::GetDRTrajectory(DiscretizedTrajectory* ptr_discretized_trajectory,
    const VehicleState* vehicle_state_ptr) {
  double dr_yaw_rad = vehicle_state_ptr->dr_yaw;
  double yaw_rad = vehicle_state_ptr->yaw;
  AINFO_IF(FLAGS_enable_debug_motion)<<__FUNCTION__<<fixed<<std::setprecision(3)
    <<" dr_yaw "<<dr_yaw_rad<<" yaw"<<yaw_rad
    <<" xg "<<vehicle_state_ptr->x<<" yg "<<vehicle_state_ptr->y
    <<" drxg "<<vehicle_state_ptr->dr_x<<" dryg "<<vehicle_state_ptr->dr_y;
  for(auto& point:*ptr_discretized_trajectory) {
    double dx =  (point.path_point().x() - vehicle_state_ptr->x) * cos(yaw_rad)
                +(point.path_point().y() - vehicle_state_ptr->y) * sin(yaw_rad);
    double dy = -(point.path_point().x() - vehicle_state_ptr->x) * sin(yaw_rad)
                +(point.path_point().y() - vehicle_state_ptr->y) * cos(yaw_rad);
    double dtheta = point.path_point().theta() - yaw_rad;
    point.mutable_path_point()->set_dr_x(dx * cos(dr_yaw_rad) - dy * sin(dr_yaw_rad) + vehicle_state_ptr->dr_x);
    point.mutable_path_point()->set_dr_y(dx * sin(dr_yaw_rad) + dy * cos(dr_yaw_rad) + vehicle_state_ptr->dr_y);
    double dr_theta = acu::common::math::NormalizeAngle(dtheta + dr_yaw_rad);
    point.mutable_path_point()->set_dr_theta(dr_theta);
  }
}


}  // namespace planning
}  // namespace acu
