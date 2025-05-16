/**
 * @file path_planner.cpp
 **/

#include <fstream>
#include <limits>
#include <utility>

//#include "ros/ros.h"
#include "path_planner.h"
#include "common/math/math_utils.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "speed_profile_generator.h"
#include "fallback_path_generator.h"
#include "src/execution/motionplan/planner/path_decider.h"
#include "src/algorithm/curve/quartic_polynomial_curve1d.h"
#include "src/algorithm/path_bound_generator/path_bounds_decider.h"
#include "src/algorithm/optimizer/piecewise_jerk_path_optimizer.h"
#include "src/algorithm/optimizer/quasi_potential_field_path_optimizer.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "src/execution/motionplan/planner/fallback_path_decider.h"


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

void PathPlanner::RegisterTasks() { 
  task_factory_.Register(PATH_DECIDER,
                         []() -> Procedure* { return new PathDecider(); });
  task_factory_.Register(PATH_BOUNDS_DECIDER,
                         []() -> Procedure* { return new PathBoundsDecider(); });
  task_factory_.Register(PIECEWISE_JERK_PATH_OPTIMIZER,
                         []() -> Procedure* { return new PiecewiseJerkPathOptimizer(); });
  task_factory_.Register(QUASI_POTENTIAL_FIELD_PATH_OPTIMIZER,
                         []() -> Procedure* { return new QuasiPotentialFieldPathOptimizer(); });
}

Status PathPlanner::Init(const PlanningConfig& config) {
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
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
  }
  return Status::OK();
}

Status PathPlanner::Plan(const common::TrajectoryPoint& planning_start_point,
                       Frame* frame) {
  auto status = Status::OK();
  status = PlanOnReferenceLine(planning_start_point, frame,
                               frame->mutable_reference_line_info().get());
  if (status != Status::OK()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "planner failed to make a driving plan . ";
  }
  return status;
}

Status PathPlanner::PlanOnReferenceLine(
  const common::TrajectoryPoint& planning_start_point, Frame* frame,
  ReferenceLineInfo* reference_line_info) {
  const double pathplan_start_timestamp = acu::common::NodeTime::Now().ToSecond();
  if (!reference_line_info->IsInited()) {
    if (!reference_line_info->Init(frame->obstacles())) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to init reference line";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "Init reference line failed");
    }
  }

  auto* heuristic_speed_data = reference_line_info->mutable_speed_data();
  auto speed_profile =
    speed_profile_generator::GenerateInitSpeedProfile(planning_start_point, reference_line_info);
  if (speed_profile.empty()) {
    speed_profile = speed_profile_generator::GenerateSpeedHotStart(planning_start_point,
          reference_line_info->planning_target().cruise_speed());
  }
  *heuristic_speed_data = SpeedInfo(speed_profile);
  auto ret = Status::OK();

  bool use_piecewise_jerk_path_optimizer = true;
  if (reference_line_info->LaterAction().first == eActionEnum::CHANGE_OFFSET
    && !(reference_line_info->LaterAction().second == eDirectionEnum::DEFAULT_VALUE)) {
    use_piecewise_jerk_path_optimizer = false;
  }
  if (!FLAGS_use_piecewise_jerk_path_optimizer_mainly) {
    use_piecewise_jerk_path_optimizer = false;
  }

  if (!FLAGS_enable_apply_piecewise_jerk_plan_in_pull_over 
    && reference_line_info->LaterAction().first == eActionEnum::PULL_OVER) {
    use_piecewise_jerk_path_optimizer = false;
  }

  // bool use_quasi_potential_field_path_optimizer = reference_line_info->HasUncertainObstacles() ;
  bool use_quasi_potential_field_path_optimizer = false;
  if(reference_line_info->LaterAction().first == eActionEnum::PULL_OVER){
    use_quasi_potential_field_path_optimizer = false;
  }
  for (auto& optimizer : tasks_) {
    const double start_timestamp = acu::common::NodeTime::Now().ToSecond();
    if (use_piecewise_jerk_path_optimizer) {
      if (optimizer->Name() == "DpPolyPathOptimizer") {
        continue;
      }
      if(use_quasi_potential_field_path_optimizer){
        if (optimizer->Name() == "PiecewiseJerkPathOptimizer") {
          continue;
        }
      } else {
        if (optimizer->Name() == "QuasiPotentialFieldPathOptimizer") {
          continue;
        }
      }
    } else {
      if ( optimizer->Name() == "PathBoundsDecider" ||
         optimizer->Name() == "PiecewiseJerkPathOptimizer") {
        continue;
      }
    }

    ret = optimizer->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      reference_line_info->AddCost(std::numeric_limits<double>::infinity());
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to run procedure[" << optimizer->Name()<<"];" 
            << "Error message: " << ret.error_message();
      break;
    }
    const double end_timestamp = acu::common::NodeTime::Now().ToSecond();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    AINFO_IF(FLAGS_enable_debug_motion) << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";
  }

  if (reference_line_info->path_data().Empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Path fallback.";
    reference_line_info->SetPathFallbackReason(ret.error_message());
    // NewPathManager::instance()->Clear();//@pqg add 1215
    if (FLAGS_use_last_publishable_trajectory_for_fallback_path) {
      auto vehicle_state = EgoInfo::instance()->vehicle_state();
      const auto* ptr_last_frame = FrameHistory::instance()->Latest();
      if (ptr_last_frame == nullptr) {
        AERROR_IF(FLAGS_enable_debug_motion)<< "Last frame doesn't succeed, fail to retrieve last frame path data";
        fallback_path_generator::GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
      } else {
        auto last_frame_discretized_path =
                     ptr_last_frame->ComputedTrajectory();
        if (!last_frame_discretized_path.empty()) {
          last_frame_discretized_path.UpdateTrajectoryPoint({vehicle_state.x, vehicle_state.y}, 1.0e-6);
          fallback_path_generator::GenerateFallbackPathProfileUsingPrevTrajectory(reference_line_info,
                                  reference_line_info->mutable_path_data(),last_frame_discretized_path);
        } else {
          AERROR_IF(FLAGS_enable_debug_motion)<< "Last frame trajectory is empty .";
          fallback_path_generator::GenerateFallbackPathProfile(reference_line_info,
                                  reference_line_info->mutable_path_data());
        }
      }
      
    } else {
      fallback_path_generator::GenerateFallbackPathProfile(reference_line_info,
                                reference_line_info->mutable_path_data());
    }
    FallBackPathDecider::MakeStaticObstacleDecision(
          reference_line_info->path_data(), reference_line_info->path_decision(), reference_line_info);
    reference_line_info->AddCost(kPathOptimizationFallbackCost);
    reference_line_info->set_trajectory_type(ADCTrajectory::PATH_FALLBACK);
  }
  double path_plan_total_time = 1000*(acu::common::NodeTime::Now().ToSecond() - pathplan_start_timestamp);
  auto latency_stats = reference_line_info->mutable_latency_stats();
  auto time_latency = latency_stats->add_task_stats();
  time_latency->set_name("pathplan"); 
  time_latency->set_time_ms(path_plan_total_time);
  if ( FLAGS_generate_DRpath_type == 2 ) {
    const VehicleState* vehicle_state_ptr = &frame->vehicle_state();
    GetDRTrajectory(reference_line_info->mutable_path_data(), vehicle_state_ptr);
  }
  return ret;
}

void PathPlanner::GetDRTrajectory(PathInfo* pathdata,
    const VehicleState* vehicle_state_ptr) {
  double dr_yaw_rad = vehicle_state_ptr->dr_yaw;
  double yaw_rad = vehicle_state_ptr->yaw;
  for(auto& point:*pathdata->mutable_discretized_path()) {
    double dx =  (point.x() - vehicle_state_ptr->x) * cos(yaw_rad)
                +(point.y() - vehicle_state_ptr->y) * sin(yaw_rad);
    double dy = -(point.x() - vehicle_state_ptr->x) * sin(yaw_rad)
                +(point.y() - vehicle_state_ptr->y) * cos(yaw_rad);
    double dtheta = point.theta() - yaw_rad;
    point.set_dr_x(dx * cos(dr_yaw_rad) - dy * sin(dr_yaw_rad) + vehicle_state_ptr->dr_x);
    point.set_dr_y(dx * sin(dr_yaw_rad) + dy * cos(dr_yaw_rad) + vehicle_state_ptr->dr_y);
    double dr_theta = acu::common::math::NormalizeAngle((dtheta + dr_yaw_rad));
    point.set_dr_theta(dr_theta);
  }
  pathdata->SetDiscretizedPath(pathdata->discretized_path(), false);
}

}  // namespace planning
}  // namespace acu
