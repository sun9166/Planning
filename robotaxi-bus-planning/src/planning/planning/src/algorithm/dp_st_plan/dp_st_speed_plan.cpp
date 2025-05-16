/**
 * @file dp_st_speed_optimizer.cpp
 **/

////#include "ros/ros.h"
#include <vector>
#include "planning_internal_info.pb.h"

#include "dp_st_graph.h"
#include "dp_st_speed_plan.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/st_graph/st_graph_data.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;
using acu::common::TrajectoryPoint;
using acu::planning_internal::STGraphDebug;

DpStSpeedPlan::DpStSpeedPlan()
    : SpeedOptimizer("DpStSpeedPlan") {}

bool DpStSpeedPlan::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.planner_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}

Status DpStSpeedPlan::Process(const SLBoundary& adc_sl_boundary,
                                   const PathInfo& path_data,
                                   const TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   const SpeedInfo& reference_speed_data,
                                   PathDecision* const path_decision,
                                   SpeedInfo* const speed_data) {

  
  init_point_ = init_point;
  adc_sl_boundary_ = adc_sl_boundary;
  reference_line_ = &reference_line;

  // 1.检查路径规划是否为零
  if (path_data.discretized_path().empty()) {
    std::string msg("Empty path data");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  
  StBoundaryMapper boundary_mapper(
      adc_sl_boundary, st_boundary_config_, *reference_line_, path_data,
      dp_st_speed_config_.total_path_length(), dp_st_speed_config_.total_time(),
      reference_line_info_->IsChangeLanePath());

  auto* debug = reference_line_info_->mutable_debug();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();

  path_decision->EraseStBoundaries();
  speed_data->SetReverse(path_data.IsReverse());
  if (boundary_mapper.CreateStBoundary(path_decision).code() ==   
      ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR) {
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AWARN_IF(FLAGS_enable_debug_motion)<< msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (FLAGS_enable_decision_replace_dp_speed_optimizer) {//@pqg 
    for (auto* obstacle : path_decision->obstacles().Items()) {
      auto id = obstacle->Id();
      if (!obstacle->st_boundary().IsEmpty()) {
        if (obstacle->st_boundary().boundary_type() ==
            StBoundary::BoundaryType::KEEP_CLEAR) {    
          path_decision->Find(id)->SetBlockingObstacle(false);
        } else {
          path_decision->Find(id)->SetBlockingObstacle(true);
        }
      }
    }
    AWARN_IF(FLAGS_enable_debug_motion && BehaviorParser::instance()->IsObstacleDecisionFailed())
                <<"obstacle decision failed";
    if (BehaviorParser::instance()->IsObstacleDecisionFailed()) {
      const std::string msg(Name() +
                          ":obstacle decision failed.");
      AWARN_IF(FLAGS_enable_debug_motion)<< msg; 
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
    }
    return Status::OK();
  }
  
  SpeedLimitDecider speed_limit_decider(adc_sl_boundary, st_boundary_config_,
                                        *reference_line_, path_data, false);

  if (!SearchStGraph(boundary_mapper, speed_limit_decider, path_data,
                     speed_data, path_decision, st_graph_debug)) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AWARN_IF(FLAGS_enable_debug_motion)<< msg; 
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  return Status::OK();
}

bool DpStSpeedPlan::SearchStGraph(
    const StBoundaryMapper& boundary_mapper,
    const SpeedLimitDecider& speed_limit_decider, const PathInfo& path_data,
    SpeedInfo* speed_data, PathDecision* path_decision,
    STGraphDebug* st_graph_debug) const {
  std::vector<const StBoundary*> boundaries;
  for (auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    if (!obstacle->st_boundary().IsEmpty()) {
      if (obstacle->st_boundary().boundary_type() ==
          StBoundary::BoundaryType::KEEP_CLEAR) {
        
        path_decision->Find(id)->SetBlockingObstacle(false);
      } else {
        path_decision->Find(id)->SetBlockingObstacle(true);
      }
      boundaries.push_back(&obstacle->st_boundary());
    }
  }

  // step 2 perform graph search
  SpeedLimit speed_limit;
  if (!speed_limit_decider
           .GetSpeedLimits(path_decision->obstacles(), &speed_limit)
           .ok()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Getting speed limits for dp st speed optimizer failed!";
    return false;
  }

  const double path_length = path_data.discretized_path().Length();

  StGraphData st_graph_data(boundaries, init_point_, speed_limit, path_length,
                          reference_line_->cruise_speed());

  DpStGraph st_graph(st_graph_data, dp_st_speed_config_,
                     reference_line_info_->path_decision()->obstacles().Items(),
                     init_point_, adc_sl_boundary_);
  if (!st_graph.Search(speed_data).ok()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "failed to search graph with dynamic programming.";
    return false;
  }
  return true;
}


}  // namespace planning
}  // namespace acu
