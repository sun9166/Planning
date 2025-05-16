/**
 * @file path_decider.cpp
 **/

//#include "ros/ros.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "decision.pb.h"

#include "fallback_path_decider.h"
#include "common/util/util.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

FallBackPathDecider::FallBackPathDecider() {
  
}

void FallBackPathDecider::MakeStaticObstacleDecision(const PathInfo &path_data, 
    PathDecision *const path_decision, ReferenceLineInfo *reference_line_info) {
  DCHECK_NOTNULL(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Path is empty.";
    return ;
  }
 
  CarModel vehicle_param = EgoInfo::instance()->vehicle_param(); 
  const double half_width = vehicle_param.half_wheel;
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<", IsStatic = "<<obstacle->IsStatic();
    if (!obstacle->IsStatic()) {
      continue;
    }

    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_ignore() &&
        obstacle->HasLateralDecision() &&
        obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();
    ObjectDecisionType lateral_decision_last;
    FindLateralDecisionHistory(lateral_decision_last, obstacle->Id() );
    if (HasOverlap(path_data, *obstacle) && !lateral_decision_last.has_nudge()) {
      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle, path_data);
      
      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info->reference_line(),
              reference_line_info->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()<<"] obstacle nearest-stop. ";

      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/ignore",
                                               obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()<<"] obstacle ignore. ";
      }
    } else {
      ObjectDecisionType object_decision;
      object_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision("PathDecider/ignore",
                                             obstacle->Id(), object_decision);
      AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()<<"] obstacle has no overlap , ignore. ";
    }
  }
}

bool FallBackPathDecider::HasOverlap(const PathInfo &path_data, const Obstacle &path_obstacle) {
  auto obstacle_box = path_obstacle.PerceptionBoundingBox();
  const double obstacle_center_s = 
       path_obstacle.PerceptionSLBoundary().start_s() + 0.5 * obstacle_box.length();
  const double search_front_s = obstacle_center_s - 10.0;
  const double search_back_s = obstacle_center_s + 10.0;
  AWARN_IF(FLAGS_enable_debug_motion)<<"search range = ("<<search_front_s<<", "<<search_back_s;
  const double back_edge_to_center = EgoInfo::instance()->vehicle_back_edge_to_center();
  const double front_edge_to_center = EgoInfo::instance()->vehicle_front_edge_to_center();
  const double left_edge_to_center = EgoInfo::instance()->vehicle_param().half_wheel;
  const double right_edge_to_center = left_edge_to_center;
  const double length = EgoInfo::instance()->vehicle_length();
  const double car_width = EgoInfo::instance()->vehicle_width();
  const double buffer = path_obstacle.nudge_l_buffer_min();
  for (const auto& path_point : path_data.discretized_path()) {
    if (path_point.s() >= search_front_s && path_point.s() <= search_back_s) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"s = "<<path_point.s();
      Vec2d vec_to_center =
          Vec2d(0.5 * (front_edge_to_center - back_edge_to_center),
                0.5 * (left_edge_to_center - right_edge_to_center))
              .rotate(path_point.theta());
      Vec2d center = Vec2d(path_point.x(), path_point.y()) + vec_to_center;
    
      const Box2d adc_box =
          Box2d(center, path_point.theta(), length + 2 * buffer, car_width + 2 * buffer);
      if (obstacle_box.HasOverlap(adc_box)) {
        return true;
      }
    } 

  }
  return false;
}

ObjectStop FallBackPathDecider::GenerateObjectStopDecision(const Obstacle &path_obstacle,const PathInfo &path_data) {
  ObjectStop object_stop;

  const CarModel vehicle_param = EgoInfo::instance()->vehicle_param();
  double stop_distance = path_obstacle.GetStopDistance(vehicle_param);
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  common::PathPoint stop_ref_point;
      path_data.GetPathPointWithPathS(stop_ref_s,&stop_ref_point);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.theta());
  return object_stop;
}

bool FallBackPathDecider::FindLateralDecisionHistory(ObjectDecisionType& lat_desicion_last, 
                     const std::string& id) {
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last frame is empty";
  } else {
    const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
    if (!last_reference_line_info) {
      AWARN_IF(FLAGS_enable_debug_motion) << "last reference line info is empty";
    } else {
      auto last_obstacle = last_reference_line_info->path_decision().Find(id);
      if (!last_obstacle) {
      //cannot find this obstalce in history;
      } else {
        if (last_obstacle->HasLateralDecision()) {
          lat_desicion_last = last_obstacle->LateralDecision();
          return true;
        }
      }
    }
  }
  return false;
}


}  // namespace planning
}  // namespace acu
