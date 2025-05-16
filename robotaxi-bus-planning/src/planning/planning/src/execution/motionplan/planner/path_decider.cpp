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

#include "path_decider.h"
#include "common/util/util.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "common/math/vec2d.h"


namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;

PathDecider::PathDecider() : Procedure("PathDecider") {
  has_stop_decision_history_ = false;
}

acu::common::Status PathDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  Procedure::Execute(frame, reference_line_info);
  return Process(reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const PathInfo &path_data,
                            PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  AddTerminalVirtualObstacle(path_data);
  if (!MakeObjectDecision(path_data, path_decision)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "dp_road_graph decision ");
  }
  return Status::OK();
}

void PathDecider::AddTerminalVirtualObstacle(const PathInfo &path_data) {
  if (path_data.frenet_frame_path().empty()) {
    return;
  }
  const double kDeceleration = 0.5;
  const double kPathLength = 90;
  double preview_distance = 0.5 * EgoInfo::instance()->vehicle_state().linear_velocity 
                     * EgoInfo::instance()->vehicle_state().linear_velocity / kDeceleration + 5;
  preview_distance = std::fmax(preview_distance,30.0);
  preview_distance = std::fmin(preview_distance,kPathLength);
  if (ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::PULL_OVER){
    auto destination_point = path_data.discretized_path().back(); 
    Box2d destination_wall_box{common::math::Vec2d(destination_point.x(),destination_point.y()),
                                 destination_point.theta(),
                                 FLAGS_virtual_stop_wall_length,//0.1
                                 3.5};
      auto* obstacle = frame_->CreateStaticVirtualObstacle("terminal_stop", destination_wall_box);
      Obstacle* stop_obstacle = reference_line_info_->AddObstacle(obstacle);
      if (!stop_obstacle) {
        AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create obstacle for terminal_stop !";
      } else {
        const double stop_distance = -1*FLAGS_pull_over_extend_s;//0.01;
        auto stop_point = reference_line_info_->reference_line().GetReferencePoint(
                            stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance);
        ObjectDecisionType stop;
        auto stop_decision = stop.mutable_stop();
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);//所有停车行为都当成红灯停车
        stop_decision->set_distance_s(-stop_distance);
        stop_decision->set_stop_heading(stop_point.heading());
        stop_decision->mutable_stop_point()->set_x(stop_point.x());
        stop_decision->mutable_stop_point()->set_y(stop_point.y());
        stop_decision->mutable_stop_point()->set_z(0.0);
        auto* path_decision = reference_line_info_->path_decision();
        path_decision->AddLongitudinalDecision("destination stop ", "terminal_stop", stop);
      }

    }else if(path_data.frenet_frame_path().back().s() < preview_distance){ 
    auto destination_point_ = path_data.discretized_path().back(); 
    common::SLPoint sl_point;
    sl_point.set_s(path_data.frenet_frame_path().back().s()+EgoInfo::instance()->vehicle_front_edge_to_center());
    sl_point.set_l(path_data.frenet_frame_path().back().l());
    Vec2d destination_point;
    reference_line_info_->reference_line().SLToXY(sl_point,&destination_point);
    Box2d destination_wall_box{common::math::Vec2d(destination_point.x(),destination_point.y()),
                                 destination_point_.theta(),
                                 FLAGS_virtual_stop_wall_length,//0.1
                                 3.5};
      AINFO<<"current_latscenario:  "<<(int)ScenarioContext::instance()->scenario_info().current_latscenario;
      std::string stop_id ="last_point";
      auto* obstacle = frame_->CreateStaticVirtualObstacle(stop_id, destination_wall_box);
      Obstacle* stop_obstacle = reference_line_info_->AddObstacle(obstacle);
      AINFO<<"stop_id: "<<stop_id<<" stop_s :"<< stop_obstacle->PerceptionSLBoundary().start_s();
      if (!stop_obstacle) {
        AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create obstacle for terminal_stop !";
      } else {
        const double stop_distance = 0.01;
        auto stop_point = reference_line_info_->reference_line().GetReferencePoint(
                            stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance);
        ObjectDecisionType stop;
        auto stop_decision = stop.mutable_stop();
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);//所有停车行为都当成红灯停车
        stop_decision->set_distance_s(-stop_distance);
        stop_decision->set_stop_heading(stop_point.heading());
        stop_decision->mutable_stop_point()->set_x(stop_point.x());
        stop_decision->mutable_stop_point()->set_y(stop_point.y());
        stop_decision->mutable_stop_point()->set_z(0.0);
        auto* path_decision = reference_line_info_->path_decision();
        path_decision->AddLongitudinalDecision("destination stop ", stop_id, stop);
      }
  }
}

bool PathDecider::MakeObjectDecision(const PathInfo &path_data,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to make decisions for static obstacles.";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathInfo &path_data, PathDecision *const path_decision) {
  
  DCHECK_NOTNULL(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  if (frenet_path.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Path is empty.";
    return false;
  }
 
  CarModel vehicle_param = EgoInfo::instance()->vehicle_param(); 
  const double half_width = vehicle_param.half_wheel;

  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;
  // FLAGS_lateral_ignore_buffer = 3.0，If an obstacle's lateral distance is further away than this 
  // distance, ignore it 
  bool has_stop_decision = false;
  for (const auto *obstacle : path_decision->obstacles().Items()) {
    bool is_bicycle_or_pedestrian =
        (obstacle->Perception().type == 2  //TypeEnum::PEDESTRIAN
         || obstacle->Perception().type == 3);//TypeEnum::CYCLIST
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
      // 已有的路径停车决策中，已有的虚拟障碍物的距离也更新到sto_s dzx
      path_decision->MergeWithMainStop(
                  obstacle->LongitudinalDecision().stop(), obstacle->Id(),
                  reference_line_info_->reference_line(),
                  reference_line_info_->AdcSlBoundary());
      continue;
    }
    if (obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = obstacle->PerceptionSLBoundary();

    //障碍物在路径方向之外，忽略
    if (sl_boundary.end_s() < frenet_path.front().s() ||
        (sl_boundary.start_s() > frenet_path.back().s() + FLAGS_exceed_path_length_buffer)) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle->Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle->Id(),
                                        object_decision);
      AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()<<"] obstacle not-in-s ";
      continue;
    }

    //1.check heading
    bool has_cut_int_intetion = true;
    if (obstacle->Perception().is_moveble) {
      common::PathPoint math_point;
      if (path_data.GetPathPointWithPathS(obstacle->PerceptionSLBoundary().start_s(),
                                 &math_point)) {
        double ego_heading = math_point.theta();
        double angle_diff = ego_heading - obstacle->Perception().global_angle * kDEG_TO_RAD;
        angle_diff = acu::common::math::NormalizeAngle(angle_diff); 
    
        if (fabs(angle_diff) <= 0.16 * M_PI) {//<=30degre 认为这种障碍物没有危险。
           has_cut_int_intetion = false;
        }
      }  
    }

    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);//从障碍物的boundary计算找到与路径的最小距离的路点
    const double curr_l = frenet_point.l();
    //障碍物是否在U掉头区域内
    std::pair<double, double> u_turn_interval=std::make_pair(0.0,0.0);
    bool is_u_turn = BehaviorParser::instance()->target_reference_line().IsUturn(u_turn_interval);
    bool obstacle_is_in_unturn = is_u_turn 
        && sl_boundary.start_s() < u_turn_interval.second+5.0 
        && sl_boundary.start_s() > u_turn_interval.first-5.0 
        &&  curr_l - half_width < sl_boundary.end_l() + 0.2;//障碍物左边界在右车身0.2m以内
    SLBoundary new_sl_boundary;
    new_sl_boundary.set_start_s(sl_boundary.start_s());
    new_sl_boundary.set_end_s(sl_boundary.end_s());
    new_sl_boundary.set_start_l(sl_boundary.start_l() - curr_l);
    new_sl_boundary.set_end_l(sl_boundary.end_l() - curr_l);
    double boudary_l_min = 
         std::fmin(fabs(new_sl_boundary.start_l()),fabs(new_sl_boundary.end_l()));
    const double adc_front_edge_s = EgoInfo::instance()->start_point().path_point().s() 
                + EgoInfo::instance()->vehicle_front_edge_to_center();
    //距离车身的距离超过3m，忽略
    if ( (-lateral_radius > new_sl_boundary.end_l() ||
        lateral_radius < new_sl_boundary.start_l()) 
        && new_sl_boundary.end_l() * new_sl_boundary.start_l() > 0) { 
      // ignore //忽略
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle->Id(),
                                        object_decision);
      AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                          <<"] obstacle not-in-l, min_l = "<<boudary_l_min;
    } else if (obstacle_is_in_unturn) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->PerceptionId()<<"], " << 
          "U_turn_obstacle_speed_limit";
        *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
        if (path_decision->MergeWithMainStop(
                  object_decision.stop(), obstacle->Id(),
                  reference_line_info_->reference_line(),
                  reference_line_info_->AdcSlBoundary())) {
            path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                                   obstacle->Id(), object_decision);
            AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id() <<"] "
                              <<", U_turn stop ";
            has_stop_decision = true;
          }
    } else if ((boudary_l_min < half_width + obstacle->nudge_l_buffer_min() 
        && new_sl_boundary.end_l() * new_sl_boundary.start_l() > 0)
           || new_sl_boundary.end_l() * new_sl_boundary.start_l() <= 0) { 
      // stop 障碍物阻挡了整条路径，停止
      ObjectDecisionType lateral_decision_last;
      if (obstacle->nudge_l_buffer_min() >= FLAGS_static_decision_nudge_l_buffer//如果比这个小，说明降低了避障横向距离，不需要滞回
        && FindLateralDecisionHistory(lateral_decision_last,obstacle->Id())) {
        // && !obstacle->Perception().is_moveble) {//moveble obstacle not consider this
        if (lateral_decision_last.has_nudge() && boudary_l_min > 
                        half_width + FLAGS_static_decision_nudge_l_buffer_min) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"change stop -> nudge. ";
          if (curr_l > sl_boundary.end_l()) {
            // LEFT_NUDGE
            ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
            object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
            object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
            path_decision->AddLateralDecision("PathDecider/left-nudge",
                                              obstacle->Id(), object_decision);
            AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                              <<"] obstacle left-nudge , min_l = "<<boudary_l_min
                              <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
          } else if (curr_l < sl_boundary.start_l()) {
            // RIGHT_NUDGE
            ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
            object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
            object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
            path_decision->AddLateralDecision("PathDecider/right-nudge",
                                              obstacle->Id(), object_decision);
            AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                              <<"] obstacle right-nudge , min_l = "<<boudary_l_min
                              <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
          }
          continue;
        }
      }

      *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle->Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                          <<"] obstacle nearest-stop, min_l = "<<boudary_l_min
                          <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
        has_stop_decision = true;

      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/ignore",
                                               obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                          <<"] obstacle ignore, min_l = "<<boudary_l_min
                          <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
      }
    } else if (FLAGS_enable_nudge_decision) {
      // nudge
      if (!has_cut_int_intetion && obstacle->Perception().is_moveble) {//moveble类型的障碍物并且与自车几乎平行，离自车较远，就忽略
        path_decision->AddLongitudinalDecision("PathDecider/not-in-lane",
                                             obstacle->Id(), object_decision);
        path_decision->AddLateralDecision("PathDecider/not-in-lane", obstacle->Id(),
                                          object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()<<"] obstacle is moveble , not-in-lane ";
      }
    

      ObjectDecisionType longitudinal_decision_last;
      if (!path_data.is_new() &&
        obstacle->nudge_l_buffer_min() >= FLAGS_static_decision_nudge_l_buffer//如果比这个小，说明降低了避障横向距离，不需要滞回
        && FindLongitudinalDecisionHistory(longitudinal_decision_last,obstacle->Id())){
        // && !obstacle->Perception().is_moveble) {
        if (longitudinal_decision_last.has_stop() && boudary_l_min <= 
                        half_width + FLAGS_static_decision_nudge_l_buffer_max) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"change nudge -> stop. ";
          *object_decision.mutable_stop() = GenerateObjectStopDecision(*obstacle);
          
          if (path_decision->MergeWithMainStop(
                  object_decision.stop(), obstacle->Id(),
                  reference_line_info_->reference_line(),
                  reference_line_info_->AdcSlBoundary())) {
            path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                                   obstacle->Id(), object_decision);
            AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                              <<"] obstacle nearest-stop, min_l = "<<boudary_l_min
                              <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
            has_stop_decision = true;
    
          } else {
            ObjectDecisionType object_decision;
            object_decision.mutable_ignore();
            path_decision->AddLongitudinalDecision("PathDecider/ignore",
                                                   obstacle->Id(), object_decision);
            AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                              <<"] obstacle ignore, min_l = "<<boudary_l_min
                              <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
          }
          continue;  
        }
      }
      
      if (curr_l > sl_boundary.end_l()) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                          <<"] obstacle left-nudge , min_l = "<<boudary_l_min
                          <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
      } else if (curr_l < sl_boundary.start_l()) {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle->Id(), object_decision);
        AINFO_IF(FLAGS_enable_debug_motion) << "PathDecider No["<<obstacle->Id()
                          <<"] obstacle right-nudge , min_l = "<<boudary_l_min
                          <<", nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min();
      }
    }
  }
  
  has_stop_decision_history_ = has_stop_decision;

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const Obstacle &path_obstacle) const {
  ObjectStop object_stop;

  const CarModel vehicle_param = EgoInfo::instance()->vehicle_param();
  double stop_distance = path_obstacle.GetStopDistance(vehicle_param);//MinRadiusStopDistance(vehicle_param);
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

bool PathDecider::FindLongitudinalDecisionHistory(ObjectDecisionType& lon_desicion_last, 
                     const std::string& id) const {
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
        if (last_obstacle->HasLongitudinalDecision()) {
          lon_desicion_last = last_obstacle->LongitudinalDecision();
          return true;
        }
      }
    }
  }
  return false;
}

bool PathDecider::FindLateralDecisionHistory(ObjectDecisionType& lat_desicion_last, 
                     const std::string& id) const {
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
