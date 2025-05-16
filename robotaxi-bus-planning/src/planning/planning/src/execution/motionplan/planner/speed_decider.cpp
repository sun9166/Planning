/**
 * @file speed_decider.cpp
 **/


//#include "ros/ros.h"
#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "decision.pb.h"

#include "speed_decider.h"
#include "common/util/util.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/scenario_context.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::math::Vec2d;
using acu::common::Status;

SpeedDecider::SpeedDecider() : Procedure("SpeedDecider") {}

bool SpeedDecider::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.planner_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  return true;
}

common::Status SpeedDecider::Execute(Frame* frame,
                                     ReferenceLineInfo* reference_line_info) {
  Procedure::Execute(frame, reference_line_info);
  ScenarioContext::eLonScenarioEnum current_scenario = ScenarioContext::eLonScenarioEnum::LANE_FOLLOWING;
  init_point_ = frame_->PlanningStartPoint();
  adc_sl_boundary_ = reference_line_info_->AdcSlBoundary();
  reference_line_ = &reference_line_info_->reference_line();
  if (FLAGS_enable_decision_replace_dp_speed_optimizer) {
    if (FLAGS_enable_plan_based_on_stmap && 
      (!reference_line_info_->IsChangeLanePath() 
        || reference_line_info_->IsChangeLanePath() && !BehaviorParser::instance()->has_decision_lateral_behavior())) {
      MakeObjectDecisionBasedOnStMap(reference_line_info->path_decision());
    } else {
      MakeObjectDecision(reference_line_info->path_decision());
    }
    
  } else if (!MakeObjectDecision(reference_line_info->speed_data(),
                          reference_line_info->path_decision())
           .ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AWARN_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  
  const auto& obstacles = reference_line_info->path_decision()->obstacles().Items();
  for (auto& obstacle : obstacles) {
    const auto& decision = obstacle->LongitudinalDecision();
    if (!obstacle->IsVirtual() && decision.has_stop()) {
      current_scenario = ScenarioContext::eLonScenarioEnum::STOP;
      break;
    }
  }

  ScenarioContext::instance()->UpdateLonScenario(current_scenario);       

  reference_line_info->path_decision()->GetMainFocusObstacle();//@pqg debug
  reference_line_info->path_decision()->GetMainYieldObstacle();//@pqg debug

  return Status::OK();
}

SpeedDecider::StPosition SpeedDecider::GetStPosition(
    const PathDecision* const path_decision, const SpeedInfo& speed_profile,
    const StBoundary& st_boundary) const {
  StPosition st_position = BELOW;
  if (st_boundary.IsEmpty()) {
    return st_position;
  }

  bool st_position_set = false;
  const double start_t = st_boundary.min_t();
  const double end_t = st_boundary.max_t();
  size_t speed_profile_size = speed_profile.size(); 
  for (size_t i = 0; i + 1 < speed_profile_size; ++i) {
    const STPoint curr_st(speed_profile[i].s(), speed_profile[i].t());
    const STPoint next_st(speed_profile[i + 1].s(), speed_profile[i + 1].t());
    if (curr_st.t() < start_t && next_st.t() < start_t) {
      continue;
    }
    if (curr_st.t() > end_t) {
      break;
    }

    common::math::LineSegment2d speed_line(curr_st, next_st);
    if (st_boundary.HasOverlap(speed_line)) {
      st_position = CROSS;
      if (st_boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
        if (!CheckKeepClearCrossable(path_decision, speed_profile,
                                     st_boundary)) {
          st_position = BELOW;
        }
      }
      break;
    }

    // note: st_position can be calculated by checking two st points once
    //       but we need iterate all st points to make sure there is no CROSS
    if (!st_position_set) {
      if (start_t < next_st.t() && curr_st.t() < end_t) {
        STPoint bd_point_front = st_boundary.upper_points().front();
        double side = common::math::CrossProd(bd_point_front, curr_st, next_st);
        st_position = side < 0.0 ? ABOVE : BELOW;
        st_position_set = true;
      }
    }
  }
  return st_position;
}

bool SpeedDecider::CheckKeepClearCrossable(
    const PathDecision* const path_decision, const SpeedInfo& speed_profile,
    const StBoundary& keep_clear_st_boundary) const {
  bool keep_clear_crossable = true;

  const auto& last_speed_point = speed_profile.back();
  double last_speed_point_v = 0.0;
  if (last_speed_point.has_v()) {
    last_speed_point_v = last_speed_point.v();
  } else {
    const size_t len = speed_profile.size();
    if (len > 1) {
      const auto& last_2nd_speed_point = speed_profile[len - 2];
      last_speed_point_v = (last_speed_point.s() - last_2nd_speed_point.s()) /
                           (last_speed_point.t() - last_2nd_speed_point.t());
    }
  }
  constexpr double kKeepClearSlowSpeed = 2.5;  // m/s
  if (last_speed_point.s() <= keep_clear_st_boundary.max_s() &&
      last_speed_point_v < kKeepClearSlowSpeed) {
    keep_clear_crossable = false;
  }
  return keep_clear_crossable;
}

bool SpeedDecider::CheckKeepClearBlocked(
    const PathDecision* const path_decision,
    const Obstacle& keep_clear_obstacle) const {
  bool keep_clear_blocked = false;

  // check if overlap with other stop wall
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto id = obstacle->Id();
    if (id == keep_clear_obstacle.Id()) {
      continue;
    }

    const auto& decision = obstacle->LongitudinalDecision();
    bool is_blocking_obstacle = false;
    if (decision.has_stop()){
      is_blocking_obstacle = true;
      AWARN_IF(FLAGS_enable_debug_motion)<<"id = "<<id<<"SetBlockingObstacle";
    } else {
      continue;
    }
    const CarModel vehicle_param = EgoInfo::instance()->vehicle_param();
    const double obstacle_start_s = obstacle->PerceptionSLBoundary().start_s();
    const double obstacle_stop_s = obstacle->GetStopDistance(vehicle_param);
    const double adc_length = EgoInfo::instance()->vehicle_length(); 
    const double remain_dis = obstacle_start_s - obstacle_stop_s;
    //block障碍物在静停区内，或者跟停距离在停止区内
    if (is_blocking_obstacle && remain_dis > keep_clear_obstacle.PerceptionSLBoundary().start_s()
         && remain_dis < keep_clear_obstacle.PerceptionSLBoundary().end_s() + adc_length) {
      keep_clear_blocked = true;
      break;
    }
  }
  return keep_clear_blocked;
}

bool SpeedDecider::IsFollowTooClose(const Obstacle& obstacle) const {
  if (!obstacle.IsBlockingObstacle()) {
    return false;
  }

  const double obs_speed = obstacle.speed();
  const double ego_speed = init_point_.v();
  if (obs_speed > ego_speed || obs_speed > 1.0) {
    return false;
  }
  const double deta_v = ego_speed - obs_speed;

  const double min_stop_distance_obstacle = std::fmin(std::fmax(0.0, 1.0*deta_v), 3.0);
  double distance = 
      obstacle.PerceptionSLBoundary().start_s() 
      - EgoInfo::instance()->vehicle_front_edge_to_center() 
      - min_stop_distance_obstacle;
  if (!obstacle.st_boundary().IsEmpty()) {
    distance = 
      obstacle.st_boundary().min_s()  - EgoInfo::instance()->vehicle_front_edge_to_center() 
      - min_stop_distance_obstacle;
  }    
  constexpr double decel = 1.0;//1.0;
  const double target_distance = std::pow((ego_speed - obs_speed), 2) * 0.5 / decel;
            //(ego_speed * ego_speed - obs_speed * obs_speed)/2/decel;
  AINFO_IF(FLAGS_enable_debug_motion)<<"distance = "<<distance
      <<", target distance = ("<<std::pow((ego_speed - obs_speed), 2) * 0.5 / decel<<", "<<target_distance<<"). ";
  return distance < target_distance;
}

//校验下让行的目标，以免因为误生成的预测线引发的刹车 dzx
bool SpeedDecider::IsYieldObstacleIgnore(const Obstacle& obstacle) const{
  int obstacle_type = obstacle.Perception().type;
  if (obstacle_type == 2 || obstacle_type == 3) return false ;
  const double ego_heading = EgoInfo::instance()->vehicle_state().heading;
  const double obs_heading = obstacle.heading();
  const common::SLPoint start_sl_point = reference_line_info_->init_sl_point();
  const SLBoundary obs_sl_boundary = obstacle.PerceptionSLBoundary();
  //横向距离
  double lateral_dis = std::fmin(std::fabs(obs_sl_boundary.start_l()-start_sl_point.l()), 
                                    std::fabs(obs_sl_boundary.end_l()-start_sl_point.l()))
                                    - EgoInfo::instance()->vehicle_width() / 2.0;
  //航向角偏差
  double deta_heading = ego_heading - obs_heading;
  //障碍物航向和自车基本平行，同时横向距离>5m
  if (lateral_dis > 5 && (std::fabs(deta_heading) < 10 || std::fabs(deta_heading - 180) < 10)){
    AWARN_IF(FLAGS_enable_debug_motion) << "***YieldObstacle : " << obstacle.Id() << "can be ignore!! ";
    return true;
  }
  return false;
}

Status SpeedDecider::MakeObjectDecision(
    const SpeedInfo& speed_profile, PathDecision* const path_decision) const {
  if (speed_profile.size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AWARN_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }
  
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->st_boundary();

    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }

    auto position = GetStPosition(path_decision, speed_profile, boundary);
    if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      if (CheckKeepClearBlocked(path_decision, *obstacle)) {
        position = BELOW;
      }
    }

    auto box = obstacle->PerceptionBoundingBox();
    common::SLPoint start_sl_point;
    reference_line_->XYToSL({box.center_x(), box.center_y()}, &start_sl_point);
    double start_abs_l = std::abs(start_sl_point.l());

    switch (position) {
      case BELOW:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                      stop_decision);
          }
        } else if (CheckIsFollowByT(boundary) &&
                   (boundary.max_t() - boundary.min_t() >
                    FLAGS_follow_min_time_sec) &&
                   start_abs_l < FLAGS_follow_min_obs_lateral_distance) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*mutable_obstacle)) {
            ObjectDecisionType stop_decision;
            AWARN_IF(FLAGS_enable_debug_motion)<< "is follow too close ! stop in front of No["
                       <<obstacle->Id()<<"] obstacle .";
            double stop_distance = -3;
            if (mutable_obstacle->speed() > 1.0 && !mutable_obstacle->IsStatic()) {
              stop_distance = -std::fmin(std::fmax(0.0, 1.0*(init_point_.v() - mutable_obstacle->speed())), 3.0);
            }             
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   stop_distance)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          } else {  // high speed or low speed accelerating
            // FOLLOW decision
            ObjectDecisionType follow_decision;
            AINFO_IF(FLAGS_enable_debug_motion)<<"follow No["<<obstacle->Id()<<"] obstacle.";
            if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                        follow_decision);
            }
          }
        } else {
          // YIELD decision
          ObjectDecisionType yield_decision;
          AINFO_IF(FLAGS_enable_debug_motion)<< "yield No["<<obstacle->Id()<<"] obstacle.";
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
        break;
      case ABOVE:
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
          ObjectDecisionType ignore;
          ignore.mutable_ignore();
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph", ignore);
        } else {
          // OVERTAKE decision
          ObjectDecisionType overtake_decision;
          AINFO_IF(FLAGS_enable_debug_motion) << "overtake No["<<obstacle->Id()<<"] obstacle.";
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        break;
      case CROSS:
        if (mutable_obstacle->IsBlockingObstacle()) {
          ObjectDecisionType stop_decision;
          if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 -FLAGS_min_stop_distance_obstacle)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/cross",
                                                      stop_decision);
          }
          const std::string msg =
              "Failed to find a solution for crossing obstacle:" +
              mutable_obstacle->Id();
          AWARN_IF(FLAGS_enable_debug_motion) << msg;
          return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
        }
        break;
      default:
        AWARN_IF(FLAGS_enable_debug_motion) << "Unknown position:" << position;
    }
    AppendIgnoreDecision(mutable_obstacle);
  }
  return Status::OK();
}

Status SpeedDecider::MakeObjectDecision(PathDecision* const path_decision) const {//使用struct_decision的结果生成障碍物的决策
  auto bp_ptr = BehaviorParser::instance();
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->st_boundary();
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
      boundary.max_t() < 0.0 || boundary.min_t() >= 8.0) {
      if (!(mutable_obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::GIVEWAY) || 
        mutable_obstacle->reference_line_st_boundary().IsEmpty() || reference_line_info_->path_data().is_new()) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
              <<"] , because boundary empty or boundary is out of range. "; 
        AppendIgnoreDecision(mutable_obstacle);
        continue;
      }
      
    }
    if (obstacle->HasLongitudinalDecision()) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
           <<"] , because it has LongitudinalDecision already . "; 
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    //用前端的对障碍物的决策结果来填充障碍物的纵向决策
    switch (mutable_obstacle->BehaviorLongitudinalDecision()) {
    case eObjectDecisionEnum::IGNORE: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is IGNORE "; 
      {
        if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
         if (CheckKeepClearBlocked(path_decision, *obstacle)) {
           ObjectDecisionType stop_decision;
           if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
             mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                       stop_decision);
           }
         }
       } else if (FLAGS_enable_decision_ignore_obstacle) {
         AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
            <<"] , because decision ignore it . "; 
         if (!mutable_obstacle->IsStatic()) {
           AppendIgnoreDecision(mutable_obstacle);
         }
      
       }
      }
      break;  
    case eObjectDecisionEnum::TAKEWAY: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is TAKEOVER .. "; 
      {
        if (reference_line_info_->trajectory_type() ==
          ADCTrajectory::PATH_FALLBACK 
          && reference_line_info_->IsChangeLanePath()
          && bp_ptr->has_decision_lateral_behavior()) {//换道路径规划失败时忽略超车决策
          AWARN_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
           <<"] , because lane chenge path plan failed. "; 
          AppendIgnoreDecision(mutable_obstacle);
        } else {
          ObjectDecisionType overtake_decision;
          if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                      overtake_decision);
          }
        }
        
      }
      break;
    case eObjectDecisionEnum::GIVEWAY: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is GIVEWAY .. "; 
      {
        if (!IsYieldObstacleIgnore(*mutable_obstacle)) {
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
      }
      break;
    case eObjectDecisionEnum::FOLLOW: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is FOLLOW .. "; 
      {            
        // stop for low_speed decelerating
        if (IsFollowTooClose(*mutable_obstacle)) {
          ObjectDecisionType stop_decision;
          AWARN_IF(FLAGS_enable_debug_motion)<< "is follow too close ! stop in front of No["
                     <<obstacle->Id()<<"] obstacle .";
          double stop_distance = -3;
          if (mutable_obstacle->speed() > 1.0 && !mutable_obstacle->IsStatic()) {
            stop_distance = -std::fmin(std::fmax(0.0, 1.0*(init_point_.v() - mutable_obstacle->speed())), 3.0);
          }            
          if (FLAGS_enable_follow_obj_status_estimation) {
            const double obj_speed = obstacle->speed_along_reference_line();
            const double deceleration_estimate = std::fmin(-1.0, obstacle->acc());
            double stop_distance_estimate = fabs(obj_speed*obj_speed/2/deceleration_estimate);
            AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance_estimate : "<<stop_distance_estimate;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 stop_distance, stop_distance_estimate)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }  
          } else {
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   stop_distance)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          }           
        } else {  // high speed or low speed accelerating
          // FOLLOW decision
          ObjectDecisionType follow_decision;
          AINFO_IF(FLAGS_enable_debug_motion)<<"follow No["<<obstacle->Id()<<"] obstacle.";
          if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      follow_decision);
          }
        }
      }
      break;  
    default:
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], no decision "; 
      { //默认为跟随决策 
        auto box = obstacle->PerceptionBoundingBox();
        common::SLPoint start_sl_point;
        reference_line_->XYToSL({box.center_x(), box.center_y()}, &start_sl_point);
        double start_abs_l = std::abs(start_sl_point.l());
        if (CheckIsFollowByT(boundary) &&
                   (boundary.max_t() - boundary.min_t() >
                    FLAGS_follow_min_time_sec) &&
                   start_abs_l < FLAGS_follow_min_obs_lateral_distance
                   && obstacle->PerceptionSLBoundary().end_s() > 0) {
          // stop for low_speed decelerating
          if (IsFollowTooClose(*mutable_obstacle)) {
            ObjectDecisionType stop_decision;
            AWARN_IF(FLAGS_enable_debug_motion)<< "is follow too close ! stop in front of No["
                       <<obstacle->Id()<<"] obstacle .";
            double stop_distance = -3;
            if (mutable_obstacle->speed() > 1.0 && !mutable_obstacle->IsStatic()) {
              stop_distance = -std::fmin(std::fmax(0.0, 1.0*(init_point_.v() - mutable_obstacle->speed())), 3.0);
            }              
            if (FLAGS_enable_follow_obj_status_estimation) {
              const double obj_speed = obstacle->speed_along_reference_line();
              const double deceleration_estimate = std::fmin(-1.0, obstacle->acc());
              double stop_distance_estimate = fabs(obj_speed*obj_speed/2/deceleration_estimate);
              AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance_estimate : "<<stop_distance_estimate;
              if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   stop_distance, stop_distance_estimate)) {
                mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                          stop_decision);
              }  
            } else {
              if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                     stop_distance)) {
                mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                          stop_decision);
              }
            }           
          } else {  // high speed or low speed accelerating
          //   // FOLLOW decision
            ObjectDecisionType follow_decision;
            AINFO_IF(FLAGS_enable_debug_motion)<<"follow No["<<obstacle->Id()<<"] obstacle.";
            if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                        follow_decision);
            }
          }
        } 
      }
    }
  }
  return Status::OK();
}

Status SpeedDecider::MakeObjectDecisionBasedOnStMap(PathDecision* const path_decision) const {//使用struct_decision的结果生成障碍物的决策
  auto bp_ptr = BehaviorParser::instance();
  for (const auto* obstacle : path_decision->obstacles().Items()) {
    auto* mutable_obstacle = path_decision->Find(obstacle->Id());
    const auto& boundary = mutable_obstacle->st_boundary();
    if (obstacle->HasLongitudinalDecision()) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
           <<"] , because it has LongitudinalDecision already . "; 
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    //用前端的对障碍物的决策结果来填充障碍物的纵向决策
    switch (mutable_obstacle->BehaviorLongitudinalDecision()) {
    case eObjectDecisionEnum::IGNORE: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is IGNORE "; 
      {
        if (FLAGS_enable_decision_ignore_obstacle) {
          if (!mutable_obstacle->IsStatic()) {
            AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
              <<"] , because decision ignore it . "; 
            AppendIgnoreDecision(mutable_obstacle);
          }
        }
      }
      break;  
    case eObjectDecisionEnum::TAKEWAY: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is TAKEOVER .. "; 
      {
        ObjectDecisionType overtake_decision;
        if (CreateOvertakeDecision(*mutable_obstacle, &overtake_decision)) {
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph/overtake",
                                                    overtake_decision);
        }
      }
      break;
    case eObjectDecisionEnum::GIVEWAY: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is GIVEWAY .. "; 
      {
        if (!IsYieldObstacleIgnore(*mutable_obstacle)) {
          ObjectDecisionType yield_decision;
          if (CreateYieldDecision(*mutable_obstacle, &yield_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      yield_decision);
          }
        }
      }
      break;
    case eObjectDecisionEnum::FOLLOW: 
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"], decision is FOLLOW .. "
       << "obstacle->acc():" << obstacle->acc(); 
      {            
        // stop for low_speed decelerating
        if (IsFollowTooClose(*mutable_obstacle)) {
          ObjectDecisionType stop_decision;
          AWARN_IF(FLAGS_enable_debug_motion)<< "is follow too close ! stop in front of No["
                     <<obstacle->Id()<<"] obstacle ." ;
          double stop_distance = -3;
          if (mutable_obstacle->speed() > 1.0 && !mutable_obstacle->IsStatic()) {
            stop_distance = -std::fmin(std::fmax(0.0, 1.0*(init_point_.v() - mutable_obstacle->speed())), 3.0);
          }           
          if (FLAGS_enable_follow_obj_status_estimation) {
            const double obj_speed = obstacle->speed_along_reference_line();
            const double deceleration_estimate = std::fmin(-1.0, obstacle->acc());
            double stop_distance_estimate = fabs(obj_speed*obj_speed/2/deceleration_estimate);
            AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance_estimate : "<<stop_distance_estimate;
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                 stop_distance, stop_distance_estimate)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }  
          } else {
            if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                   stop_distance)) {
              mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                        stop_decision);
            }
          }           
        } else {  // high speed or low speed accelerating
          // FOLLOW decision
          ObjectDecisionType follow_decision;
          AINFO_IF(FLAGS_enable_debug_motion)<<"follow No["<<obstacle->Id()<<"] obstacle.";
          if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
            mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                      follow_decision);
          }
        }
      }
      break;  
    default:
      AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<mutable_obstacle->Id()<<"], no decision "; 
      if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<mutable_obstacle->Id()<<"], KEEP_CLEAR ";
         if (CheckKeepClearBlocked(path_decision, *mutable_obstacle)) {
           ObjectDecisionType stop_decision;
           if (CreateStopDecision(*mutable_obstacle, &stop_decision, 0.0)) {
             mutable_obstacle->AddLongitudinalDecision("dp_st_graph/keep_clear",
                                                       stop_decision);
           }
         }
      } else if (FLAGS_enable_decision_ignore_obstacle) {
        if (!mutable_obstacle->IsStatic()) {
          const PathInfo path_data = reference_line_info_->GetPathInfo();
          const double obstacle_center_s = (obstacle->PerceptionSLBoundary().start_s() 
                                  + obstacle->PerceptionSLBoundary().end_s()) / 2.0;
          const double frenet_point_l = path_data.frenet_frame_path().EvaluateByS(obstacle_center_s).l();                        
          const SLBoundary obs_sl_boundary = obstacle->PerceptionSLBoundary();
          //横向距离
          double lateral_dis = std::fmin(std::fabs(obs_sl_boundary.start_l()-frenet_point_l), 
                                  std::fabs(obs_sl_boundary.end_l()-frenet_point_l))
                                  - EgoInfo::instance()->vehicle_width() / 2.0;
          AINFO_IF(FLAGS_enable_debug_motion)<<"lateral_dis :"<< lateral_dis;
          if (lateral_dis > 0.2) {
            AINFO_IF(FLAGS_enable_debug_motion)<<"ignore No["<<obstacle->Id()
            <<"] , because decision ignore it . "; 
            AppendIgnoreDecision(mutable_obstacle);
          } else {
            if (IsFollowTooClose(*mutable_obstacle)) {
              ObjectDecisionType stop_decision;
              AWARN_IF(FLAGS_enable_debug_motion)<< "is follow too close ! stop in front of No["
                        <<obstacle->Id()<<"] obstacle ." ;
              double stop_distance = -3;
              if (mutable_obstacle->speed() > 1.0 && !mutable_obstacle->IsStatic()) {
                stop_distance = -std::fmin(std::fmax(0.0, 1.0*(init_point_.v() - mutable_obstacle->speed())), 3.0);
              }           
              if (FLAGS_enable_follow_obj_status_estimation) {
                const double obj_speed = obstacle->speed_along_reference_line();
                const double deceleration_estimate = std::fmin(-1.0, obstacle->acc());
                double stop_distance_estimate = fabs(obj_speed*obj_speed/2/deceleration_estimate);
                AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance_estimate : "<<stop_distance_estimate;
                if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                    stop_distance, stop_distance_estimate)) {
                  mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                            stop_decision);
                }  
              } else {
                if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                                      stop_distance)) {
                  mutable_obstacle->AddLongitudinalDecision("dp_st_graph/too_close",
                                                            stop_decision);
                }
              }           
            } else {  
              // FOLLOW decision
              ObjectDecisionType follow_decision;
              AINFO_IF(FLAGS_enable_debug_motion)<<"follow No["<<obstacle->Id()<<"] obstacle.";
              if (CreateFollowDecision(*mutable_obstacle, &follow_decision)) {
                mutable_obstacle->AddLongitudinalDecision("dp_st_graph",
                                                          follow_decision);
              }
            }
          }
        }
      }
    }
  }
  return Status::OK();
}

void SpeedDecider::AppendIgnoreDecision(Obstacle* obstacle) const {
  ObjectDecisionType ignore_decision;
  ignore_decision.mutable_ignore();
  if (!obstacle->HasLongitudinalDecision()) {
    obstacle->AddLongitudinalDecision("dp_st_graph", ignore_decision);
  }
  if (!obstacle->HasLateralDecision()) {
    obstacle->AddLateralDecision("dp_st_graph", ignore_decision);
  }
}

bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                                      ObjectDecisionType* const stop_decision,
                                      double stop_distance) const {
  DCHECK_NOTNULL(stop_decision);

  const auto& boundary = obstacle.st_boundary();
  double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() + stop_distance;
  if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s();
  }
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    // AINFO_IF(FLAGS_enable_debug_motion)<< "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }
  return true;
}

bool SpeedDecider::CreateStopDecision(const Obstacle& obstacle,
                          ObjectDecisionType* const stop_decision,
                          double stop_distance, 
                          double stop_distance_buffer_estimate) const {
  
  DCHECK_NOTNULL(stop_decision);

  const auto& boundary = obstacle.st_boundary();
  double fence_s = adc_sl_boundary_.end_s() + boundary.min_s() 
                   + stop_distance_buffer_estimate + stop_distance;
  if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
    fence_s = obstacle.PerceptionSLBoundary().start_s();
  }
  const double main_stop_s =
      reference_line_info_->path_decision()->stop_reference_line_s();
  if (main_stop_s < fence_s) {
    // ADEBUG << "Stop fence is further away, ignore.";
    return false;
  }

  const auto fence_point = reference_line_->GetReferencePoint(fence_s);

  // set STOP decision
  auto* stop = stop_decision->mutable_stop();
  stop->set_distance_s(stop_distance);
  auto* stop_point = stop->mutable_stop_point();
  stop_point->set_x(fence_point.x());
  stop_point->set_y(fence_point.y());
  stop_point->set_z(0.0);
  stop->set_stop_heading(fence_point.heading());

  if (boundary.boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
    stop->set_reason_code(StopReasonCode::STOP_REASON_CLEAR_ZONE);
  }

  return true;
}

bool SpeedDecider::CreateFollowDecision(
    const Obstacle& obstacle, ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);

  const double follow_speed = init_point_.v();
  //const double follow_speed = obstacle.speed();
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);//跟车距离？

  const auto& boundary = obstacle.st_boundary();
  double reference_s = /*adc_sl_boundary_.end_s() +*/ 
         obstacle.PerceptionSLBoundary().start_s() + follow_distance_s;
  AINFO_IF(FLAGS_enable_debug_motion) << "follow_distance_s= " <<follow_distance_s 
                                      << " ,adc_sl_boundary_= " << adc_sl_boundary_.end_s()
                                      << " ,obstacle s= "<<obstacle.PerceptionSLBoundary().start_s()
                                      << " ,reference_s= " << reference_s;
  const double main_stop_s =
        reference_line_info_->path_decision()->stop_reference_line_s();
  // 停车点在跟随目标的距离前面，不能以跟随的距离点判断  
  if (main_stop_s - EgoInfo::instance()->vehicle_front_edge_to_center()  < obstacle.PerceptionSLBoundary().end_s() ) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Follow obstacle is overtake stop_point, ignore.";
    //return false;
  }

  auto ref_point = reference_line_->GetReferencePoint(reference_s);

  // set FOLLOW decision
  auto* follow = follow_decision->mutable_follow();
  follow->set_distance_s(follow_distance_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CreateYieldDecision(
    const Obstacle& obstacle, ObjectDecisionType* const yield_decision) const {
  DCHECK_NOTNULL(yield_decision);
  int obstacle_type = obstacle.Perception().type;

  double yield_distance = FLAGS_yield_distance;//frame_->PlanningStartPoint().v();
  switch (obstacle_type) {
    case 0:
      yield_distance = std::max(yield_distance,FLAGS_yield_distance);
      break;
    case 2:// case PEDESTRIAN
    case 3:// case CYCLIST
      yield_distance = std::max(yield_distance,FLAGS_yield_distance_pedestrian_bycicle);
      break;
    default:
      yield_distance = std::max(yield_distance,FLAGS_yield_distance);
      break;
  }

  // const auto& obstacle_boundary = obstacle.st_boundary();
  auto obstacle_boundary = FLAGS_using_cognition_st_boundary && 
      !obstacle.reference_line_st_boundary().IsEmpty() && 
      !reference_line_info_->path_data().is_new() ?
      obstacle.reference_line_st_boundary() : obstacle.st_boundary();
  if (obstacle_boundary.IsEmpty()) {
    obstacle_boundary = obstacle.reference_line_st_boundary();
  }	 

  double yield_distance_s = -yield_distance;//default value
  double reference_line_fence_s =
        adc_sl_boundary_.end_s() + yield_distance_s;
  if (!obstacle_boundary.IsEmpty()) {
    
    yield_distance_s =
      std::max(-obstacle_boundary.min_s(), -yield_distance);

    reference_line_fence_s =
        adc_sl_boundary_.end_s() + obstacle_boundary.min_s() + yield_distance_s;
    const double main_stop_s =
        reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < reference_line_fence_s) {
      // ADEBUG << "Yield reference_s is further away, ignore.";
      return false;
    }
  } 
  
  //if use stmap , info provided by below is not used 
  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set YIELD decision
  auto* yield = yield_decision->mutable_yield();
  yield->set_distance_s(yield_distance_s);
  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CreateOvertakeDecision(
    const Obstacle& obstacle,
    ObjectDecisionType* const overtake_decision) const {
  DCHECK_NOTNULL(overtake_decision);

  constexpr double kOvertakeTimeBuffer = 0.8;//3.0;    // in seconds
  constexpr double kMinOvertakeDistance = 2.0;  // in meters
  constexpr double kMaxOvertakeDistance = 10.0;  // in meters

  const double obstacle_speed =
          obstacle.Perception().speed;  //@pqg 更改

  double overtake_distance_s = std::fmin(
      std::fabs(init_point_.v() - obstacle_speed) * kOvertakeTimeBuffer,
      kMaxOvertakeDistance);
  overtake_distance_s = std::fmax(overtake_distance_s, kMinOvertakeDistance);
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"overtake_distance_s = "<<overtake_distance_s;

  const auto& boundary = obstacle.st_boundary();
  double reference_line_fence_s = overtake_distance_s;//default_value
  if (!boundary.IsEmpty()) {
    reference_line_fence_s =
        adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;
    const double main_stop_s =
        reference_line_info_->path_decision()->stop_reference_line_s();
    if (main_stop_s < reference_line_fence_s) {
      // ADEBUG << "Overtake reference_s is further away, ignore.";
      return false;
    }
  }

  //if use stmap , info provided by below is not used 
  auto ref_point = reference_line_->GetReferencePoint(reference_line_fence_s);

  // set OVERTAKE decision
  auto* overtake = overtake_decision->mutable_overtake();
  overtake->set_distance_s(overtake_distance_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  return true;
}

bool SpeedDecider::CheckIsFollowByT(const StBoundary& boundary) const {
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  constexpr double kFollowTimeEpsilon = 1e-3;
  constexpr double kFollowCutOffTime = 0.5;
  if (boundary.min_t() > kFollowCutOffTime ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace acu
