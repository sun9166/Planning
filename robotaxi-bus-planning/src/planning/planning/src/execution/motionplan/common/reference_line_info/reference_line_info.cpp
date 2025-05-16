/** 
 * @file reference_line_info.cpp
 **/

#include "sl_boundary.pb.h"

//#include "ros/ros.h"
#include "common/util/util.h"
#include "reference_line_info.h"
#include "common/util/string_util.h"
#include "src/execution/cognition/struct_cognition/conf/cognition_gflags.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"

namespace acu {
namespace planning {

using acu::common::TrajectoryPoint;
using acu::common::math::Box2d;
using acu::common::math::Vec2d;

ReferenceLineInfo::ReferenceLineInfo(const VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const TrajectoryPoint& pathplan_init_point,
                                     const ReferenceLine& reference_line,
                                     const std::string id,
                                     const LanePosition position,
                                     const std::pair<eActionEnum,eDirectionEnum> action,
                                     const double start_time,
                                     const bool reverse_flag)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      pathplan_init_point_(pathplan_init_point),
      reference_line_(reference_line),
      id_(id),
      position_(position),
      lateral_action_(action),
      start_time_(start_time) {
        path_data_.SetReverse(reverse_flag);
      }

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  double back_edge_to_center  = EgoInfo::instance()->vehicle_back_edge_to_center();
  double front_edge_to_center = EgoInfo::instance()->vehicle_front_edge_to_center();
  double left_edge_to_center  = EgoInfo::instance()->vehicle_width() / 2.0;
  double right_edge_to_center = left_edge_to_center;
  // stitching point
  const auto& path_point = adc_planning_point_.path_point();
  Vec2d position(path_point.x(), path_point.y());
  Vec2d vec_to_center(
      (front_edge_to_center - back_edge_to_center) / 2.0,
      (left_edge_to_center - right_edge_to_center) / 2.0);
  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
  Box2d box(center, path_point.theta(), 
          EgoInfo::instance()->vehicle_length(), EgoInfo::instance()->vehicle_width());
  // realtime vehicle position
  Box2d vehicle_box = EgoInfo::instance()->ego_box();
  
  // 1.GET sl_boundary_info_.vehicle_sl_boundary_
  if (!reference_line_.GetSLBoundary(vehicle_box,
                                     &sl_boundary_info_.vehicle_sl_boundary_)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Failed to get ADC boundary from vehicle_box(realtime position "
           <<  "of the car): ";
    return false;
  }
  // 2.GET sl_boundary_info_.adc_sl_boundary_
  if (!reference_line_.GetSLBoundary(box,
                                     &sl_boundary_info_.adc_sl_boundary_)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Failed to get ADC boundary from box: ";
    return false;
  }

  if (sl_boundary_info_.adc_sl_boundary_.end_s() < 0 ||
      sl_boundary_info_.adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Vehicle SL "
          << sl_boundary_info_.adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }

  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
  if (sl_boundary_info_.adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      sl_boundary_info_.adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Ego vehicle is too far away from reference line.";
    return false;
  }

  if (FLAGS_enable_two_stitching_trajectory) { 
    if (!reference_line_.XYToSL({pathplan_init_point_.path_point().x(), pathplan_init_point_.path_point().y()},&start_sl_point_)) {
      return false;
    }
    if (!reference_line_.XYToSL({path_point.x(), path_point.y()},&speed_plan_start_sl_point_)) {
      return false;
    }
  } else {
    if (!reference_line_.XYToSL({path_point.x(), path_point.y()},&start_sl_point_)) {
      return false;
    }
    speed_plan_start_sl_point_ = start_sl_point_;
  }

  if (FLAGS_enable_smooth_reference_line) {
    if (!reference_line_.XYToSL(
       {EgoInfo::instance()->vehicle_state().x, EgoInfo::instance()->vehicle_state().y},&car_sl_)) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"Can't get car sl !!!";
      return false;
    }
  } else {
    car_sl_.set_s(0.0);
    car_sl_.set_l(0.0);
  }
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"init point sl = ("<<start_sl_point_.s()<<", "<<start_sl_point_.l()<<"). "
                                     <<", speed_plan_start_sl_point = ("<<speed_plan_start_sl_point_.s()
                                     <<", "<<speed_plan_start_sl_point_.l()<<"), "
                                     <<" car sl = ("<<car_sl_.s()<<", "<<car_sl_.l();

  is_on_reference_line_ =
      reference_line_.IsOnLane(sl_boundary_info_.adc_sl_boundary_);   
  if (!AddObstacles(obstacles)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Failed to add obstacles to reference line";
    return false;
  }

  if (!BehaviorParser::instance()->decision_speed_limits().empty()) {
    for (const auto& speed_limit : BehaviorParser::instance()->decision_speed_limits()) {
      reference_line_.AddSpeedLimit(get<0>(speed_limit),
                                    get<1>(speed_limit),
                                    get<2>(speed_limit));
    }
  }

  //如果有静止障碍物的最小避障距离＜默认值，为了避免横向的波动导致的限速波动，这里只要没切换参考线则认为都需要限速。
  
  // if (!BehaviorParser::instance()->target_reference_line_change()) {
  //   for (const auto* obstacle : path_decision_.obstacles().Items()) {
  //     if (obstacle->nudge_l_buffer_min() < FLAGS_static_decision_nudge_l_buffer
  //        && obstacle->PerceptionSLBoundary().end_s() > 0.0
  //        && obstacle->IsStatic()) {
  //       AWARN_IF(FLAGS_enable_debug_motion)
  //           <<"nudge_l_buffer_min = "<<obstacle->nudge_l_buffer_min()<<" < default value, limit passby speed.";
  //       double end_s = 1000.0;    
  //       reference_line_.AddSpeedLimit(
  //           0.0, end_s, FLAGS_obstacle_extreme_nudge_speed);
  //       break;
  //     }
  //   }
  // }
  
  is_safe_to_change_lane_ = false;

  if (IsChangeLanePath() && FLAGS_enable_change_lane_safety_check) {//换道的校验
    is_safe_to_change_lane_ = CheckChangeLane();
  }

  SDistanceToFirstStaticObstacle();//@pqg add 

  is_inited_ = true;
  return true;
}

void ReferenceLineInfo::CheckCurrentPathAccess(const vector<const Obstacle*> obstacles_vec) {
  if (obstacles_vec.empty()) {
    EgoInfo::instance()->UpdateCurrentPathStatus(true);
    return;
  }
  // 当前前车道存在静态障碍物，或者旁车道存在压线或接近压线的车速低于自车的车辆
  for(const auto* const_obstacle : obstacles_vec) {
    double obstacle_center_s = (const_obstacle->PerceptionSLBoundary().start_s() 
                            + const_obstacle->PerceptionSLBoundary().end_s()) / 2.0;
    const double frenet_point_l = path_data_.frenet_frame_path().EvaluateByS(obstacle_center_s).l();
    // 当前前车道存在静态障碍物
    if ((const_obstacle->IsStatic() || const_obstacle->speed() < 0.5 || const_obstacle->IsUltraStatic())){
      if (const_obstacle->PerceptionSLBoundary().start_s() > DistanceToStopPoint() 
        || const_obstacle->PerceptionSLBoundary().start_s() > 6.0 * fabs(vehicle_state_.linear_velocity)) {
        continue;
      }
      double static_obs_passby_min_l = frenet_point_l - EgoInfo::instance()->vehicle_param().half_wheel 
                    - FLAGS_static_decision_nudge_l_buffer_max;
      double static_obs_passby_max_l = frenet_point_l + EgoInfo::instance()->vehicle_param().half_wheel 
                    + FLAGS_static_decision_nudge_l_buffer_max;
      if (const_obstacle->PerceptionSLBoundary().end_s() < car_sl_.s()) continue;
      if (const_obstacle->PerceptionSLBoundary().end_l() < static_obs_passby_min_l || 
        const_obstacle->PerceptionSLBoundary().start_l() > static_obs_passby_max_l) {
        continue;
      }
      AWARN_IF(FLAGS_enable_debug_motion)<< "!!!Impact on traffic, const_obstacle_id = " << const_obstacle->Id();
      EgoInfo::instance()->UpdateCurrentPathStatus(false);
      return;
    }
    // 速度比自车高的动态障碍物
    if (const_obstacle->speed() > fabs(vehicle_state_.linear_velocity) + 1.0 && 
          const_obstacle->PerceptionSLBoundary().start_s() > car_sl_.s()) continue;
    double ttc = (car_sl_.s() - const_obstacle->PerceptionSLBoundary().end_s()) 
            /(const_obstacle->speed() -  fabs(vehicle_state_.linear_velocity) + FLAGS_numerical_epsilon);
    if (const_obstacle->PerceptionSLBoundary().end_s() < car_sl_.s() && (ttc < 0 || ttc > 6.0)) continue;
    // 在规划轨迹上的动态障碍物，肯定已经有决策结果了，会做响应的处理，不影响安全
    if (((const_obstacle->PerceptionSLBoundary().start_l()-frenet_point_l)
          *(const_obstacle->PerceptionSLBoundary().end_l()-frenet_point_l)) < 0) continue;
    double dynamic_obs_passby_min_l = frenet_point_l - EgoInfo::instance()->vehicle_param().half_wheel 
                    - FLAGS_dynamic_obstacle_lateral_ignore_buffer;
    double dynamic_obs_passby_max_l = frenet_point_l + EgoInfo::instance()->vehicle_param().half_wheel 
                    + FLAGS_dynamic_obstacle_lateral_ignore_buffer;
    if (const_obstacle->PerceptionSLBoundary().end_l() < dynamic_obs_passby_min_l || 
        const_obstacle->PerceptionSLBoundary().start_l() > dynamic_obs_passby_max_l)  continue;
    // 剩下的是在自车轨迹两侧, 不可忽略的动态障碍物
    AWARN_IF(FLAGS_enable_debug_motion)<< "!!!Impact on traffic, const_obstacle_id = " << const_obstacle->Id();
    EgoInfo::instance()->UpdateCurrentPathStatus(false);
    return;
  }
  EgoInfo::instance()->UpdateCurrentPathStatus(true);
  return;
}

void ReferenceLineInfo::AddPassByObstacleSpeedlimit() {
  int passby_flag = 0;
  static int passby_flag_last = 0;
  static string last_passby_obs_id = "nothing";
  double passby_limit_s = 100;
  double passby_limit_v = 30;
  auto vehicle_param = EgoInfo::instance()->vehicle_param();
  const auto& obstacles = path_decision_.whole_obstacles();
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();
  vector<const Obstacle*> obstacles_vec = obstacles.Items();
  std::sort(obstacles_vec.begin(), obstacles_vec.end(),
          [](const Obstacle* lhs, const Obstacle* rhs) {
            if (fabs(lhs->PerceptionSLBoundary().start_s() - rhs->PerceptionSLBoundary().start_s()) > 1e-3) {
              return lhs->PerceptionSLBoundary().start_s() < rhs->PerceptionSLBoundary().start_s();
            } else {
              return rhs->PerceptionSLBoundary().end_s() < rhs->PerceptionSLBoundary().end_s();
            }
          });
  CheckCurrentPathAccess(obstacles_vec);
  for (const auto* const_obstacle : obstacles_vec) {
    if (const_obstacle->IsVirtual() || const_obstacle->IsStatic() || const_obstacle->speed() < 0.5) {
      continue;
    }
    /* ref line:
     * -------------------------------
     *    start_s   end_s
     * ------|  adc   |---------------
     * ------------|  obstacle |------
     */
    if (const_obstacle->Perception().type != 4 && passby_flag == 0
      && ( const_obstacle->BehaviorLongitudinalDecision() != eObjectDecisionEnum::TAKEWAY
        && const_obstacle->BehaviorLongitudinalDecision() != eObjectDecisionEnum::SIDEPASS
        )) {
      double back_edge_to_center = vehicle_param.back_over_hang;
      double front_edge_to_center = vehicle_param.length - back_edge_to_center;
      //过滤掉车前悬后的障碍物和距离较远的障碍物
      if (const_obstacle->PerceptionSLBoundary().end_s() < front_edge_to_center || 
          const_obstacle->PerceptionSLBoundary().start_s() > 5 + 3.5 * fabs(vehicle_state_.linear_velocity)) {
        continue;
      }
      double dis_range = 0.6;  // meters
      double dis_delt_range = 0.2;//meters
      double lane_left_width = 0;
      double lane_right_width = 0;
      double lane_width = 3.5;
      double obstacle_center_s = (const_obstacle->PerceptionSLBoundary().start_s() 
                                    + const_obstacle->PerceptionSLBoundary().end_s()) / 2.0;
      if (reference_line_.GetLaneWidth(obstacle_center_s, &lane_left_width, &lane_right_width)) {
        lane_width = lane_left_width + lane_right_width;
        if(lane_width < 3){
          lane_width = 3;
        }else if(lane_width > 3.5){
          lane_width = 3.5;
        }
        dis_range = (lane_width - vehicle_param.car_width) * 0.5 + dis_delt_range;
      }
        
      const double frenet_point_l = path_data_.frenet_frame_path().EvaluateByS(obstacle_center_s).l();
      double passby_min_l = frenet_point_l - vehicle_param.half_wheel - dis_range;
      double passby_max_l = frenet_point_l + vehicle_param.half_wheel + dis_range;
      //完全在车道内的障碍物
      if (const_obstacle->PerceptionSLBoundary().end_l() < passby_max_l && 
        const_obstacle->PerceptionSLBoundary().start_l() > passby_min_l) {
        continue;
      }
      // if (passby_flag_last == 1 && const_obstacle->Id() == last_passby_obs_id) {
      //   passby_min_l -= dis_delt_range;
      //   passby_max_l += dis_delt_range;
      // }

      //完全在车道外的障碍物
      if (const_obstacle->PerceptionSLBoundary().end_l() < passby_min_l || 
        const_obstacle->PerceptionSLBoundary().start_l() > passby_max_l) {
        continue;
      }
      double lateral_passby_dis = std::fmin(std::fabs(const_obstacle->PerceptionSLBoundary().end_l() - frenet_point_l),
          std::fabs(const_obstacle->PerceptionSLBoundary().start_l() - frenet_point_l))- vehicle_param.half_wheel;
      AWARN_IF(FLAGS_enable_debug_motion)<< "const_obstacle_id = " << const_obstacle->Id() << "lateral_passby_dis :" << lateral_passby_dis;
      double min_follow_dis = 3;
      if (vehicle_state_.linear_velocity > 12.0/3.6) {
        min_follow_dis = 5;
      } else if (vehicle_state_.linear_velocity < 10.0/3.6) {
        min_follow_dis = 3;
      } else {
        min_follow_dis = 3 + 3.6 * (vehicle_state_.linear_velocity - 10/3.6);
      }
      //速度略高于自车的障碍物有切入迹象时，适当减速
      if (const_obstacle->speed() > vehicle_state_.linear_velocity) {
        if (const_obstacle->PerceptionSLBoundary().start_s() - front_edge_to_center > min_follow_dis) {
          continue;
          //在前方近距离
        } else {
          if (const_obstacle->speed() < 1.5* vehicle_state_.linear_velocity
              && const_obstacle->PerceptionSLBoundary().end_s() > front_edge_to_center
              && lateral_passby_dis < 0.8) {
            passby_limit_v = 0.8*vehicle_state_.linear_velocity;//2s
            if (passby_limit_v <= 1.0) {
              passby_limit_v = 1.0;
            }
            passby_limit_s = const_obstacle->PerceptionSLBoundary().end_s() - front_edge_to_center;
            last_passby_obs_id = const_obstacle->Id();
            passby_flag = 1;
            AINFO_IF(FLAGS_enable_debug_motion)<<"******passby speedlimit : const_obstacle_id = " << const_obstacle->Id()
            << ", passby_limit_v = " << passby_limit_v << ", passby_limit_s = " << passby_limit_s;
            break;
          }
          continue;
        }
      }
      //剩下的障碍物是压线的低于本车车速的障碍物
      passby_limit_s = const_obstacle->PerceptionSLBoundary().start_s() - front_edge_to_center - min_follow_dis;
      passby_limit_s = std::fmax(0.5, passby_limit_s);
      passby_limit_v = std::fmax(std::fmax(0.5, 15.0*lateral_passby_dis), const_obstacle->speed());
      last_passby_obs_id = const_obstacle->Id();
      passby_flag = 1;
      AINFO_IF(FLAGS_enable_debug_motion)<<"******passby speedlimit : const_obstacle_id = " << const_obstacle->Id()
          << ", passby_limit_v = " << passby_limit_v << ", passby_limit_s = " << passby_limit_s;
      break;
    }
  }
  passby_flag_last = passby_flag;
  if (passby_flag == 1) {
    double reference_line_length = reference_line_.Length();
    reference_line_.AddSpeedLimit(passby_limit_s, reference_line_length, passby_limit_v);
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_flag(passby_flag);
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_s(passby_limit_s);
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_v(passby_limit_v);
  } else {
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_flag(passby_flag);
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_s(-1);
    DP_ptr->debug_planning_msg.motionplan_debug.set_passby_limit_v(-1);
  }

}

bool ReferenceLineInfo::GetFirstOverlap(
    const std::vector<hdmap::PathOverlap>& path_overlaps,
    hdmap::PathOverlap* path_overlap) {
  CHECK_NOTNULL(path_overlap);
  const double start_s = sl_boundary_info_.adc_sl_boundary_.end_s();
  constexpr double kMaxOverlapRange = 500.0;
  double overlap_min_s = kMaxOverlapRange;

  for (const auto& overlap : path_overlaps) {
    if (overlap.end_s < start_s) {
      continue;
    }
    if (overlap_min_s > overlap.start_s) {
      *path_overlap = overlap;
      overlap_min_s = overlap.start_s;
    }
  }
  return overlap_min_s < kMaxOverlapRange;
}

void ReferenceLineInfo::InitFirstOverlaps() {
  const auto& map_path = reference_line_.map_path();

  // crosswalk
  hdmap::PathOverlap crosswalk_overlap;
  if (GetFirstOverlap(map_path.crosswalk_overlaps(), &crosswalk_overlap)) {
    first_encounter_overlaps_.push_back({CROSSWALK, crosswalk_overlap});
  }

  // signal
  hdmap::PathOverlap signal_overlap;
  if (GetFirstOverlap(map_path.signal_overlaps(), &signal_overlap)) {
    first_encounter_overlaps_.push_back({SIGNAL, signal_overlap});
  }

  // stop_sign
  hdmap::PathOverlap stop_sign_overlap;
  if (GetFirstOverlap(map_path.stop_sign_overlaps(), &stop_sign_overlap)) {
    first_encounter_overlaps_.push_back({STOP_SIGN, stop_sign_overlap});
  }

  // clear_zone
  hdmap::PathOverlap clear_area_overlap;
  if (GetFirstOverlap(map_path.clear_area_overlaps(), &clear_area_overlap)) {
    first_encounter_overlaps_.push_back({CLEAR_AREA, clear_area_overlap});
  }

  // pnc_junction
  hdmap::PathOverlap pnc_junction_overlap;
  if (GetFirstOverlap(map_path.pnc_junction_overlaps(),
                      &pnc_junction_overlap)) {
    first_encounter_overlaps_.push_back({PNC_JUNCTION, pnc_junction_overlap});
  }
}

bool ReferenceLineInfo::IsInited() const { return is_inited_; }

bool WithinOverlap(const hdmap::PathOverlap& overlap, double s) {
  constexpr double kEpsilon = 1e-2;
  return overlap.start_s - kEpsilon <= s && s <= overlap.end_s + kEpsilon;
}

bool ReferenceLineInfo::CheckChangeLane() const {
  if (!IsChangeLanePath()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Not a change lane path.";
    return false;
  }

  for (const auto* obstacle : path_decision_.obstacles().Items()) {
    const auto& sl_boundary = obstacle->PerceptionSLBoundary();

    constexpr float kLateralShift = 2.5f;
    if (sl_boundary.start_l() < -kLateralShift || 
        sl_boundary.end_l() > kLateralShift) {
      continue;
    }
    constexpr double kChangeLaneIgnoreDistance = 50.0;
    if ((obstacle->IsVirtual() || obstacle->IsStatic()) &&
        sl_boundary.start_s() < sl_boundary_info_.adc_sl_boundary_.end_s() +
                                    kChangeLaneIgnoreDistance &&
        sl_boundary.start_s() > sl_boundary_info_.adc_sl_boundary_.end_s()) {
      return false;
    }

    constexpr float kSafeTime = 3.0f;
    constexpr float kForwardMinSafeDistance = 6.0f;
    constexpr float kBackwardMinSafeDistance = 8.0f;

    const float kForwardSafeDistance = std::max(
        kForwardMinSafeDistance,
        static_cast<float>((adc_planning_point_.v() - obstacle->speed()) *
                           kSafeTime));
    const float kBackwardSafeDistance = std::max(
        kBackwardMinSafeDistance,
        static_cast<float>((obstacle->speed() - adc_planning_point_.v()) *
                           kSafeTime));
    if (sl_boundary.end_s() > sl_boundary_info_.adc_sl_boundary_.start_s() -
                                  kBackwardSafeDistance &&
        sl_boundary.start_s() <
            sl_boundary_info_.adc_sl_boundary_.end_s() + kForwardSafeDistance) {
      return false;
    }
  }
  return true;
}

const std::vector<std::string>& ReferenceLineInfo::TargetLaneId() const {
  return target_lane_ids_;
}

void ReferenceLineInfo::SetTargetLaneId(const std::vector<std::string>& lane_ids) {
  target_lane_ids_ = lane_ids;
}

const SLBoundary& ReferenceLineInfo::AdcSlBoundary() const {
  return sl_boundary_info_.adc_sl_boundary_;
}

const SLBoundary& ReferenceLineInfo::VehicleSlBoundary() const {
  return sl_boundary_info_.vehicle_sl_boundary_;
}

PathDecision* ReferenceLineInfo::path_decision() { return &path_decision_; }

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

ReferenceLine* ReferenceLineInfo::reference_line_ptr() {
  return &reference_line_;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
  discretized_trajectory_ = trajectory;
}

bool ReferenceLineInfo::AddObstacleHelper(
    const std::shared_ptr<Obstacle>& obstacle) {
  return AddObstacle(obstacle.get()) != nullptr;
}

// AddObstacle is thread safe
Obstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "The provided obstacle is empty";
    return nullptr;
  }
  auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  if (!FLAGS_using_cognition_sl_boundary || obstacle->IsVirtual() ) {
    SLBoundary perception_sl;
    if (obstacle->PerceptionPolygon().num_points() > 0 && !FLAGS_is_simulation_mode) {
      if (!reference_line_.GetSLBoundary(obstacle->PerceptionPolygon(), 
                                       &perception_sl)) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"Failed to get sl boundary for obstacle: " << obstacle->Id(); 
        return mutable_obstacle;
      } 
    } else {
      if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
                                       &perception_sl)) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"Failed to get sl boundary for obstacle: " << obstacle->Id(); 
        return mutable_obstacle;
      }
    }
  
    mutable_obstacle->SetPerceptionSlBoundary(perception_sl);
    mutable_obstacle->CheckLaneBlocking(reference_line_);
    
    AINFO_IF(FLAGS_enable_debug_motion)<< "id:"<<mutable_obstacle->Id()<<" perception_sl: l["<<perception_sl.start_l()<<","
       <<perception_sl.end_l()<<", s["<<perception_sl.start_s()<<","
       <<perception_sl.end_s()<<"].";
    if (mutable_obstacle->IsLaneBlocking()) {//@pqg
      AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle [" << obstacle->Id() << "] is lane blocking.";
    } else {
      AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle [" << obstacle->Id() << "] is NOT lane blocking.";
    }
  }

  if (FLAGS_using_cognition_sl_boundary && !obstacle->IsVirtual()) {
    ModifyObstacleSLBoundary(mutable_obstacle);
  }
  
  mutable_obstacle->CheckLaneBlocking(reference_line_);

  if (IsUnrelaventObstacle(mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    AWARN_IF(FLAGS_enable_debug_motion)<< "NOT build reference line st boundary. id:" << obstacle->Id();
  } else {
    if (FLAGS_enable_two_stitching_trajectory) {
      mutable_obstacle->BuildReferenceLineStBoundary(reference_line_, 
         sl_boundary_info_.adc_sl_boundary_.start_s(),speed_plan_start_sl_point_.s(), start_time_);
    } else {
      mutable_obstacle->BuildReferenceLineStBoundary(reference_line_, 
         sl_boundary_info_.adc_sl_boundary_.start_s(),start_sl_point_.s(), start_time_);
    }
  }
  return mutable_obstacle;
}

bool ReferenceLineInfo::ModifyObstacleSLBoundary(Obstacle* const obstacle) {
  if (obstacle->IsVirtual()) {
    return true;
  }
  const double modify_start_s = std::fmax(0.0, car_sl_.s()) + obstacle->PerceptionSLBoundary().start_s();
  const double modify_end_s = std::fmax(0.0, car_sl_.s()) + obstacle->PerceptionSLBoundary().end_s(); 
  const double obstacle_center_s = std::fmax(0.0, modify_start_s + modify_end_s / 2.0);
  const double offset_to_map = reference_line_.GetMapPath().GetOffsetToMapLine(obstacle_center_s);
  SLBoundary modify_obstacle_sl = obstacle->PerceptionSLBoundary(); 
  modify_obstacle_sl.set_start_s(modify_start_s);
  modify_obstacle_sl.set_end_s(modify_end_s);
  modify_obstacle_sl.set_start_l(modify_obstacle_sl.start_l() - offset_to_map);
  modify_obstacle_sl.set_end_l(modify_obstacle_sl.end_l() - offset_to_map);
  obstacle->SetPerceptionSlBoundary(modify_obstacle_sl);
  AWARN_IF(FLAGS_enable_debug_motion)<<"after modify , No["<<obstacle->Id()<<"] obstacle ,s = ("<<obstacle->PerceptionSLBoundary().start_s()
                                   << ", "<<obstacle->PerceptionSLBoundary().end_s()<<"). l = ("
                                   << obstacle->PerceptionSLBoundary().start_l()
                                   << ", "<<obstacle->PerceptionSLBoundary().end_l()<<") .";       
  return true;

}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Failed to add obstacle " << obstacle->Id();
      return false;
    }
  }
  return true;
}

bool ReferenceLineInfo::AddWholeObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto* obstacle : obstacles) {
    auto* mutable_obstacle = path_decision_.AddWholeObstacle(*obstacle);
    if (!mutable_obstacle) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "failed to add obstacle " << obstacle->Id();
      return false;
    }
  }
  return true;
}

bool ReferenceLineInfo::IsUnrelaventObstacle(const Obstacle* obstacle) {
  // if adc is out the reference line, ignore
  if (obstacle->PerceptionSLBoundary().end_s() > reference_line_.Length() + 5.0
      && obstacle->PerceptionSLBoundary().start_s() > reference_line_.Length()+ 5.0) {//@pqg modify
    AINFO_IF(FLAGS_enable_debug_motion)<<"id:"<<obstacle->Id()<<" start_s & end s > lane length";
    return true;
  }
  // if adc is static and behind adc, ignore
  if (is_on_reference_line_ && obstacle->IsStatic() &&
      obstacle->PerceptionSLBoundary().end_s() <
          sl_boundary_info_.adc_sl_boundary_.end_s() &&
      reference_line_.IsOnLane(obstacle->PerceptionSLBoundary())) {//ignore 
    AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()<<"] is static and behind adc, ignore. ";
    return true;
  }

  if (obstacle->IsVirtual() ) {//虚拟停车点的处理 @pqg 
    if (obstacle->PerceptionSLBoundary().end_s() < -10 
      && obstacle->PerceptionSLBoundary().start_s() < -10) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "virtual stop obstacle is behind more than 10m. ignore";
      return true;
    }
    bool enable_logic = false;
    if (obstacle->PerceptionSLBoundary().end_s() > 0 
      && obstacle->PerceptionSLBoundary().start_s() > 0 && enable_logic) {
      double deceleration = -1 *
          vehicle_state_.linear_velocity * vehicle_state_.linear_velocity / (2 * obstacle->PerceptionSLBoundary().end_s());
      if (deceleration < - 5) {
         return true;
      }   
    }
  }
  return false;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return discretized_trajectory_;
}

double ReferenceLineInfo::TrajectoryLength() const {
  if (discretized_trajectory_.empty()) {
    return 0.0;
  }
  return discretized_trajectory_.back().path_point().s();
}

void ReferenceLineInfo::SetStopPoint(const StopPoint& stop_point) {
  planning_target_.mutable_stop_point()->CopyFrom(stop_point);
}

void ReferenceLineInfo::SetCruiseSpeed(double speed) {
  planning_target_.set_cruise_speed(speed);
  reference_line_.set_cruise_speed(speed);
}

bool ReferenceLineInfo::IsStartFrom(
    const ReferenceLineInfo& previous_reference_line_info) const {
  if (reference_line_.reference_points().empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "reference_line_ has no path point !!";
    return false;
  }
  auto start_point = reference_line_.reference_points().front();
  const auto& prev_reference_line =
      previous_reference_line_info.reference_line();
  common::SLPoint sl_point;
  prev_reference_line.XYToSL(start_point, &sl_point);
  return previous_reference_line_info.reference_line_.IsOnLane(sl_point);
}

const PathInfo& ReferenceLineInfo::path_data() const { return path_data_; }

const SpeedInfo& ReferenceLineInfo::speed_data() const { return speed_data_; }

PathInfo* ReferenceLineInfo::mutable_path_data() { return &path_data_; }

SpeedInfo* ReferenceLineInfo::mutable_speed_data() { return &speed_data_; }

const RSSInfo& ReferenceLineInfo::rss_info() const { return rss_info_; }

RSSInfo* ReferenceLineInfo::mutable_rss_info() { return &rss_info_; }

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    const double relative_time, const double start_s,
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  CHECK(ptr_discretized_trajectory != nullptr);
  // use varied resolution to reduce data load but also provide enough data
  // point for control module
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;//0.02
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;//0.1
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;//1
  if (path_data_.discretized_path().size() == 0) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "path data is empty";
    return false;
  }
  //@pqg add 
  double v_end = 0;
  double s_end = 0;
  double a_end = 0;
  double t_end = 0;
  bool has_zero_point = false;
  const double kEpsilon = 1e-6;
  for (double cur_rel_time = 0.0; cur_rel_time <= speed_data_.TotalTime() + kEpsilon;
        cur_rel_time += kDenseTimeResoltuion) {
    common::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }
 
    if (speed_point.s() > path_data_.discretized_path().Length()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"t = "<<cur_rel_time<<", s = "
        <<speed_point.s()<<" > "<<path_data_.discretized_path().Length();
      break;
    }

    common::PathPoint path_point;
    if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Fail to get path data with s " << speed_point.s()
                   << "path total length " << path_data_.discretized_path().Length();
      return false;
    }
    path_point.set_s(path_point.s() + start_s);
    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    if (!has_zero_point && trajectory_point.v() < 0.01) {//debug
      has_zero_point = true;
    }
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
    //record data end @pqg add
    s_end = path_point.s();
    v_end = speed_point.v();
    a_end = speed_point.a();
    t_end = trajectory_point.relative_time();
  }

  if (!has_zero_point) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"there are no zero point!!!";
  }
  if (t_end < relative_time) {
    t_end = relative_time;
  }
  //@pqg add 
  bool enable_extend_path = true;
  AINFO_IF(FLAGS_enable_debug_motion)<<"s_end: "<<s_end 
               <<", v_end = "<<v_end<<", t_end"<<t_end<<", path_data_.discretized_path().Length() "
               <<path_data_.discretized_path().Length() ;
  
  const double kDistanceBuffer = 1.0;
  if (DistanceToStopPoint() < kDistanceBuffer) {
    v_end = 0.0;
  }

  if (s_end - start_s < path_data_.discretized_path().Length() 
      && enable_extend_path) {
    double s_start = s_end - start_s;
    double path_resolution = FLAGS_extend_path_resolution;//0.5;
    double t_sim = 0.05;
    common::SpeedPoint speed_point;
    speed_point.set_s(s_start + path_resolution);
    speed_point.set_v(v_end);
    speed_point.set_t(t_end + t_sim);
    speed_point.set_a(a_end);

    while (speed_point.s() < path_data_.discretized_path().Length()) {
      common::PathPoint path_point;
      if (!path_data_.GetPathPointWithPathS(speed_point.s(), &path_point)) {
        AINFO<<"Fail to get path data with s " << speed_point.s()
                   << "path total length " << path_data_.discretized_path().Length();
        return true;
      }
      path_point.set_s(path_point.s() + start_s);
      common::TrajectoryPoint trajectory_point;
      trajectory_point.mutable_path_point()->CopyFrom(path_point);
      trajectory_point.set_v(speed_point.v());
      trajectory_point.set_a(speed_point.a());
      trajectory_point.set_relative_time(speed_point.t());
      ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
      s_end = speed_point.s();
      speed_point.set_s(speed_point.s() + path_resolution);
      speed_point.set_v(v_end);
      speed_point.set_t(speed_point.t() + t_sim);
    }
    if (s_end < path_data_.discretized_path().Length()) {
      common::PathPoint path_point = path_data_.discretized_path().back();
      path_point.set_s(path_point.s() + start_s);
      common::TrajectoryPoint trajectory_point;
      trajectory_point.mutable_path_point()->CopyFrom(path_point);
      trajectory_point.set_v(speed_point.v());
      trajectory_point.set_a(speed_point.a());
      trajectory_point.set_relative_time(speed_point.t());
      ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion)<< "combine path_data_.property_length() " << path_data_.property_length();
  ptr_discretized_trajectory->SetPropertyLength(path_data_.property_length());
  ptr_discretized_trajectory->SetOffsetPropertyLength(path_data_.offset_property_length());
  
  return true;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

bool ReferenceLineInfo::IsDrivable() const { return is_drivable_; }

bool ReferenceLineInfo::IsChangeLanePath() const {
  return reference_line_.IsChangeLanePath();
}

LanePosition ReferenceLineInfo::Position() const {
  return position_;
}

std::pair<eActionEnum,eDirectionEnum> ReferenceLineInfo::LaterAction() const {
  return lateral_action_;
}

bool ReferenceLineInfo::ReachedDestination() const {
  constexpr double kDestinationDeltaS = 0.05;
  double dis2destination = SDistanceToDestination();
  AWARN_IF(FLAGS_enable_debug_motion)<<"dis2destination "<<dis2destination;
  return dis2destination <= kDestinationDeltaS;
}

double ReferenceLineInfo::SDistanceToDestination() const {
  double res = std::numeric_limits<double>::max();
  const auto* dest_ptr = path_decision_.Find(FLAGS_destination_obstacle_id);
  if (!dest_ptr) {
    return res;
  }
  if (!dest_ptr->LongitudinalDecision().has_stop()) {
    return res;
  }
  if (!reference_line_.IsOnLane(dest_ptr->PerceptionBoundingBox().center())) {
    return res;
  }
  const double stop_s = dest_ptr->PerceptionSLBoundary().start_s() +
                        dest_ptr->LongitudinalDecision().stop().distance_s();
  return stop_s - sl_boundary_info_.adc_sl_boundary_.end_s();
}

double ReferenceLineInfo::DistanceToStopPoint() const {
  double res = std::numeric_limits<double>::max();
  for (const auto& obstacle: path_decision_.obstacles().Items()) {
    if (obstacle->HasLongitudinalDecision()/* && !obstacle->IsVirtual()*/) {
      const auto& decision = obstacle->LongitudinalDecision();
      if ((decision.has_stop()) && 
        obstacle->st_boundary().min_s() < res) {
        res = obstacle->st_boundary().min_s();
      }
    }
  }
  // AINFO_IF(FLAGS_enable_debug_motion)<<"DistanceToStopPoint = "<<res;
  
  return res;
}

void ReferenceLineInfo::SDistanceToFirstStaticObstacle() {
  double res = std::numeric_limits<double>::max();
  std::vector<const Obstacle*> obstacles = path_decision_.obstacles().Items();
  //TODO
  for (auto& object : obstacles) {
    if (object->IsStatic() && !object->IsVirtual() &&
      object->PerceptionSLBoundary().start_s() < res &&
      object->PerceptionSLBoundary().start_s() > EgoInfo::instance()->vehicle_front_edge_to_center()
      && (object->PerceptionSLBoundary().start_l() * object->PerceptionSLBoundary().end_l() <= 0 || 
        std::fmin(fabs(object->PerceptionSLBoundary().start_l()),fabs(object->PerceptionSLBoundary().end_l())) 
        < EgoInfo::instance()->vehicle_param().half_wheel + FLAGS_static_decision_nudge_l_buffer )) {
      res = object->PerceptionSLBoundary().start_s();
    }
  }
  nearest_static_obstacle_s_ = res ;
  if (nearest_static_obstacle_s_ <= reference_line_.Length()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"nearest_static_obstacle_s_ = "<<nearest_static_obstacle_s_;
  }
}

const std::vector<PathInfo>& ReferenceLineInfo::GetCandidatePathData() const {
  return candidate_path_data_;
}

void ReferenceLineInfo::SetCandidatePathData(
    std::vector<PathInfo> candidate_path_data) {
  candidate_path_data_ = std::move(candidate_path_data);
}

const std::vector<PathBoundary>& ReferenceLineInfo::GetCandidatePathBoundaries()
    const {
  return candidate_path_boundaries_;
}

void ReferenceLineInfo::SetCandidatePathBoundaries(
    std::vector<PathBoundary> path_boundaries) {
  candidate_path_boundaries_ = std::move(path_boundaries);
}

bool ReferenceLineInfo::HasUncertainObstacles() const {
  //<暂时>会车时不使用Quasi，由于Quasi中没有增加障碍物权重
  if(!BehaviorParser::instance()->decision_dynamic_obstacle_boxes().empty()){
    AWARN<<"Ignore Quasi for decision_dynamic_obstacle_boxes: "
      <<BehaviorParser::instance()->decision_dynamic_obstacle_boxes().size();
    return false;
  }
  for (const auto* const_obstacle : path_decision_.obstacles().Items()) {
    if (!const_obstacle->Perception().sl_polygons.empty()) {
      return true;
    }
  }
  return false;
}

//[调试]为调试信息增加笛卡尔坐标值
//return: -1 boundary转换错误，-2 dynamic_obstacle_boundary转换错误，-3 soft_boundary转换错误
int ReferenceLineInfo::AssaignCartesianValues(planning_debug_msgs::DebugSLBoundary& input) const{
  int rtvalue = -1;

  auto output = input;
  common::SLPoint sl_point;
  common::math::Vec2d cartesian_point;
  auto start_s = output.start_s();
  auto delta_s = output.delta_s();

  //1.处理boundary
  int bounds_num = 0;
  for (int i = 0; i < output.boundary().size(); i++) {
    auto b = output.boundary()[i];
    sl_point.set_s(start_s + i * delta_s);

    sl_point.set_l(b.right_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_right_boundary_xg(cartesian_point.x());
    b.set_right_boundary_yg(cartesian_point.y());
    b.set_right_boundary_globalangle(cartesian_point.Angle());

    sl_point.set_l(b.left_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_left_boundary_xg(cartesian_point.x());
    b.set_left_boundary_yg(cartesian_point.y());
    b.set_left_boundary_globalangle(cartesian_point.Angle());
  }

  //2.处理box_modified_boundary
  bounds_num = 0;
  for (int i = 0; i < output.box_modified_boundary().size(); i++) {
    auto b = output.boundary()[i];
    sl_point.set_s(start_s + i * delta_s);

    sl_point.set_l(b.right_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_right_boundary_xg(cartesian_point.x());
    b.set_right_boundary_yg(cartesian_point.y());
    b.set_right_boundary_globalangle(cartesian_point.Angle());

    sl_point.set_l(b.left_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_left_boundary_xg(cartesian_point.x());
    b.set_left_boundary_yg(cartesian_point.y());
    b.set_left_boundary_globalangle(cartesian_point.Angle());
  }
  //3.处理dynamic_obstacle_boundary
  bounds_num = 0;
  rtvalue--;
  for (int i = 0; i < output.dynamic_obstacle_boundary().size(); i++) {
    auto b = output.boundary()[i];
    sl_point.set_s(start_s + i * delta_s);

    sl_point.set_l(b.right_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_right_boundary_xg(cartesian_point.x());
    b.set_right_boundary_yg(cartesian_point.y());
    b.set_right_boundary_globalangle(cartesian_point.Angle());

    sl_point.set_l(b.left_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_left_boundary_xg(cartesian_point.x());
    b.set_left_boundary_yg(cartesian_point.y());
    b.set_left_boundary_globalangle(cartesian_point.Angle());
  }

  //3.处理soft_boundary
  bounds_num = 0;
  rtvalue--;
  for (int i = 0; i < output.soft_boundary().size(); i++) {
    auto b = output.boundary()[i];
    sl_point.set_s(start_s + i * delta_s);

    sl_point.set_l(b.right_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_right_boundary_xg(cartesian_point.x());
    b.set_right_boundary_yg(cartesian_point.y());
    b.set_right_boundary_globalangle(cartesian_point.Angle());

    sl_point.set_l(b.left_boundary());
    if(!reference_line().SLToXY(sl_point, &cartesian_point)) {return rtvalue;}
    b.set_left_boundary_xg(cartesian_point.x());
    b.set_left_boundary_yg(cartesian_point.y());
    b.set_left_boundary_globalangle(cartesian_point.Angle());
  }

  input = output;
  rtvalue = 0;
  return rtvalue;
}

}  // namespace planning
}  // namespace acu
