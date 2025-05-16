/**
 * @file ego_info.cpp
 **/

#include "ego_info.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"

namespace acu {
namespace planning {

using common::math::Vec2d;
using common::math::Box2d;

EgoInfo::EgoInfo(): kEstimateDecel_(2.0){
}

void EgoInfo::SetVehicleParams(const CarModel& vehicle_params) {
  vehicle_param_ = vehicle_params;
  back_edge_to_center_ = vehicle_param_.back_over_hang;
  front_edge_to_center_ = vehicle_param_.length - back_edge_to_center_;
  left_edge_to_center_ = vehicle_param_.half_wheel;
  right_edge_to_center_ = left_edge_to_center_;
}

bool EgoInfo::Update(const StructVehicleInfo& veh_info, const StructLocalizationInfo& newest_loc) {
  vehicle_state_.timestamp = veh_info.localization.time_stamp;
  vehicle_state_.x = veh_info.localization.xg;
  vehicle_state_.y = veh_info.localization.yg;
  vehicle_state_.z = 0.0;
  vehicle_state_.heading = veh_info.localization.global_angle * M_PI/180.0;
  vehicle_state_.yaw = veh_info.localization.global_angle * M_PI/180.0;
  vehicle_state_.angular_velocity = veh_info.localization.angular_velocity;
  vehicle_state_.linear_acceleration = veh_info.chassis.vehicle_accel;
  vehicle_state_.gear = veh_info.chassis.drive_gear;
  vehicle_state_.brake_state = veh_info.chassis.brake_state;
  vehicle_state_.dr_x = veh_info.localization.dr_x;
  vehicle_state_.dr_y = veh_info.localization.dr_y;
  vehicle_state_.dr_yaw = veh_info.localization.dr_angle * M_PI/180.0;
  on_accpedal_ = veh_info.chassis.on_accpedal;

  vehicle_state_.linear_velocity = 
      FLAGS_using_chassis_velocity ? veh_info.chassis.chassis_velocity : veh_info.localization.loc_velocity;
  AINFO_IF(FLAGS_enable_debug_motion) <<fixed << setprecision(4)
                <<"localization xg :"<<vehicle_state_.x
                <<", yg: "<<vehicle_state_.y
                <<", global_angle: "<<veh_info.localization.global_angle
                <<", dr_xg :"<<vehicle_state_.dr_x
                <<", dr_yg: "<<vehicle_state_.dr_y
                <<", dr_global_angle: "<<veh_info.localization.dr_angle
                <<", a: "<<veh_info.chassis.vehicle_accel
                <<", current speed :"<< vehicle_state_.linear_velocity<< " .";    
  
  constexpr double kEpsilon = 1e-6;
  if (std::fabs(vehicle_state_.linear_velocity) < kEpsilon) {
    vehicle_state_.kappa = 0.0;
  } else {
    vehicle_state_.kappa = vehicle_state_.angular_velocity /
                           vehicle_state_.linear_velocity;
  }
  //或者直接从认知取 待定
  vehicle_state_.kappa = veh_info.chassis.kappa_curvature;
  vehicle_state_.driving_mode = veh_info.chassis.drive_mode;//驾驶模式，自动/手动

  vehicle_state_new_.timestamp = newest_loc.time_stamp;
  vehicle_state_new_.x = newest_loc.xg;
  vehicle_state_new_.y = newest_loc.yg;
  vehicle_state_new_.z = 0.0;
  vehicle_state_new_.heading = newest_loc.global_angle * M_PI/180.0;
  vehicle_state_new_.yaw = newest_loc.global_angle * M_PI/180.0;
  vehicle_state_new_.linear_velocity= newest_loc.loc_velocity;
  vehicle_state_new_.angular_velocity= newest_loc.angular_velocity;
  vehicle_state_new_.linear_acceleration = veh_info.chassis.vehicle_accel;
  vehicle_state_new_.gear = veh_info.chassis.drive_gear;
  vehicle_state_new_.driving_mode = veh_info.chassis.drive_mode;
  vehicle_state_new_.brake_state = veh_info.chassis.brake_state;
  vehicle_state_new_.dr_x = newest_loc.dr_x;
  vehicle_state_new_.dr_y = newest_loc.dr_y;
  vehicle_state_new_.dr_yaw = newest_loc.dr_angle * M_PI/180.0;
  if (std::fabs(vehicle_state_.linear_velocity) < kEpsilon) {
    vehicle_state_new_.kappa = 0.0;
  } else {
    vehicle_state_new_.kappa = vehicle_state_.angular_velocity /
                           vehicle_state_.linear_velocity;
  }

  double kInValidThreshold = 10000;
  if (std::isnan(vehicle_state_.x) || std::isnan(vehicle_state_.y) || 
      vehicle_state_.x < kInValidThreshold || vehicle_state_.y < kInValidThreshold) {
    AWARN_IF(FLAGS_enable_debug_motion) << "localization point is invalid. xg "<<vehicle_state_.x<<" yg "<<vehicle_state_.y;
    return false;
  }

  return true;
}

bool EgoInfo::Update(const common::TrajectoryPoint& start_point) {
  set_start_point(start_point);
  CalculateEgoBox();
  return true;
}

void EgoInfo::CalculateEgoBox() {
  Vec2d vec_to_center(
      (front_edge_to_center_ - back_edge_to_center_) / 2.0,
      (left_edge_to_center_ - right_edge_to_center_) / 2.0);
  Vec2d position(vehicle_state_.x, vehicle_state_.y);
  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading));

  const double buffer = 0.1;  // in meters
  ego_box_ = Box2d(center, vehicle_state_.heading, vehicle_param_.length + buffer,
                   vehicle_param_.car_width + buffer);
}

void EgoInfo::Clear() {
  start_point_.Clear();
  vehicle_state_.Clear();
  front_clear_distance_ = FLAGS_default_front_clear_distance;//300m
}

void EgoInfo::CalculateFrontObstacleClearDistance(
    const std::vector<const Obstacle*>& obstacles) {
  Vec2d position(vehicle_state_.x, vehicle_state_.y);
  Vec2d vec_to_center(
    (front_edge_to_center_ - back_edge_to_center_) / 2.0,
    (left_edge_to_center_ - right_edge_to_center_) / 2.0);

  Vec2d center(position + vec_to_center.rotate(vehicle_state_.heading));

  Vec2d unit_vec_heading = Vec2d::CreateUnitVec2d(vehicle_state_.heading);

  // Due to the error of ego heading, only short range distance is meaningful
  constexpr double kDistanceThreshold = 50.0;
  constexpr double buffer = 0.1;  // in meters
  const double impact_region_length =
      vehicle_param_.length + buffer + kDistanceThreshold;
  Box2d ego_front_region(center + unit_vec_heading * kDistanceThreshold / 2.0,
                         vehicle_state_.heading, impact_region_length,
                         vehicle_param_.car_width + buffer);
  front_clear_distance_ = FLAGS_default_front_clear_distance;//@pqg add 
  for (const auto& obstacle : obstacles) {
    if (!ego_front_region.HasOverlap(obstacle->PerceptionBoundingBox())) {
      continue;
    }

    double dist = ego_box_.center().DistanceTo(
                      obstacle->PerceptionBoundingBox().center()) -
                  ego_box_.diagonal() / 2.0;

    if(FLAGS_enable_limit_decel_when_fallback && !obstacle->IsVirtual()) {
      dist = std::max(dist, vehicle_state_new_.linear_velocity * vehicle_state_new_.linear_velocity / (2.0 * kEstimateDecel_));
    }
    if (front_clear_distance_ < 0.0 || dist < front_clear_distance_) {
      front_clear_distance_ = dist;
    }
  }
}

void EgoInfo::CalculateFrontObstacleClearDistance(
    ReferenceLineInfo* reference_line_info) {
  front_clear_distance_ = FLAGS_default_front_clear_distance;//@pqg add 
  const double buffer = 1.0;
  auto obstacles = reference_line_info->path_decision()->obstacles().Items();

  std::multiset<double> obstacle_dists;//默认从小到大排序
  obstacle_dists.insert(front_clear_distance_);

  for (const auto& obstacle : obstacles) {
    if (obstacle->PerceptionSLBoundary().end_s() <= -10 
      && obstacle->IsVirtual() && obstacle->Id().substr(0,1) == "r") { //ignore obj behind @pqg add 
      continue;
    }
    if (obstacle->PerceptionSLBoundary().end_s() <= -1 
      && obstacle->IsVirtual() && obstacle->Id().substr(0,1) == "s") { //ignore obj behind @pqg add 
      continue;
    }
    if (obstacle->PerceptionSLBoundary().end_s() <= front_edge_to_center_
       && !obstacle->IsVirtual()) { //ignore obj behind @pqg add 
      continue;
    }
    bool is_on_path = false;
    const auto& decision = obstacle->LongitudinalDecision();
    if (obstacle->IsVirtual() 
        || decision.has_stop()) {
      is_on_path = true;
    } 
    if (!is_on_path) {//is new path ,need calculate new_sl_boundary
      const auto &frenet_path = reference_line_info->path_data().frenet_frame_path();
      if (frenet_path.empty() ) {
        AERROR<<"path is empty!!!!";
        return;
      }
      const auto frenet_point = frenet_path.GetNearestPoint(obstacle->PerceptionSLBoundary());//从障碍物的boundary计算找到与路径的最小距离的路点
      const double curr_l = frenet_point.l();
      SLBoundary new_sl_boundary;
      new_sl_boundary.set_start_s(obstacle->PerceptionSLBoundary().start_s());
      new_sl_boundary.set_end_s(obstacle->PerceptionSLBoundary().end_s());
      new_sl_boundary.set_start_l(obstacle->PerceptionSLBoundary().start_l() - curr_l);
      new_sl_boundary.set_end_l(obstacle->PerceptionSLBoundary().end_l() - curr_l);
      if (new_sl_boundary.start_l() * new_sl_boundary.end_l() <= 0) {
        is_on_path = true;
      } else {
        double boudary_l_min = 
             std::fmin(fabs(new_sl_boundary.start_l()),fabs(new_sl_boundary.end_l()));
        if (boudary_l_min <= vehicle_param_.half_wheel+ FLAGS_static_decision_nudge_l_buffer) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"based on sl_boundary (boudary_l_min = "<<boudary_l_min
                <<"), this obstacle must be made into consideration. ";
          is_on_path = true;
        } else if (boudary_l_min <= vehicle_param_.half_wheel+ FLAGS_static_decision_nudge_l_buffer + 0.2
           && obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::FOLLOW) {
          is_on_path = true;
        }
      }
    } 
    double dist = obstacle->PerceptionSLBoundary().start_s();
    if (!obstacle->IsStatic()) {
      if (obstacle->speed() > vehicle_state_.linear_velocity) {
        double safe_dist = 0.5 * vehicle_state_.linear_velocity + 5.0;
        if (dist < safe_dist + front_edge_to_center_) {
          dist = dist + obstacle->speed()*obstacle->speed();//assume it will decellerate with -0.5m/s^2;
        } else {
          continue;
        }
      } else {
        dist = dist + obstacle->speed()*obstacle->speed() / 2; //assume it will decellerate with -1m/s^2;
      }
    }
    
    if (is_on_path) {
      if(FLAGS_enable_limit_decel_when_fallback && !obstacle->IsVirtual()) {
        dist = std::max(dist, vehicle_state_new_.linear_velocity * vehicle_state_new_.linear_velocity / (2.0 * kEstimateDecel_));
      }
      obstacle_dists.insert(dist);
    }
  }

  front_clear_distance_ = *(obstacle_dists.begin());
}

bool EgoInfo::is_in_solid_line_area() const {
  vector<pair<double, int> > left_bd_types = BehaviorParser::instance()->target_reference_line().mapinfo.left_bd_types;
  vector<pair<double, int> > right_bd_types = BehaviorParser::instance()->target_reference_line().mapinfo.right_bd_types;

  if (!left_bd_types.empty() && !right_bd_types.empty()) {
    if (left_bd_types.front().second > 2 && right_bd_types.front().second > 2) {
      return true;
    }
  } 
  return false;
}

bool EgoInfo::is_in_juction_area() const {
  double distance_to_junction = BehaviorParser::instance()->target_reference_line().mapinfo.distance_to_junctions.size() ? 
    BehaviorParser::instance()->target_reference_line().mapinfo.distance_to_junctions.front().first : std::numeric_limits<double>::max();
  if (distance_to_junction <= 50) return true;
  return false;
}

}  // namespace planning
}  // namespace acu
