/**
 * @file speed_limit_decider.cpp
 **/

#include <limits>
//#include "ros/ros.h"
#include "pnc_point.pb.h"
#include "decision.pb.h"

#include "common/util/util.h"
#include "speed_limit_decider.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

using acu::common::Status;

SpeedLimitDecider::SpeedLimitDecider(const SLBoundary& adc_sl_boundary,
                                     const StBoundaryConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathInfo& path_data,
                                     const bool& has_overtake_decision)
  : adc_sl_boundary_(adc_sl_boundary),
    st_boundary_config_(config),
    reference_line_(reference_line),
    path_data_(path_data),
    has_overtake_decision_(has_overtake_decision) {
  vehicle_param_ = EgoInfo::instance()->vehicle_param();
}

void SpeedLimitDecider::GetAvgKappa(
  const std::vector<common::PathPoint>& path_points,
  std::vector<double>* kappa) const {
  CHECK_NOTNULL(kappa);
  const int kHalfNumPoints = st_boundary_config_.num_points_to_avg_kappa() / 2;
  CHECK_GT(kHalfNumPoints, 0);
  kappa->clear();
  kappa->resize(path_points.size());
  double sum = 0.0;
  int start = 0;
  int end = 0;
  while (end < static_cast<int>(path_points.size()) &&
         end - start < kHalfNumPoints + 1) {
    sum += path_points[end].kappa();
    ++end;
  }

  int iter = 0;
  while (iter < static_cast<int>(path_points.size())) {
    kappa->at(iter) = sum / (end - start);
    if (start < iter - kHalfNumPoints) {
      sum -= path_points[start].kappa();
      ++start;
    }
    if (end < static_cast<int>(path_points.size())) {
      sum += path_points[end].kappa();
      ++end;
    }
    ++iter;
  }
}

Status SpeedLimitDecider::GetSpeedLimits(
  const IndexedList<std::string, Obstacle>& obstacles,
  SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  std::vector<double> avg_kappa;

  GetAvgKappa(path_data_.discretized_path(), &avg_kappa);
  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();
  uint32_t discretized_path_size = discretized_path.size();
  const double max_centric_acc_limit = 
       has_overtake_decision_? FLAGS_overtake_max_centric_acc_limit : FLAGS_max_centric_acc_limit;

  for (uint32_t i = 0; i < discretized_path_size; ++i) {
    const double path_s = discretized_path.at(i).s();
    const double frenet_point_s = frenet_path.at(i).s();
    const double frenet_point_l = frenet_path.at(i).l();
    if (frenet_point_s > reference_line_.Length()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"path length [" << frenet_point_s
                       << "] is LARGER than reference_line_ length ["
                       << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // (1) speed limit from map and cognition and decision
    double speed_limit_on_reference_line =
      reference_line_.GetSpeedLimitFromS(frenet_point_s);

    const double mission_end_s = EgoInfo::instance()->dis_to_end();
    double mission_end_speed_limit = GetMissionEndSpeedLimit(obstacles, mission_end_s, frenet_point_s, frenet_point_l);
    speed_limit_on_reference_line = std::fmin(mission_end_speed_limit, speed_limit_on_reference_line);

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration) a = v^2/R;
    double point_kappa = FLAGS_use_average_kappa_to_calculate_speed_limit ? 
                std::fabs(avg_kappa[i]): std::fabs(discretized_path.at(i).kappa());
    const double centri_acc_speed_limit =
         std::sqrt(max_centric_acc_limit / 
                std::fmax(point_kappa, st_boundary_config_.minimal_kappa()));                       

    // -- 2.2: limit by centripetal jerk
    double centri_jerk_speed_limit = std::numeric_limits<double>::max();
    if (i + 1 < discretized_path.size()) {
      const double ds =
        discretized_path.at(i + 1).s() - discretized_path.at(i).s();
      DCHECK_GE(ds, 0.0);
      const double kEpsilon = 1e-9;
      const double centri_jerk =
        std::fabs(avg_kappa[i + 1] - avg_kappa[i]) / (ds + kEpsilon);
      centri_jerk_speed_limit = std::fmax(
                                  10.0, st_boundary_config_.centri_jerk_speed_coeff() / centri_jerk);//centri_jerk_speed_coeff :1.0
    }

    // (3) speed limit from nudge obstacles
    double nudge_obstacle_speed_limit = std::numeric_limits<double>::max();
    for (const auto* const_obstacle : obstacles.Items()) {
      if (const_obstacle->IsVirtual()) {
        continue;
      }
      if (!const_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */

      double back_edge_to_center = vehicle_param_.back_over_hang;
      double front_edge_to_center = vehicle_param_.length - back_edge_to_center;

      if (frenet_point_s + front_edge_to_center + FLAGS_obstacle_lon_start_buffer <
          const_obstacle->PerceptionSLBoundary().start_s() ||
          frenet_point_s - back_edge_to_center - FLAGS_obstacle_lon_end_buffer >
          const_obstacle->PerceptionSLBoundary().end_s()) {
        continue;
      }
      constexpr double kRange = 1.0;  // meters
      const auto& nudge = const_obstacle->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path.at(i).l();

      if (const_obstacle->Perception().is_moveble) {
        nudge_obstacle_speed_limit = 
            std::fmin(nudge_obstacle_speed_limit, 
                  GetMovebleObstacleSpeedLimit(
                     const_obstacle, frenet_point_s, frenet_point_l)); 
      } else if (FLAGS_enable_nudge_slowdown) {  
        bool is_close_on_left =
          (nudge.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - (vehicle_param_.half_wheel) - kRange <
           const_obstacle->PerceptionSLBoundary().end_l());

        // obstacle is on the left of ego vehicle (at path point i)
        bool is_close_on_right =
          (nudge.type() == ObjectNudge::RIGHT_NUDGE) &&
          (const_obstacle->PerceptionSLBoundary().start_l() - kRange <
           frenet_point_l + (vehicle_param_.half_wheel) );
        double lateral_dis = std::fmin(std::fabs(frenet_point_l - vehicle_param_.half_wheel-
                  const_obstacle->PerceptionSLBoundary().end_l()), std::fabs(const_obstacle->PerceptionSLBoundary().start_l()
                  - frenet_point_l - vehicle_param_.half_wheel));
        if (is_close_on_left || is_close_on_right) {
          double nudge_speed_ratio = 1.0;
          if (const_obstacle->IsStatic()) {
             AWARN_IF(FLAGS_enable_debug_motion)<<"*****lateral_dis:;"<< lateral_dis;
            nudge_speed_ratio = lateral_dis / vehicle_param_.half_wheel;
            nudge_speed_ratio = std::fmax(0.1, nudge_speed_ratio);
            nudge_speed_ratio = std::fmin(1, nudge_speed_ratio);
            // nudge_speed_ratio =
            //   st_boundary_config_.static_obs_nudge_speed_ratio();//0.6
          } else {
            nudge_speed_ratio =
              st_boundary_config_.dynamic_obs_nudge_speed_ratio();//0.8
          }
          nudge_obstacle_speed_limit =
            nudge_speed_ratio * speed_limit_on_reference_line;//参考线速度的成比例下降
          break;
        }
      }
    }

    double curr_speed_limit = 0.0;
    curr_speed_limit =
      std::fmax(st_boundary_config_.lowest_speed(),//lowest_speed :2.5
    common::util::MinElement(std::vector<double> {         //取最小值
      speed_limit_on_reference_line, centri_acc_speed_limit,
      centri_jerk_speed_limit, nudge_obstacle_speed_limit
    }));

    if (FLAGS_enable_pedestrian_bicycle_slowdown) {
      for (const auto* const_obstacle : obstacles.Items()) {
        if (const_obstacle->Perception().type == 2 || const_obstacle->Perception().type == 3 //bicycle is TODO
          ) {
          double car_speed = EgoInfo::instance()->vehicle_state_new().linear_velocity;
          if (frenet_point_s >= 6.0 *car_speed ) continue;
          double back_edge_to_center = vehicle_param_.back_over_hang;
          double front_edge_to_center = vehicle_param_.length - back_edge_to_center;
          const double bicycle_pedestrian_slowdown_base_speed = curr_speed_limit;
          const double slow_range = 10.0;
          // const double speed_lowest_1 = 0.0;
          const double speed_lowest_2 = 2.0;//7.2km/h
          if (frenet_point_s + front_edge_to_center + slow_range <
              const_obstacle->PerceptionSLBoundary().start_s() ||
              frenet_point_s - back_edge_to_center -1.0 >
              const_obstacle->PerceptionSLBoundary().end_s()) {
            continue;
          }
          // double frenet_point_min_s = const_obstacle->PerceptionSLBoundary().start_s() - frenet_point_s - front_edge_to_center;
          // if (frenet_point_min_s > -vehicle_param_.length) {
          //   if (frenet_point_min_s < 0) frenet_point_min_s = 0.0;//保证车身范围内限速一样
          double ratio = 1.0;
          if (frenet_point_s > 4 * car_speed ) {
            ratio = 0.4;
          } else if (frenet_point_s > 2 * car_speed ) {
            ratio = 0.6;
          }
          double bicycle_pedestrian_s_speed_limit = std::fmax(ratio * bicycle_pedestrian_slowdown_base_speed, speed_lowest_2);
          double min_l = std::min(fabs(const_obstacle->PerceptionSLBoundary().start_l()),
                         fabs(const_obstacle->PerceptionSLBoundary().end_l()));
          bool is_conflict_pedestrian = const_obstacle->Perception().type == 2 && ((const_obstacle->conflict_type() == 2 
                || const_obstacle->conflict_type() == 3 || const_obstacle->conflict_type() == 5)
                || min_l < 3.0);
          bool is_conflict_bicycle = const_obstacle->Perception().type == 2 && const_obstacle->conflict_type() == 5;

          double bicycle_pedestrian_speed_limit = bicycle_pedestrian_slowdown_base_speed;
          if (is_conflict_pedestrian || is_conflict_bicycle) {
            bicycle_pedestrian_speed_limit = std::min(bicycle_pedestrian_s_speed_limit , bicycle_pedestrian_speed_limit);
          }
          // AWARN_IF(FLAGS_enable_debug_motion)<<"bicycle_pedestrian_speed_limit :"<< bicycle_pedestrian_speed_limit;
          curr_speed_limit = std::fmin(curr_speed_limit, bicycle_pedestrian_speed_limit);
          // AWARN_IF(FLAGS_enable_debug_motion)<<"No["<<const_obstacle->PerceptionId()<<"] pedestrian "
                      //  <<",s = "<<frenet_point_s<<", curr_speed_limit = " <<curr_speed_limit;
                //<<", slowdown_ratio = "<<slowdown_ratio;
          break;               
        }
      }
    }

    // AINFO_IF(FLAGS_enable_debug_speedplan)<<"s = "<<path_s<<", speed_limit_on_reference_line = "
                                        // <<speed_limit_on_reference_line
                                        // <<", kappa = "<<point_kappa<<", centri_acc_speed_limit = "<<centri_acc_speed_limit
                                        // <<", curr_speed_limit = "<<curr_speed_limit;
    
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }
  return Status::OK();
}


double SpeedLimitDecider::GetUncertainObstacleNudgeSpeedLimit(
    const Obstacle* obstacle_ptr, const double car_s, const double car_l) const {
  double speed_limit = 100.0;
  const double acceleration = -1.0;
  const double car_speed = EgoInfo::instance()->vehicle_state_new().linear_velocity;
  const double t_estimate = car_s / car_speed;
  const Vec2d car_left(car_s, car_l + EgoInfo::instance()->vehicle_width() / 2.0);
  const Vec2d car_right(car_s, car_l - EgoInfo::instance()->vehicle_width() / 2.0);
  int collision_index = -1;
  for (const auto& sl_polygon : obstacle_ptr->Perception().sl_polygons) {
    if (sl_polygon.IsPointIn(car_left) || sl_polygon.IsPointIn(car_right)
      || sl_polygon.IsPointOnBoundary(car_left) || sl_polygon.IsPointOnBoundary(car_right)) {
      collision_index++;
    }
  }
  switch (collision_index) {
    case 0:
      {
        double target_speed = std::fmax(2.0, car_speed + acceleration * t_estimate);
        speed_limit = target_speed;
        // AINFO_IF(FLAGS_enable_debug_motion)<<"collision with first circle set speed_limit = "<<speed_limit;
      }                
      break;
    case 1:
      {
        double target_speed = std::fmax(3.0, car_speed + acceleration * 0.8 * t_estimate);
        speed_limit = target_speed;
        // AINFO_IF(FLAGS_enable_debug_motion)<<"collision with second circle set speed_limit = "<<speed_limit;
        break;
      }
      
    case 2:
      {
        double target_speed = std::fmax(4.0, car_speed + acceleration * 0.6 * t_estimate);
        speed_limit = target_speed;
        // AINFO_IF(FLAGS_enable_debug_motion)<<"collision with third circle set speed_limit = "<<speed_limit;
        break;
      }
      
    // default:
    //   AWARN_IF(FLAGS_enable_debug_motion) << "no collision with No[" 
    //      << obstacle_ptr->Id()<<"], car s = "<<car_s <<" l = ("<<car_left.y()<<", "<<car_right.y()<<").";
      // int i = 0;    
      // for (const auto& sl_polygon : obstacle_ptr->Perception().sl_polygons) {
      //   // std::cout<<"i = "<<i<<"====================================="<<std::endl;
      //   for (const auto& sl : sl_polygon.points()) {
      //     // AINFO_IF(FLAGS_enable_debug_motion)<<"s = "<<sl.x()<<", l = "<<sl.y();
      //   }
      //   i++;
      // }
  }
  return speed_limit;
}

double SpeedLimitDecider::GetMissionEndSpeedLimit(const IndexedList<std::string, Obstacle>& obstacles,
        const double& mission_end_s, const double point_s,  const double point_l) const {
  double speed_limit = 100.0;
  double mission_end_ratio = 1.0;
  const double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;

  if (mission_end_s < 200.0) {
    bool is_obstacle_near_mission_end = false;
    int num_obstacle_near_mission_end = 0;
    for (const auto* obstacle : obstacles.Items()) {
      // Obstacle should be non-virtual.
      if (obstacle->IsVirtual()) {
        continue;
      }
      // Obstacle should not have ignore decision.
      if (obstacle->HasLongitudinalDecision() && obstacle->HasLateralDecision() &&
          obstacle->IsIgnore()) {
        continue;
      }
      const auto obstacle_sl = obstacle->PerceptionSLBoundary();
      bool is_on_right = false;
      //1.先判断是否在自车右侧
      if (obstacle_sl.start_l() < point_l &&  obstacle_sl.end_l() < point_l) { // 障碍物任何一个边都在自车的右边
        is_on_right = true;
      } else {//中轴线在自车右边，且超过半个车身
        if ((obstacle_sl.start_l() + obstacle_sl.end_l()) / 2.0 < point_l + adc_half_width
          && fabs(obstacle_sl.start_l() - point_l) > adc_half_width) {
          is_on_right = true;
        } 
      }
      //在右侧同时距离终点100m内的障碍物
      if (is_on_right && mission_end_s - obstacle_sl.start_s() < 100.0) {
        num_obstacle_near_mission_end++;
      }
    }
    if (num_obstacle_near_mission_end >= 4) {
      mission_end_ratio = 0.4;
    } else if (num_obstacle_near_mission_end >= 2) {
      mission_end_ratio = 0.6;
    } else {
      mission_end_ratio = 0.8;
    }
    if (num_obstacle_near_mission_end >= 2) {
      if (mission_end_s - point_s < 80.0) {
        speed_limit = mission_end_ratio * reference_line_.GetSpeedLimitFromS(point_s);
      }
    } else {
      if (mission_end_s - point_s < 60.0) {
        speed_limit = mission_end_ratio * reference_line_.GetSpeedLimitFromS(point_s);
      }
    } 
  }
  return speed_limit;
}


double SpeedLimitDecider::GetMovebleObstacleSpeedLimit(
    const Obstacle* obstacle_ptr, const double car_s, const double car_l) const {
  double speed_limit = 100.0;
  const double acceleration = -1.0;
  const double car_speed = EgoInfo::instance()->vehicle_state_new().linear_velocity;
  const double t_estimate = car_s / car_speed;
  double lateral_dis = 0;
  if (obstacle_ptr->PerceptionSLBoundary().start_l() >= (car_l + EgoInfo::instance()->vehicle_width() / 2.0)) {
    // 障碍物在左侧
    lateral_dis = (obstacle_ptr->PerceptionSLBoundary().start_l() - (car_l + EgoInfo::instance()->vehicle_width() / 2.0));
  } else if (obstacle_ptr->PerceptionSLBoundary().end_l() <= (car_l - EgoInfo::instance()->vehicle_width() / 2.0)) {
    //障碍物在右侧
    lateral_dis = (obstacle_ptr->PerceptionSLBoundary().end_l() - (car_l - EgoInfo::instance()->vehicle_width() / 2.0));
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"lateral_dis = "<<lateral_dis;

  double target_speed = std::fmax( 15*std::fabs(lateral_dis) , car_speed + acceleration * 0.6 * t_estimate);
  speed_limit = target_speed;
  AINFO_IF(FLAGS_enable_debug_motion)<<"moveble obstacle set speed_limit = "<<speed_limit
              << ", moveble obstacle id: " << obstacle_ptr->Perception().id;
  return speed_limit;
}
double SpeedLimitDecider::GetCentricAccLimit(const double kappa) const {
  // this function uses a linear model with upper and lower bound to determine
  // centric acceleration limit

  // suppose acc = k1 * v + k2
  // consider acc = v ^ 2 * kappa
  // we determine acc by the two functions above, with uppper and lower speed
  // bounds
  const double v_high = st_boundary_config_.high_speed_threshold();
  const double v_low = st_boundary_config_.low_speed_threshold();//7.5

  const double h_v_acc =
    st_boundary_config_.high_speed_centric_acceleration_limit();//0.8
  const double l_v_acc =
    st_boundary_config_.low_speed_centric_acceleration_limit();//1.2

  if (std::fabs(v_high - v_low) < 1.0) {
    return h_v_acc;
  }
  const double kMinKappaEpsilon = 1e-9;
  if (kappa < kMinKappaEpsilon) {//直道向心加速度为h_v_acc
    return h_v_acc;
  }

  const double k1 = (h_v_acc - l_v_acc) / (v_high - v_low);
  const double k2 = h_v_acc - v_high * k1;

  const double v = (k1 + std::sqrt(k1 * k1 + 4.0 * kappa * k2)) / (2.0 * kappa);

  if (v > v_high) {
    return h_v_acc;
  } else if (v < v_low) {
    return l_v_acc;
  } else {
    return v * k1 + k2;
  }
}

}  // namespace planning
}  // namespace acu
