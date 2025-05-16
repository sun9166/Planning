/**
 * @file speed_profile_generator.cpp
 **/

#include <algorithm>
#include "../common/ego_info.h"
#include "../common/planning_gflags.h"
#include "speed_profile_generator.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/algorithm/curve/standing_still_trajectory1d.h"
#include "src/algorithm/curve/constant_deceleration_trajectory1d.h"

namespace acu {
namespace planning {
namespace speed_profile_generator {     

using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::math::Vec2d; 

std::vector<SpeedPoint> GenerateInitSpeedProfile(
    const TrajectoryPoint& planning_init_point,
    const ReferenceLineInfo* reference_line_info) {
  std::vector<SpeedPoint> speed_profile;
  const auto* last_frame = FrameHistory::instance()->Latest();//
  if (!last_frame) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last frame is empty";
    return speed_profile;
  }
  const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last reference line info is empty";
    return speed_profile;
  }
  if (!reference_line_info->IsStartFrom(*last_reference_line_info)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Current reference line is not started previous drived line";
    return speed_profile;
  }
  const auto& last_speed_data = last_reference_line_info->speed_data();

  if (!last_speed_data.empty()) {//上一帧的速度file不为空
    const auto& last_init_point = last_frame->PlanningStartPoint().path_point();
    Vec2d last_xy_point(last_init_point.x(), last_init_point.y());
    SLPoint last_sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(last_xy_point,
                                                           &last_sl_point)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Fail to transfer xy to sl when init speed profile";
    }

    Vec2d xy_point(planning_init_point.path_point().x(),
                   planning_init_point.path_point().y());
    SLPoint sl_point;
    if (!last_reference_line_info->reference_line().XYToSL(xy_point,
                                                           &sl_point)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Fail to transfer xy to sl when init speed profile";
    }

    double s_diff = sl_point.s() - last_sl_point.s();
    double start_time = 0.0;
    double start_s = 0.0;
    bool is_updated_start = false;
    for (const auto& speed_point : last_speed_data) {
      if (speed_point.s() < s_diff) {
        continue;
      }
      if (!is_updated_start) {
        start_time = speed_point.t();
        start_s = speed_point.s();
        is_updated_start = true;
      }
      SpeedPoint refined_speed_point;
      refined_speed_point.set_s(speed_point.s() - start_s);
      refined_speed_point.set_t(speed_point.t() - start_time);
      refined_speed_point.set_v(speed_point.v());
      refined_speed_point.set_a(speed_point.a());
      refined_speed_point.set_da(speed_point.da());
      speed_profile.push_back(std::move(refined_speed_point));
    }
    if (speed_profile.empty()) {
      
      double s = 0.0;
      double t = 0.0;
      double v = last_speed_data.back().v();
      AWARN_IF(FLAGS_enable_debug_motion)<<"planning_init_point s > last_speed_data max_s ! set "<<"v = "<<v ;
      while (t < FLAGS_trajectory_time_length) {//默认值为8s
        SpeedPoint speed_point;
        speed_point.set_s(s);
        speed_point.set_t(t);
        speed_point.set_v(v);
        speed_profile.push_back(std::move(speed_point));
        t += FLAGS_trajectory_time_min_interval;
        s += v * FLAGS_trajectory_time_min_interval;
      }
    }
  }
  return speed_profile;
}

// a dummy simple hot start
std::vector<SpeedPoint> GenerateSpeedHotStart(
    const TrajectoryPoint& planning_init_point,const double cruise_speed) {
  std::vector<SpeedPoint> hot_start_speed_profile;
  double s = 0.0;
  double t = 0.0;
  double v = common::math::Clamp(planning_init_point.v(), 5.0,
                                 cruise_speed);
  while (t < FLAGS_trajectory_time_length) {//默认值为8s
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    hot_start_speed_profile.push_back(std::move(speed_point));
    t += FLAGS_trajectory_time_min_interval;
    s += v * FLAGS_trajectory_time_min_interval;
  }
  return hot_start_speed_profile;
}

SpeedInfo GenerateFallbackSpeedProfile(ReferenceLineInfo* reference_line_info) {
  const double init_v = reference_line_info->speed_init_point().v();
  const double init_a = reference_line_info->speed_init_point().a();
  const double init_s = reference_line_info->speedplan_init_sl().s(); 
  double stop_distance = CalculateStopDistance(reference_line_info,init_s);
  AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance = "<<stop_distance;
  stop_distance = std::fmin(reference_line_info->path_data().discretized_path().Length(), stop_distance);
  AWARN_IF(FLAGS_enable_debug_motion) <<"init_v:" << init_v << " ,init_a:" << init_a 
     << ", stop distance:"<<stop_distance
     << ", pathinfo length = "<<reference_line_info->path_data().discretized_path().Length();
  double is_must_decel = BehaviorParser::instance()->has_yield_or_overtake_behavior() 
                        || BehaviorParser::instance()->has_follow_behavior()
                        || BehaviorParser::instance()->IsObstacleDecisionFailed() ? true : false;   
  return GenerateStopSpeedProfile(stop_distance,init_v,init_a, is_must_decel);  

}

SpeedInfo GenerateStopSpeedProfile(
          const double stop_point_s,const double init_speed,
          const double init_acc,const bool& is_must_decel) {
  double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;
  SpeedInfo speed_data;
  AINFO_IF(FLAGS_enable_debug_motion)<<"fallback stop_distance: "<<stop_point_s;         
  if (stop_point_s <= 0.1 || init_speed <= 0.05
      || EgoInfo::instance()->vehicle_state_new().linear_velocity < 0.05) {
    StandingStillTrajectory1d stop_traj(0.0, max_t);
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = stop_traj.Evaluate(0, t);
      double v = stop_traj.Evaluate(1, t);
      double a = stop_traj.Evaluate(2, t);
      double da = stop_traj.Evaluate(3, t);
      speed_data.AppendSpeedPoint(s, t, v, a, da);
    }
    return speed_data;
  } 

  double acc = -1 * init_speed * init_speed / (2 * stop_point_s);
  if (is_must_decel) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"must make decelleration speedplan!";
    acc = std::fmin(acc, -1.0);
  }
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"fallback acc: "<<acc;  
  if (fabs(acc - 0.0) < 0 || acc < -5) {
    StandingStillTrajectory1d stop_traj(0.0, max_t);
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = stop_traj.Evaluate(0, t);
      double v = stop_traj.Evaluate(1, t);
      double a = stop_traj.Evaluate(2, t);
      double da = stop_traj.Evaluate(3, t);
      speed_data.AppendSpeedPoint(s, t, v, a, da);
    }
    return speed_data;
  }    
  double t_stop = init_speed / fabs(acc); 
  double pre_s = 0.0;
  double pre_v = init_speed;
  if (t_stop <= unit_t) {
    StandingStillTrajectory1d stop_traj(0.0, max_t);
    for (double t = 0.0; t < max_t; t += unit_t) {
      double s = stop_traj.Evaluate(0, t);
      double v = stop_traj.Evaluate(1, t);
      double a = stop_traj.Evaluate(2, t);
      double da = stop_traj.Evaluate(3, t);
      speed_data.AppendSpeedPoint(s, t, v, a, da);
    }
    return speed_data;
  }
  max_t = std::fmax(max_t,t_stop);
  max_t = std::fmin(max_t,8.0);

  ConstantDecelerationTrajectory1d const_acc_traj(0.0, init_speed, acc);
  for (double t = 0.0; t <= max_t; t += unit_t) {
    double s = const_acc_traj.Evaluate(0, t);
    double v = const_acc_traj.Evaluate(1, t);
    double a = const_acc_traj.Evaluate(2, t);
    double da = const_acc_traj.Evaluate(3, t);
    speed_data.AppendSpeedPoint(s, t, v, a, da);
  }

  return speed_data;
}

SpeedInfo GenerateStopProfile(const double init_speed,
                                                     const double init_acc) {
  AWARN_IF(FLAGS_enable_debug_motion) << "Using fallback stopping profile: Slowing down the car!";
  SpeedInfo speed_data;

  const double max_t = FLAGS_fallback_total_time;
  const double unit_t = FLAGS_fallback_time_unit;

  double pre_s = 0.0;
  double pre_v = init_speed;
  double acc = FLAGS_slowdown_profile_deceleration;//默认-1

  for (double t = 0.0; t < max_t; t += unit_t) {
    double s = 0.0;
    double v = 0.0;
    s = std::fmax(pre_s,
                  pre_s + 0.5 * (pre_v + (pre_v + unit_t * acc)) * unit_t);
    v = std::fmax(0.0, pre_v + unit_t * acc);
    speed_data.AppendSpeedPoint(s, t, v, acc, 0.0);
    pre_s = s;
    pre_v = v;
  }
  return speed_data;
}

bool IsValidProfile(
    const math::QuinticPolynomialCurve1d& curve) {
  for (double evaluate_t = 0.1; evaluate_t <= curve.ParamLength();
       evaluate_t += 0.2) {
    const double v = curve.Evaluate(1, evaluate_t);
    const double a = curve.Evaluate(2, evaluate_t);
    constexpr double kEpsilon = 1e-3;
    if (v < -kEpsilon || a < -5.0) {
      return false;
    }
  }
  return true;
}

double CalculateStopDistance(
    ReferenceLineInfo* reference_line_info, const double init_s) {
  double front_clear_distance = FLAGS_default_front_clear_distance;//@pqg add 
  const double buffer = 1.0;
  auto obstacles = reference_line_info->path_decision()->obstacles().Items();
  auto vehicle_param = EgoInfo::instance()->vehicle_param();

  std::multiset<double> obstacle_dists;//默认从小到大排序
  obstacle_dists.insert(front_clear_distance);

  for (const auto& obstacle : obstacles) {
    if (obstacle->PerceptionSLBoundary().end_s() <= -10 
      && obstacle->IsVirtual() && obstacle->Id().substr(0,1) == "r") { //ignore obj behind @pqg add 
      continue;
    }
    if (obstacle->PerceptionSLBoundary().end_s() <= -1 
      && obstacle->IsVirtual() && obstacle->Id().substr(0,1) == "s") { //ignore obj behind @pqg add 
      continue;
    }
    if (obstacle->PerceptionSLBoundary().end_s() <= EgoInfo::instance()->vehicle_front_edge_to_center()
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
        return front_clear_distance;
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
        if (boudary_l_min <= vehicle_param.half_wheel+ FLAGS_static_decision_nudge_l_buffer) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"based on sl_boundary (boudary_l_min = "<<boudary_l_min
                <<"), this obstacle must be made into consideration. ";
          is_on_path = true;
        } else if (boudary_l_min <= vehicle_param.half_wheel+ FLAGS_static_decision_nudge_l_buffer + 0.2
           && obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::FOLLOW) {
          is_on_path = true;
        }
      }
    } 
    double dist = obstacle->PerceptionSLBoundary().start_s();
    if (!obstacle->IsStatic()) {
      if (obstacle->speed() > EgoInfo::instance()->vehicle_state_new().linear_velocity) {
        double safe_dist = 0.5 * EgoInfo::instance()->vehicle_state_new().linear_velocity + 5.0;
        if (dist < safe_dist + EgoInfo::instance()->vehicle_front_edge_to_center()) {
          dist = dist + obstacle->speed()*obstacle->speed();//assume it will decellerate with -0.5m/s^2;
        } else {
          continue;
        }
      } else {
        dist = dist + obstacle->speed()*obstacle->speed() / 2; //assume it will decellerate with -1m/s^2;
      }
    } else {// isStatic
      if (obstacle->IsVirtual()) {//isVirtual
        dist = dist - init_s - EgoInfo::instance()->vehicle_front_edge_to_center() - 0.01;
      } else {
        double stop_distance = obstacle->GetStopDistance(vehicle_param);
        AINFO_IF(FLAGS_enable_debug_motion)<<"dist "<<dist<<", init_s = "<<init_s
                  <<", vehicle_front_edge_to_center "<<EgoInfo::instance()->vehicle_front_edge_to_center()
                  <<", stop_distance = "<<stop_distance;
        dist = dist - init_s - EgoInfo::instance()->vehicle_front_edge_to_center() 
           - stop_distance;
        AINFO_IF(FLAGS_enable_debug_motion)<<"dist "<<dist;   
      }
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"dist "<<dist; 
    if (is_on_path) {
      obstacle_dists.insert(dist);
    }
    for (const auto& s : obstacle_dists) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"s = "<<s;
    }
  }

  front_clear_distance = *(obstacle_dists.begin());
  AINFO_IF(FLAGS_enable_debug_motion)<<"front_clear_distance = "<<front_clear_distance;

  return front_clear_distance;
}

}
}  // namespace planning
}  // namespace acu
