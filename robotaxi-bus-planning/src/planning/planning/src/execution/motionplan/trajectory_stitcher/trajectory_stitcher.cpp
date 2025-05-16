/**
 * @file trajectory_stitcher.cpp
 **/

#include <algorithm>
#include "common/util/util.h"
#include "common/math/angle.h"
#include "trajectory_stitcher.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/new_path_manager.h"

namespace acu {   
namespace planning {
namespace trajectory_stitcher {
using acu::common::TrajectoryPoint;
using acu::common::math::Vec2d;

std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
  const VehicleState& vehicle_state) {
  AWARN_IF(FLAGS_enable_debug_motion) <<"trigger replan";
  common::TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x);
  init_point.mutable_path_point()->set_y(vehicle_state.y);
  init_point.mutable_path_point()->set_z(vehicle_state.z);
  init_point.mutable_path_point()->set_theta(vehicle_state.heading);
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa);
  init_point.set_v(std::max(vehicle_state.linear_velocity, 0.1)); //从当前点开始重规划则速度初始值为当前车速，初始加速度为当前实际加速度
  init_point.set_a(vehicle_state.linear_acceleration);
  init_point.set_relative_time(0.0);
  if (BehaviorParser::instance()->reference_lines_data().size() == 1 
      && vehicle_state.driving_mode < 1) {
    if (BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.size()) {
      geometry::Site point;
      point.xg = vehicle_state.x;
      point.yg = vehicle_state.y;
      double s = 0;
      double l = 0;
      XYToSL(BehaviorParser::instance()->reference_lines_data().front().mapinfo, point,s,l);
      AINFO_IF(FLAGS_enable_debug_motion)<<" l = "<<l;
      if (fabs(l) < FLAGS_replan_lateral_distance_threshold) {
        init_point.mutable_path_point()->set_x(
             BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.front().xg);
        init_point.mutable_path_point()->set_y(
             BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.front().yg); 
        init_point.mutable_path_point()->set_theta(
             BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.front().globalangle * kDEG_TO_RAD);
        init_point.mutable_path_point()->set_kappa(
             BehaviorParser::instance()->reference_lines_data().front().mapinfo.path_points.front().curvature);
      }
    }
  }
  if(BehaviorParser::instance()->is_stop_or_suspend_task()){
        if(vehicle_state.linear_acceleration>0.3){
            init_point.set_a(vehicle_state.linear_acceleration*0.3);
        }else if(vehicle_state.linear_acceleration>0){
            init_point.set_a(0);
        }else{
            init_point.set_a(vehicle_state.linear_acceleration);
        }
  }
  return std::vector<common::TrajectoryPoint>(1, init_point);
}

std::vector<common::TrajectoryPoint> ComputeReplanStartPoint(
            const VehicleState& vehicle_state,const common::TrajectoryPoint& matched_point) {
  common::TrajectoryPoint init_point;
  init_point = matched_point;
  init_point.set_v(std::fmax(std::fmin(vehicle_state.linear_velocity,matched_point.v()), 0.1)); //从当前点开始重规划则速度初始值为当前车速，初始加速度为当前实际加速度
  init_point.set_a(vehicle_state.linear_acceleration);
  init_point.set_relative_time(0.0);
  return std::vector<common::TrajectoryPoint>(1, init_point);
}

std::vector<common::TrajectoryPoint> ComputeReplanStartPoint(
            const PublishableTrajectory* prev_trajectory, const VehicleState& vehicle_state, 
            const VehicleState& vehicle_state_new) {
  if (!prev_trajectory) {
    AWARN_IF(FLAGS_enable_debug_motion) << "prev_trajectory is ptrnull ";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  size_t new_position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state_new.x, vehicle_state_new.y}, 1.0e-6);
  AINFO_IF(FLAGS_enable_debug_motion) << "position_matched_index: " << position_matched_index
                                      << "new_position_matched_index : "<<new_position_matched_index;
  if (position_matched_index > new_position_matched_index) {
    AERROR_IF(FLAGS_enable_debug_motion) << "prev_locpos far away from new_locpos!!!! ";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }                                    

  std::vector<common::TrajectoryPoint> stitching_trajectory(
    prev_trajectory->begin() +
    std::max(0, static_cast<int>(position_matched_index)),
    prev_trajectory->begin() + new_position_matched_index + 1);  
  int count = 0;
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      AWARN_IF(FLAGS_enable_debug_motion) <<"replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    if (tp.v() < 0.1) {
      tp.set_v(0.1);
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"relan start point v["<<count<<"] "<<tp.v()<<" t "<<tp.relative_time()
      <<" s "<<tp.path_point().s()<<" a "<<tp.a();
    count++;
  }

  return stitching_trajectory;
}

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryReInit(
  const VehicleState& vehicle_state, const common::TrajectoryPoint& reinit_point) {
  common::TrajectoryPoint init_point;
  init_point = reinit_point;
  init_point.set_v(std::fmax(vehicle_state.linear_velocity, FLAGS_start_velocity)); //从当前点开始重规划则速度初始值为当前车速，初始加速度为当前实际加速度
  if (vehicle_state.linear_velocity <= FLAGS_start_velocity) {
    init_point.set_a(std::fmax(vehicle_state.linear_acceleration, FLAGS_start_acceleration));
  } else {
    init_point.set_a(vehicle_state.linear_acceleration);
  }
  
  init_point.set_relative_time(0.0);
  return std::vector<common::TrajectoryPoint>(1, init_point);
}

std::vector<common::TrajectoryPoint> ConstLengthStitchingTrajectory(const VehicleState& vehicle_state,
     const PublishableTrajectory* prev_trajectory, const size_t time_matched_index,
     const size_t position_matched_index, const double length, const double veh_rel_time) {
  if (!prev_trajectory) {
    return ComputeReinitStitchingTrajectory(vehicle_state) ;
  }
  double distance_keep = length + 
      prev_trajectory->TrajectoryPointAt(position_matched_index).path_point().s();
  AINFO_IF(FLAGS_enable_debug_motion)<<"position_matched_point s :"
    << prev_trajectory->TrajectoryPointAt(position_matched_index).path_point().s();  
  Vec2d locpos(vehicle_state.x,vehicle_state.y);
  Vec2d position_matched_point(prev_trajectory->TrajectoryPointAt(position_matched_index).path_point().x(),
       prev_trajectory->TrajectoryPointAt(position_matched_index).path_point().y());  
  AINFO_IF(FLAGS_enable_debug_motion)<< "distance between locpos and position_matched_point : "
    << (locpos - position_matched_point).Length();
  size_t stithing_target_index = 0;
  bool find_target_index = false;
  for (size_t i = position_matched_index; i < prev_trajectory->NumOfPoints();++i) {
    if (prev_trajectory->TrajectoryPointAt(i).path_point().s() >= distance_keep) {
      stithing_target_index = i;
      find_target_index = true;
      AINFO_IF(FLAGS_enable_debug_motion)<< "stitching_trajectory start index :"<< i<<", s :" << distance_keep;
      break;
    }
  }
  if (!find_target_index) {
    stithing_target_index = std::max(0, static_cast<int>(prev_trajectory->NumOfPoints() - 1));
    AINFO_IF(FLAGS_enable_debug_motion)<<"prev_trajectory->length < distance_keep"<<distance_keep
      <<", stithing_target_index = "<<stithing_target_index<<", prev_trajectory->NumOfPoints = "<<prev_trajectory->NumOfPoints();
  }

  auto matched_index = std::min(time_matched_index, position_matched_index);
  

  size_t non_zero_index = stithing_target_index;
  if (!FLAGS_enable_two_stitching_trajectory) {
    for (size_t i = stithing_target_index; i > 0;--i) {
      if (prev_trajectory->TrajectoryPointAt(i).v() >= 0.05) {
        non_zero_index = i;
        AINFO_IF(FLAGS_enable_debug_motion)<< "stitching_trajectory start index :"<< i<<", s :" << distance_keep;
        break;
      }
      if (i == 0) {
        non_zero_index = i;
      }
    }
    if (non_zero_index <= matched_index) {
      non_zero_index = matched_index;
    }
  }
  
  AINFO_IF(FLAGS_enable_debug_motion)<<"matched_index = "<<matched_index<<", non_zero_index = "<<non_zero_index;

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max( 0, static_cast<int>(matched_index) ),
      prev_trajectory->begin() + 
          std::max( 0, static_cast<int>(non_zero_index)+1 ) );
  const double zero_s = stitching_trajectory.back().path_point().s();
  AINFO<< "zero_s: "<< zero_s;
  const double zero_t = stitching_trajectory.front().relative_time();

  const double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(matched_index))
                            .relative_time();
  const double time_match = vehicle_state.linear_velocity < 0.5 ? start_time : veh_rel_time;

  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() - time_match);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  // ROS_INFO_STREAM("stithing trajectory back point s : "<<stitching_trajectory.back().path_point().s());
  AINFO_IF(FLAGS_enable_debug_motion)<<"stitching_trajectory size = "<<stitching_trajectory.size();
  return stitching_trajectory;
}

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
  const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
  const double current_timestamp, const double planning_cycle_time,
  const PublishableTrajectory* prev_trajectory, int* replan_reason) {
  if (!prev_trajectory) {
    *replan_reason = 1;
    AWARN_IF(FLAGS_enable_debug_motion) << "prev_trajectory is ptrnull ";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  if (vehicle_state_latest.driving_mode != 0
     || vehicle_state_latest.brake_state == 1) {// 0 : auto mode ,1: manual mode ; brake_state 1 : brake_pedal 
    *replan_reason = 2;
    AWARN_IF(FLAGS_enable_debug_motion) << "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    AWARN_IF(FLAGS_enable_debug_motion) <<"Projected trajectory at time [" << prev_trajectory->header_time()
                     << "] size is zero! Previous planning not exist or failed. Use "
                     << "origin car status instead.";
    *replan_reason = 3;
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  AINFO_IF(FLAGS_enable_debug_motion) << "position_matched_index: " << position_matched_index;
  size_t latest_position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state_latest.x, vehicle_state_latest.y}, 1.0e-6);

  const common::TrajectoryPoint position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index));
  const common::TrajectoryPoint latest_position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(latest_position_matched_index));

  if (prev_trajectory->trajectory_type() ==
            ADCTrajectory::PATH_FALLBACK ||
        prev_trajectory->trajectory_type() ==
            ADCTrajectory::SPEED_FALLBACK && latest_position_matched_point.v() < 0.1) {
    *replan_reason = 8; 
    AWARN_IF(FLAGS_enable_debug_motion) <<"prev trajectory type is FALLBACK ,replan ";
    return ComputeReplanStartPoint(vehicle_state_latest, latest_position_matched_point);
  }

  if (BehaviorParser::instance()->is_restart()) {
    *replan_reason = 9;
    AWARN_IF(FLAGS_enable_debug_motion) << "replan due to restart.";
    return ComputeStitchingTrajectoryReInit(vehicle_state_latest,latest_position_matched_point);
  }

  if (ScenarioContext::instance()->scenario_info().history_lonscenario 
                          == ScenarioContext::eLonScenarioEnum::STOP && 
      !(ScenarioContext::instance()->scenario_info().current_lonscenario 
                          == ScenarioContext::eLonScenarioEnum::STOP) && 
                          vehicle_state.linear_velocity < 0.5) {//add v constraint to avoid stop dismiss and lane change at the same time
    *replan_reason = 10;
    AWARN_IF(FLAGS_enable_debug_motion) <<"target object start moving , replan. ";
    return ComputeStitchingTrajectoryReInit(vehicle_state_latest, latest_position_matched_point);
  }
  if ((BehaviorParser::instance()->has_lateral_behavior() || !NewPathManager::instance()->stitch_state())
        && FLAGS_enable_const_length_stitching_trajectory 
        && vehicle_state.linear_velocity > 0.1) {
    AINFO_IF(FLAGS_enable_debug_motion) << "calulate constance length stitching_trajectory.";
    double keep_current_trajectory_length = 
         FLAGS_stitching_trajectory_time_buffer * vehicle_state_latest.linear_velocity  
            + fabs(latest_position_matched_point.path_point().s() - position_matched_point.path_point().s()); 
    double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
                            .relative_time();
    if (keep_current_trajectory_length > 0) {                                    
      return ConstLengthStitchingTrajectory(vehicle_state, prev_trajectory,
             position_matched_index, position_matched_index, keep_current_trajectory_length, start_time);
    }
  }
  
  double forward_rel_time =  
    prev_trajectory
    ->TrajectoryPointAt(static_cast<uint32_t>(latest_position_matched_index)).relative_time() 
      + planning_cycle_time; 

  AINFO_IF(FLAGS_enable_debug_motion)<<"latest_position_matched_index : "<<latest_position_matched_index
                <<", forward_rel_time(based on drive_system_lag_time) = "<<forward_rel_time;   

  size_t forward_time_index =
    prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  auto matched_index = position_matched_index;

  if (matched_index > forward_time_index) {
    *replan_reason = 14;
    AWARN_IF(FLAGS_enable_debug_motion) << "time matche wrong";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  constexpr size_t kNumPreCyclePoint = 0;

  std::vector<common::TrajectoryPoint> stitching_trajectory(
    prev_trajectory->begin() +
    std::max(0, static_cast<int>(matched_index - kNumPreCyclePoint)),
    prev_trajectory->begin() + forward_time_index + 1);

  AINFO_IF(FLAGS_enable_debug_motion) << "stitching_trajectory size: " << stitching_trajectory.size() ;

  const double zero_s = stitching_trajectory.back().path_point().s();
   AINFO_IF(FLAGS_enable_debug_motion) << "zero_s:  " << zero_s ;
  const double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(matched_index))
                            .relative_time();
  int count = 0;
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = 13;
      AWARN_IF(FLAGS_enable_debug_motion) <<"replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() - start_time);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
    AINFO_IF(FLAGS_enable_debug_motion)<<"v["<<count<<"] "<<tp.v()<<" t "<<tp.relative_time()
      <<" s "<<tp.path_point().s()<<" a "<<tp.a();
    count++;
  }
  return stitching_trajectory;
}

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryForSpeedPlan(
      const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory,
      int* replan_reason) {
  if (!prev_trajectory) {
    *replan_reason = 1;
    AWARN_IF(FLAGS_enable_debug_motion) << "prev_trajectory is ptrnull ";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  AINFO_IF(FLAGS_enable_debug_motion)<<"vehicle_state_latest.driving_mode = "<<vehicle_state_latest.driving_mode;
  if (vehicle_state_latest.driving_mode != 0) {// 0 : auto mode ,1: manual mode ; brake_state 1 : brake_pedal 
    *replan_reason = 2;
    AWARN_IF(FLAGS_enable_debug_motion) << "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    AWARN_IF(FLAGS_enable_debug_motion) <<"Projected trajectory at time [" << prev_trajectory->header_time()
                     << "] size is zero! Previous planning not exist or failed. Use "
                     << "origin car status instead.";
    *replan_reason = 3;
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }
    if (BehaviorParser::instance()->is_stop_or_suspend_task()) {
        AWARN_IF(FLAGS_enable_debug_motion) <<"suspend or stop task ";
        *replan_reason = 5;
        return ComputeReinitStitchingTrajectory(vehicle_state_latest);
    }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  AINFO_IF(FLAGS_enable_debug_motion) << "position_matched_index: " << position_matched_index;
  size_t latest_position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state_latest.x, vehicle_state_latest.y}, 1.0e-6);

  const common::TrajectoryPoint position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index));
  const common::TrajectoryPoint latest_position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(latest_position_matched_index));
  
  if (vehicle_state_latest.brake_state == 1
      || EgoInfo::instance()->is_on_acc_pedal() > 0 ) {//   
    common::TrajectoryPoint init_point;
    init_point = latest_position_matched_point; 
    double start_v = std::fmin(latest_position_matched_point.v(), vehicle_state_latest.linear_velocity); 
    init_point.set_v(std::fmax(start_v, 0.1));
    init_point.set_relative_time(0.0);
    *replan_reason = 4;
    AWARN_IF(FLAGS_enable_debug_motion) <<"vehicle_state_latest.brake_state = 1 ,replan" 
             <<"v = "<<latest_position_matched_point.v()<<", vehicle_state_latest.linear_velocity = "
             <<vehicle_state_latest.linear_velocity<<", init_point.v = "<<init_point.v();
    return std::vector<common::TrajectoryPoint>(1, init_point);
  }
  
  double forward_rel_time =  
    prev_trajectory
    ->TrajectoryPointAt(static_cast<uint32_t>(latest_position_matched_index)).relative_time() 
      + planning_cycle_time; 

  AINFO_IF(FLAGS_enable_debug_motion)<<"latest_position_matched_index : "<<latest_position_matched_index
                <<", forward_rel_time(based on drive_system_lag_time) = "<<forward_rel_time;

  size_t forward_time_index =
    prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  auto matched_index = position_matched_index;

  if (matched_index > forward_time_index) {
    *replan_reason = 14;
    AWARN_IF(FLAGS_enable_debug_motion) << "time matche wrong";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  constexpr size_t kNumPreCyclePoint = 0;
  size_t non_zero_index = matched_index;
  bool has_find_zero_point = false;
  for (size_t i = matched_index; i <= forward_time_index; ++i) {
    if (prev_trajectory->TrajectoryPointAt(i).v() <= 0.01) {
      non_zero_index = i > matched_index ? i - 1 : i;
      has_find_zero_point = true;
      AINFO_IF(FLAGS_enable_debug_motion)<< "stitching_trajectory end reindex as :"<< non_zero_index;
      break;
    }
  }
  if (!has_find_zero_point) {
    non_zero_index = forward_time_index;
  }
  if (non_zero_index <= matched_index- kNumPreCyclePoint) {
    non_zero_index = std::max(0, static_cast<int>(matched_index - kNumPreCyclePoint));
  }

  if (non_zero_index == matched_index) {//如果有零点，并且零点就在当前时间同步后的自车位置
    if (prev_trajectory->trajectory_type() ==
              ADCTrajectory::PATH_FALLBACK ||
          prev_trajectory->trajectory_type() ==
              ADCTrajectory::SPEED_FALLBACK) {
      *replan_reason = 8; 
      AWARN_IF(FLAGS_enable_debug_motion) <<"prev trajectory type is FALLBACK ,replan ";
      return ComputeReplanStartPoint(vehicle_state_latest, latest_position_matched_point);
    }
  
    if (BehaviorParser::instance()->is_restart()) {
      *replan_reason = 9;
      AWARN_IF(FLAGS_enable_debug_motion) << "replan due to restart.";
      return ComputeStitchingTrajectoryReInit(vehicle_state_latest,latest_position_matched_point);
    }
  
    if (ScenarioContext::instance()->scenario_info().history_lonscenario 
                            == ScenarioContext::eLonScenarioEnum::STOP && 
        !(ScenarioContext::instance()->scenario_info().current_lonscenario 
                            == ScenarioContext::eLonScenarioEnum::STOP)) {
      *replan_reason = 10;
      AWARN_IF(FLAGS_enable_debug_motion) <<"target object start moving , replan. ";
      return ComputeStitchingTrajectoryReInit(vehicle_state_latest, latest_position_matched_point);
    }
  }

  std::vector<common::TrajectoryPoint> stitching_trajectory(
    prev_trajectory->begin() +
    std::max(0, static_cast<int>(matched_index - kNumPreCyclePoint)),
    prev_trajectory->begin() + non_zero_index + 1);

  AINFO_IF(FLAGS_enable_debug_motion) << "stitching_trajectory size: " << stitching_trajectory.size() ;

  const double zero_s = stitching_trajectory.back().path_point().s();
   AINFO_IF(FLAGS_enable_debug_motion) << "zero_s:  " << zero_s ;
  const double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(matched_index))
                            .relative_time();
  int count = 0;
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = 13;
      AWARN_IF(FLAGS_enable_debug_motion) <<"replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() - start_time);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
    AINFO_IF(FLAGS_enable_debug_motion)<<"v["<<count<<"] "<<tp.v()<<" t "<<tp.relative_time()
      <<" s "<<tp.path_point().s()<<" a "<<tp.a();
    count++;
  }
  if (stitching_trajectory.size() == 1) {
    stitching_trajectory.front().set_v(std::fmax(0.01, stitching_trajectory.front().v()));
    stitching_trajectory.front().set_a(vehicle_state_latest.linear_acceleration);
  }
  return stitching_trajectory;
}

std::vector<common::TrajectoryPoint> ComputeStitchingTrajectoryForPathPlan(
      const VehicleState& vehicle_state, const VehicleState& vehicle_state_latest,
      const double current_timestamp, const double planning_cycle_time,
      const PublishableTrajectory* prev_trajectory, const ReferenceLineFrame& local_reference_line,
      int* replan_reason) {

  if (!prev_trajectory) {
    *replan_reason = 1;
    AWARN_IF(FLAGS_enable_debug_motion) << "prev_trajectory is ptrnull ";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  AINFO_IF(FLAGS_enable_debug_motion)<<"vehicle_state_latest.driving_mode = "<<vehicle_state_latest.driving_mode;
  if (vehicle_state_latest.driving_mode != 0) {// 0 : auto mode ,1: manual mode ;
    *replan_reason = 2;
    AWARN_IF(FLAGS_enable_debug_motion) << "replan for manual mode.";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    AWARN_IF(FLAGS_enable_debug_motion) <<"Projected trajectory at time [" << prev_trajectory->header_time()
                     << "] size is zero! Previous planning not exist or failed. Use "
                     << "origin car status instead.";
    *replan_reason = 3;
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state.x, vehicle_state.y}, 1.0e-6);
  size_t latest_position_matched_index = prev_trajectory->QueryNearestPointWithBuffer(
                             {vehicle_state_latest.x, vehicle_state_latest.y}, 1.0e-6);
  AINFO_IF(FLAGS_enable_debug_motion) << "position_matched_index: " << position_matched_index
                                      << ", latest_position_matched_index = "<<latest_position_matched_index ;
  const common::TrajectoryPoint position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(position_matched_index));
  const common::TrajectoryPoint latest_position_matched_point = prev_trajectory->TrajectoryPointAt(
          static_cast<uint32_t>(latest_position_matched_index));

  if (false && vehicle_state_latest.brake_state == 1) {//TODO   
    common::TrajectoryPoint init_point;
    init_point = latest_position_matched_point; 
    double start_v = std::fmin(latest_position_matched_point.v(), vehicle_state_latest.linear_velocity); 
    init_point.set_v(std::fmax(start_v, 0.1));
    init_point.set_relative_time(0.0);
    *replan_reason = 4;
    AWARN_IF(FLAGS_enable_debug_motion) <<"vehicle_state_latest.brake_state = 1 ,replan" 
             <<"v = "<<latest_position_matched_point.v()<<", vehicle_state_latest.linear_velocity = "
             <<vehicle_state_latest.linear_velocity<<", init_point.v = "<<init_point.v();
    return std::vector<common::TrajectoryPoint>(1, init_point);
  }

  std::pair<double, double> u_turn_interval = std::make_pair(0.0,0.0);
  bool is_u_turn = BehaviorParser::instance()->target_reference_line().IsUturn(u_turn_interval);
  const double stitching_trajectory_time_buffer = 
                 (is_u_turn && u_turn_interval.first <= 11.0) ? 2.0 : FLAGS_stitching_trajectory_time_buffer;
  double keep_current_trajectory_length = 
         stitching_trajectory_time_buffer * vehicle_state_latest.linear_velocity
            + fabs(latest_position_matched_point.path_point().s() - position_matched_point.path_point().s()); 
  if ((BehaviorParser::instance()->has_lateral_behavior() || !NewPathManager::instance()->stitch_state())
        && FLAGS_enable_const_length_stitching_trajectory) {
    AINFO_IF(FLAGS_enable_debug_motion) << "calulate constance length stitching_trajectory.";
    double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(position_matched_index))
                            .relative_time();  
    double optional_remain_length = 
    GetOptionalKeepLength(prev_trajectory, local_reference_line.objects_,
             vehicle_state_latest.linear_velocity,latest_position_matched_point.path_point().s());
    AINFO_IF(FLAGS_enable_debug_motion)<<"optional_remain_length = "<<optional_remain_length
                       <<", keep_current_trajectory_length = "<<keep_current_trajectory_length;   
    optional_remain_length = std::fmax(optional_remain_length , keep_current_trajectory_length);
    //针对放弃换道的场景，减小黏合线的长度 dzx
    if (BehaviorParser::instance()->abandon_lc_level() > 0) {
      optional_remain_length = std::fmax( vehicle_state_latest.linear_velocity*0.5 + 
      fabs(latest_position_matched_point.path_point().s() - position_matched_point.path_point().s()), 5.0);
      AINFO_IF(FLAGS_enable_debug_motion)<<"ABANDON_LANE_CHANGE "
                      <<", optional_remain_length = "<<optional_remain_length; 
    }
    if (optional_remain_length > 0) {
      return ConstLengthStitchingTrajectory(vehicle_state, prev_trajectory,
             position_matched_index, position_matched_index, optional_remain_length, start_time);
    }
  }

  double forward_rel_time =  
    prev_trajectory
    ->TrajectoryPointAt(static_cast<uint32_t>(latest_position_matched_index)).relative_time() 
      + planning_cycle_time; 

  AINFO_IF(FLAGS_enable_debug_motion)<<"latest_position_matched_index : "<<latest_position_matched_index
                <<", forward_rel_time(based on drive_system_lag_time) = "<<forward_rel_time;   

  size_t forward_time_index =
    prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  auto matched_index = position_matched_index;

  if (matched_index > forward_time_index) {
    *replan_reason = 14;
    AWARN_IF(FLAGS_enable_debug_motion) << "time matche wrong";
    return ComputeReinitStitchingTrajectory(vehicle_state_latest);
  }

  constexpr size_t kNumPreCyclePoint = 0;

  std::vector<common::TrajectoryPoint> stitching_trajectory(
    prev_trajectory->begin() +
    std::max(0, static_cast<int>(matched_index - kNumPreCyclePoint)),
    prev_trajectory->begin() + forward_time_index + 1);

  AINFO_IF(FLAGS_enable_debug_motion) << "stitching_trajectory size: " << stitching_trajectory.size() ;

  const double zero_s = stitching_trajectory.back().path_point().s();
   AINFO_IF(FLAGS_enable_debug_motion) << "zero_s:  " << zero_s ;
  const double start_time = prev_trajectory
                            ->TrajectoryPointAt(static_cast<uint32_t>(matched_index))
                            .relative_time();
  int count = 0;
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      *replan_reason = 13;
      AWARN_IF(FLAGS_enable_debug_motion) <<"replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() - start_time);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
    AINFO_IF(FLAGS_enable_debug_motion)<<"v["<<count<<"] "<<tp.v()<<" t "<<tp.relative_time()
      <<" s "<<tp.path_point().s()<<" a "<<tp.a();
    count++;
  }
  return stitching_trajectory;
}

double GetOptionalKeepLength(
            const PublishableTrajectory* prev_trajectory, const std::map<int, LineObject>& obstacles,
            const double& car_speed, const double car_s_in_pre_traj) {
  double remain_length = 0.0;

  common::TrajectoryPoint best_trajectory_stitcher_end_point;
  double lateral_behavior_obj_min_s = 150;
  LineObject ptr_obstacle;
  int obstacle_id;
  bool obstacle_is_static;
  StructSLBoundary sl_boundary;
  bool find_nearest_obj = false;
  for (auto it = obstacles.begin(); it != obstacles.end(); it++) {
    ptr_obstacle = it->second;
    obstacle_id = ptr_obstacle.id;
    obstacle_is_static = ptr_obstacle.is_static;
    sl_boundary = ptr_obstacle.sl_boundary;
    if(sl_boundary.max_s < 0 || sl_boundary.min_s > lateral_behavior_obj_min_s || 
      sl_boundary.max_l < -1.75 || sl_boundary.min_l > 1.75 || obstacle_is_static == false)  {
      continue;
    }
    lateral_behavior_obj_min_s = sl_boundary.min_s;
    find_nearest_obj = true;
  }
  double path_bound_limit_min_s = 150;
  PathBound path_bound =  BehaviorParser::instance()->path_bound();
  for (int i = 0; i < path_bound.size(); i++) {
    if (get<1>(path_bound.at(i)) == get<2>(path_bound.at(i)) && get<1>(path_bound.at(i)) != 10) {
      path_bound_limit_min_s = get<0>(path_bound.at(std::max(0, i - 1)));
      AINFO_IF(FLAGS_enable_debug_motion) << "path_bound_limit_min_s = " << path_bound_limit_min_s;
      break;
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion) << "lateral_behavior_obj_min_s = " << lateral_behavior_obj_min_s;
  AINFO_IF(FLAGS_enable_debug_motion) << "vehicle_state_latest.linear_velocity = " << car_speed;
  double min_lateral_behavior_dis = 7 * car_speed;
  min_lateral_behavior_dis = std::fmax(min_lateral_behavior_dis, 11);//预留的最理想的换道距离
  double max_lateral_behavior_point_s = std::fmin(lateral_behavior_obj_min_s, path_bound_limit_min_s) - min_lateral_behavior_dis;
  AINFO_IF(FLAGS_enable_debug_motion) << "1) max_lateral_behavior_point_s = " << max_lateral_behavior_point_s;
  if (max_lateral_behavior_point_s < 0) {//小于0说明无法以理想的换道距离进行操作。
    max_lateral_behavior_point_s = std::fmin(lateral_behavior_obj_min_s, path_bound_limit_min_s) - 11.0;
  }
  AINFO_IF(FLAGS_enable_debug_motion) << "2) max_lateral_behavior_point_s = " << max_lateral_behavior_point_s;

  //如果max_lateral_behavior_point_s点的曲率比较大
  //在横向控制preview_dis和max_lateral_behavior_point_s之间选取一个合适的点
  double lateral_control_preview_dis = 1.5 * car_speed;
  // lateral_control_preview_dis = std::fmax(lateral_control_preview_dis, 4);
  double s = max_lateral_behavior_point_s;
  double min_kappa_s = max_lateral_behavior_point_s;
  size_t optimal_point_index = 0;
  double min_kappa = 1;
  AINFO_IF(FLAGS_enable_debug_motion) << "find_nearest_obj = " << find_nearest_obj;
  AINFO_IF(FLAGS_enable_debug_motion) << "lateral_control_preview_dis = " << lateral_control_preview_dis;
  if (find_nearest_obj == false) {
    max_lateral_behavior_point_s = lateral_control_preview_dis + 1;
  }
  AINFO_IF(FLAGS_enable_debug_motion) << "3) max_lateral_behavior_point_s = " << max_lateral_behavior_point_s;

  if (max_lateral_behavior_point_s > lateral_control_preview_dis) {
    for (s = lateral_control_preview_dis; s < max_lateral_behavior_point_s; s = s + 0.1) {
      size_t index_of_s = prev_trajectory->QueryLowerBoundPointByS(s);
      double kappa = prev_trajectory
                  ->TrajectoryPointAt(static_cast<uint32_t>(index_of_s)).path_point().kappa(); 
      if (fabs(kappa) < fabs(min_kappa)) {
        min_kappa = kappa;
        optimal_point_index = index_of_s;
      }
      if (fabs(min_kappa) < 0.005) {
        break;
      }
    }
  } else {
    optimal_point_index = prev_trajectory->QueryLowerBoundPointByS(max_lateral_behavior_point_s);
  }
  AINFO_IF(FLAGS_enable_debug_motion) << "optimal_point_index = " << optimal_point_index;
  best_trajectory_stitcher_end_point = prev_trajectory
              ->TrajectoryPointAt(static_cast<uint32_t>(optimal_point_index));
  AINFO_IF(FLAGS_enable_debug_motion) << "best_trajectory_stitcher_end_point.path_point().s() = " << best_trajectory_stitcher_end_point.path_point().s();
  remain_length = best_trajectory_stitcher_end_point.path_point().s() - car_s_in_pre_traj;
  return remain_length;

}

void FixStitchingTrajectoryByDR(const VehicleState& vehicle_state,
    const VehicleState& vehicle_state_latest,
    std::vector<common::TrajectoryPoint>& stitching_trajectory) {
  double dr_yaw_rad = vehicle_state.dr_yaw;
  double yaw_rad = vehicle_state.yaw;
  if (BehaviorParser::instance()->has_lateral_behavior()) {
    if ( stitching_trajectory.size() == 1) {
      stitching_trajectory.back().mutable_path_point()->set_dr_x(vehicle_state_latest.dr_x);
      stitching_trajectory.back().mutable_path_point()->set_dr_y(vehicle_state_latest.dr_y);
      stitching_trajectory.back().mutable_path_point()->set_dr_theta(vehicle_state_latest.dr_yaw);
    } else if ( !stitching_trajectory.empty() ){
      AWARN_IF(FLAGS_enable_debug_motion)<<__FUNCTION__<<fixed<<std::setprecision(3)
        <<" dr_yaw "<<dr_yaw_rad<<" yaw"<<yaw_rad
        <<" xg "<<vehicle_state.x<<" yg "<<vehicle_state.y
        <<" drxg "<<vehicle_state.dr_x<<" dryg "<<vehicle_state.dr_y;
      for (auto& point:stitching_trajectory) {
        double dx =  (point.path_point().dr_x() - vehicle_state.dr_x) * cos(dr_yaw_rad)
                    +(point.path_point().dr_y() - vehicle_state.dr_y) * sin(dr_yaw_rad);
        double dy = -(point.path_point().dr_x() - vehicle_state.dr_x) * sin(dr_yaw_rad)
                    +(point.path_point().dr_y() - vehicle_state.dr_y) * cos(dr_yaw_rad);
        double dtheta = point.path_point().dr_theta() - dr_yaw_rad;
        point.mutable_path_point()->set_x(dx * cos(yaw_rad) - dy * sin(yaw_rad) + vehicle_state.x);
        point.mutable_path_point()->set_y(dx * sin(yaw_rad) + dy * cos(yaw_rad) + vehicle_state.y);
        double theta = acu::common::math::NormalizeAngle(dtheta + yaw_rad);
        point.mutable_path_point()->set_theta(theta);
      }
    }
  } else {
    for (auto& point:stitching_trajectory) {
      double dx =  (point.path_point().x() - vehicle_state.x) * cos(yaw_rad)
                  +(point.path_point().y() - vehicle_state.y) * sin(yaw_rad);
      double dy = -(point.path_point().x() - vehicle_state.x) * sin(yaw_rad)
                  +(point.path_point().y() - vehicle_state.y) * cos(yaw_rad);
      double dtheta = point.path_point().theta() - yaw_rad;
      point.mutable_path_point()->set_dr_x(dx * cos(dr_yaw_rad) - dy * sin(dr_yaw_rad) + vehicle_state.dr_x);
      point.mutable_path_point()->set_dr_y(dx * sin(dr_yaw_rad) + dy * cos(dr_yaw_rad) + vehicle_state.dr_y);
      double theta = acu::common::math::NormalizeAngle(dtheta + dr_yaw_rad);
      point.mutable_path_point()->set_dr_theta(theta);
    }
  }
}

}
}  // namespace planning
}  // namespace acu
