/**
 * @file st_boundary_mapper.cpp
 **/

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "pnc_point.pb.h"

#include "common/util/util.h"
#include "common/util/file.h"
#include "common/math/vec2d.h"
#include "st_boundary_mapper.h"
#include "common/util/string_util.h"
#include "common/math/line_segment2d.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;
using acu::common::math::Box2d;
using acu::common::math::Vec2d;
using acu::common::util::StrCat;

namespace {
constexpr double boundary_t_buffer = 0.1;
constexpr double boundary_s_buffer = 1.0;
}  // namespace

StBoundaryMapper::StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathInfo& path_data,
                                   const double planning_distance,
                                   const double planning_time,
                                   bool is_change_lane)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      planning_distance_(planning_distance),
      planning_time_(planning_time),
      is_change_lane_(is_change_lane) {
  vehicle_param_ = EgoInfo::instance()->vehicle_param(); 
  back_edge_to_center = EgoInfo::instance()->vehicle_back_edge_to_center();
  front_edge_to_center = EgoInfo::instance()->vehicle_front_edge_to_center();
  left_edge_to_center = vehicle_param_.half_wheel;
  right_edge_to_center = left_edge_to_center;
  is_reversing_ = path_data.IsReverse();
  blindScope_ = is_reversing_ ? back_edge_to_center : front_edge_to_center;
}

Status StBoundaryMapper::CreateStBoundary(PathDecision* path_decision) const {
  const auto& obstacles = path_decision->obstacles();

  if (planning_time_ < 0.0) {
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AWARN_IF(FLAGS_enable_debug_motion)<<"[StBoundaryMapper]" << msg;
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (path_data_.discretized_path().size() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Fail to get params because of too few path points. path points "
              "size: "
              << path_data_.discretized_path().size() << ".";
    return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR,
                  "Fail to get params because of too few path points");
  }

  Obstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto* const_obstacle : obstacles.Items()) {
    auto* obstacle = path_decision->Find(const_obstacle->Id());
    bool is_bycycle_or_pedestrian =
        (obstacle->Perception().type == 2 || //PEDESTRIAN 
         obstacle->Perception().type == 3 ); //CYCLIST

    const auto& lat_decision = obstacle->LateralDecision();
    if (!is_bycycle_or_pedestrian && obstacle->IsStatic() &&
        lat_decision.has_nudge()) {
      continue;
    }

    if (!obstacle->HasLongitudinalDecision()) {
      if (!MapWithoutDecision(obstacle).ok()) {  
        std::string msg = StrCat("Fail to map obstacle ", obstacle->Id(),
                                 " without decision.");
        AWARN_IF(FLAGS_enable_debug_motion)<< msg;
        return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
      }
      continue;
    }
    
    const auto& decision = obstacle->LongitudinalDecision();
    if (decision.has_stop()) {
      const double stop_s = obstacle->PerceptionSLBoundary().start_s() +
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 15.0;
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {
        AWARN_IF(FLAGS_enable_debug_motion)<< "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, "invalid decision");
      }
      if (stop_s < min_stop_s) {
        stop_obstacle = obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() || 
               decision.has_yield()) {
      if (!MapWithDecision(obstacle, decision).ok()) {
        AWARN_IF(FLAGS_enable_debug_motion)<< "Fail to map obstacle " << obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } else if (!decision.has_ignore()) {
      // AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AWARN_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_SPEEDPLAN_SOFTWARE_0_ERROR, msg);
    }
  }
  return Status::OK();
}

bool StBoundaryMapper::MapStopDecision(
    Obstacle* stop_obstacle, const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";

  if (stop_obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary_.end_s() + planning_distance_) {
    return true;
  }

  double st_stop_s = 0.0;

  double stop_ref_s = stop_obstacle->PerceptionSLBoundary().start_s() +
                            stop_decision.stop().distance_s() -
                            blindScope_;

  if (!stop_obstacle->IsStatic() 
       && FLAGS_enable_follow_obj_status_estimation) {
    const double deceleration_estimate = std::fmin(-1.0, stop_obstacle->acc());
    const double obj_speed = stop_obstacle->speed_along_reference_line();
    double stop_distance_estimate = fabs(obj_speed*obj_speed/2/deceleration_estimate);
    stop_ref_s = stop_ref_s + stop_distance_estimate;
  }                          

  if (stop_ref_s > path_data_.frenet_frame_path().back().s()) {
    st_stop_s = path_data_.discretized_path().back().s() +
                (stop_ref_s - path_data_.frenet_frame_path().back().s());
  } else {
    common::PathPoint stop_point;
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point) && !stop_obstacle->IsVirtual()) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Fail to get path point from reference s. The sl boundary of "
                "stop obstacle "
             << stop_obstacle->Id()
             << " is: " << stop_obstacle->PerceptionSLBoundary().DebugString();
      return false;
    }

    st_stop_s = stop_point.s();
  }

  constexpr double kStopEpsilon = 1e-2;
  const double s_min = std::max(0.0, st_stop_s - kStopEpsilon);
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer(), planning_time_));
  auto boundary = StBoundary(point_pairs);
  AINFO_IF(FLAGS_enable_debug_motion)<< " MapStopDecision, s_min = "<<s_min<<", id  = "<<stop_obstacle->Id()
                           <<"blindScope_: "<<blindScope_<<" stop_obstacle.start_s(): "<< stop_obstacle->PerceptionSLBoundary().start_s()
                           <<"path_end_s: "<<path_data_.frenet_frame_path().back().s();
  boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  boundary.SetObstacleId(stop_obstacle->Id(), stop_obstacle->IsVirtual());
  stop_obstacle->SetStBoundary(boundary);
  return true;
}

Status StBoundaryMapper::MapWithoutDecision(Obstacle* obstacle) const {
  if (FLAGS_using_cognition_st_boundary && !obstacle->IsVirtual() 
      && !obstacle->IsStatic()
      && !obstacle->reference_line_st_boundary().IsEmpty() 
      && !path_data_.is_new()) {
    StBoundary boundary;
    boundary = obstacle->reference_line_st_boundary();
    boundary.SetObstacleId(obstacle->Id(),obstacle->IsVirtual());
    boundary.SetSpeed(obstacle->speed());
    obstacle->SetStBoundary(boundary);
    return Status::OK();
  }

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  if (obstacle->has_over_lap()  
       && !obstacle->overlap_points().lower_points.empty()
       && !obstacle->overlap_points().upper_points.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"calulate overlap before, skip GetOverlapBoundaryPoints fuction.";
    lower_points = obstacle->overlap_points().lower_points;
    upper_points = obstacle->overlap_points().upper_points;
  } else {
    if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                  &upper_points, &lower_points)) {
      return Status::OK();
    }
    Obstacle::OverlapPoints overlap_points;
    overlap_points.lower_points = lower_points;
    overlap_points.upper_points = upper_points;
    obstacle->Set_Overlap_Points(overlap_points);
    obstacle->Set_has_overlap(true); 
  }
  
  StBoundary boundary;
  boundary = StBoundary::GenerateStBoundary(lower_points, upper_points);
  boundary.SetObstacleId(obstacle->Id(),obstacle->IsVirtual());
  const auto& prev_st_boundary = obstacle->st_boundary();
  const auto& ref_line_st_boundary = obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }

  obstacle->SetStBoundary(boundary);
  AINFO_IF(FLAGS_enable_debug_motion)<<" obstacle(without decision) id :"<<obstacle->Id()<<",speed:"<<obstacle->speed()
      <<",st boundary (t,s1,s2) ["<<boundary.lower_points().front().x() 
      <<", "<<boundary.lower_points().front().y()<<", "<<boundary.upper_points().front().y()
      <<"]; ["<<boundary.lower_points().back().x() 
      <<", "<<boundary.lower_points().back().y()<<", "<<boundary.upper_points().back().y()<<"] . ";
  return Status::OK();
}

bool StBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<common::PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  DCHECK_NOTNULL(upper_points);
  DCHECK_NOTNULL(lower_points);
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  DCHECK_GT(path_points.size(), 0);

  if (path_points.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"No points in path_data_.discretized_path().";
    return false;
  }

  const auto& trajectory = obstacle.Trajectory();
  if (trajectory.trajectory_point_size() == 0) {
    if (!obstacle.IsStatic()) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Non-static obstacle[" << obstacle.Id()
             << "] has NO prediction trajectory.";
    }
    for (const auto& curr_point_on_path : path_points) {
      if (curr_point_on_path.s() > planning_distance_) {
        break;
      }
      const Box2d obs_box = obstacle.PerceptionBoundingBox();

      if (CheckOverlap(curr_point_on_path, obs_box,
                       st_boundary_config_.boundary_buffer()) 
                        && CheckSLboundaryIsOnPath(obstacle.PerceptionSLBoundary())) {//@pqg add CheckSLboundary 
        const double backward_distance = -1 * blindScope_;
        const double forward_distance = vehicle_param_.length +
                                        vehicle_param_.car_width +
                                        obs_box.length() + obs_box.width();
        double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);
        double high_s = std::fmin(planning_distance_,
                                  curr_point_on_path.s() + forward_distance);
        double movement_distance = planning_time_ * obstacle.speed();                       
        lower_points->emplace_back(low_s, 0.0);
        upper_points->emplace_back(high_s, 0.0);
        bool use_obstacle_speed = true;//false; //apollo默认false @pqg add
        lower_points->emplace_back(low_s + movement_distance, planning_time_);
        upper_points->emplace_back(high_s + movement_distance, planning_time_);
        break;
      }
    }
  } else {
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
    if (path_points.size() > 2 * default_num_point) {
      const auto ratio = path_points.size() / default_num_point;
      std::vector<common::PathPoint> sampled_path_points;
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path = DiscretizedPath(sampled_path_points);
    } else {
      discretized_path = DiscretizedPath(path_points);
    }
    //avoid U-turn 2 collision point @pqg
    int last_index = -1;
    const int kIndexDiff = 10;
    bool break_cycle = false;

    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
      const auto& trajectory_point = trajectory.trajectory_point(i);
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);
      
      double trajectory_point_time = trajectory_point.relative_time();
      constexpr double kNegtiveTimeThreshold = -1.0;
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      const double step_length = blindScope_;
      auto path_len =
          std::min(FLAGS_max_trajectory_len, discretized_path.Length());
      for (double path_s = 0.0; path_s < path_len; path_s += step_length) {
        const auto curr_adc_path_point =
            discretized_path.Evaluate(path_s + discretized_path.front().s());
        if (CheckOverlap(curr_adc_path_point, obs_box,
                         st_boundary_config_.boundary_buffer())) {
          // AINFO_IF(FLAGS_enable_debug_motion)<<"prediction i = "<<i <<", path_s = "<<path_s;
          if (last_index > 0 && fabs(last_index - i) > kIndexDiff) {//avoid U-turn 2 collision point @pqg
            break_cycle = true;
            AWARN_IF(FLAGS_enable_debug_motion)<<"There exists at least two collision position, just consider the first position , break. ";
            break;
          }
          // found overlap, start searching with higher resolution
          const double backward_distance = -step_length;
          const double forward_distance = vehicle_param_.length +
                                          vehicle_param_.car_width +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);

          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.front().s());
              if (!CheckOverlap(point_low, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.front().s());
              if (!CheckOverlap(point_high, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
          if (find_high && find_low) {
            // AINFO_IF(FLAGS_enable_debug_motion)<<"low_s = "<<low_s<<", high_s = "<<high_s;
            lower_points->emplace_back(
                low_s - st_boundary_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + st_boundary_config_.point_extension(),
                trajectory_point_time);
          }

          last_index = i;//@pqg avoid U-turn 2 collision point 

          break;
        }
      }
      if (break_cycle) break;//@pqg avoid U-turn 2 collision point
    }
  }
  DCHECK_EQ(lower_points->size(), upper_points->size());
  if(lower_points->size() <= 1 && upper_points->size() <= 1){
    AINFO_IF(FLAGS_enable_debug_motion)<< "ego & No["<<obstacle.Id()<<"] obstacle has no collision in future." ;
  }
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

bool StBoundaryMapper::CheckSLboundaryIsOnPath(const SLBoundary& ref_line_sl_boundary) const {
  const auto &frenet_path = path_data_.frenet_frame_path();
  const auto frenet_point = frenet_path.GetNearestPoint(ref_line_sl_boundary);//从障碍物的boundary计算找到与路径的最小距离的路点
  const double curr_l = frenet_point.l();
  SLBoundary new_sl_boundary;
  new_sl_boundary.set_start_s(ref_line_sl_boundary.start_s());
  new_sl_boundary.set_end_s(ref_line_sl_boundary.end_s());
  new_sl_boundary.set_start_l(ref_line_sl_boundary.start_l() - curr_l);
  new_sl_boundary.set_end_l(ref_line_sl_boundary.end_l() - curr_l);
  if (new_sl_boundary.start_l() * new_sl_boundary.end_l() <= 0) {
    return true;
  }
  double boudary_l_min = 
         std::fmin(fabs(new_sl_boundary.start_l()),fabs(new_sl_boundary.end_l()));
  if (boudary_l_min <= vehicle_param_.half_wheel+ FLAGS_static_decision_nudge_l_buffer) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"based on sl_boundary (boudary_l_min = "<<boudary_l_min
          <<"), this obstacle must be made into consideration. ";
    return true;
  }
  return false;
}

Status StBoundaryMapper::MapWithDecision(
  Obstacle* obstacle, const ObjectDecisionType& decision) const {
  AINFO_IF(FLAGS_enable_debug_motion)<<"debug 02";
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  StBoundary boundary;  
  if (FLAGS_using_cognition_st_boundary && !obstacle->IsVirtual() 
      && !obstacle->IsStatic()
      && !obstacle->reference_line_st_boundary().IsEmpty() 
      && !path_data_.is_new()) {
    boundary = obstacle->reference_line_st_boundary();
    boundary.SetObstacleId(obstacle->Id(),obstacle->IsVirtual());
    boundary.SetSpeed(obstacle->speed());
    obstacle->SetStBoundary(boundary);
  } else {
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
  
    if (obstacle->has_over_lap()  
         && !obstacle->overlap_points().lower_points.empty()
         && !obstacle->overlap_points().upper_points.empty()) {
      // AINFO_IF(FLAGS_enable_debug_motion)<<"calculate overlap before, skip GetOverlapBoundaryPoints fuction.";
      lower_points = obstacle->overlap_points().lower_points;
      upper_points = obstacle->overlap_points().upper_points;
    } else {
      if (!GetOverlapBoundaryPoints(path_data_.discretized_path(), *obstacle,
                                    &upper_points, &lower_points)) {
        if (!(obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::GIVEWAY)
          || obstacle->reference_line_st_boundary().IsEmpty() || path_data_.is_new()) {
          return Status::OK();
        }
      }
      Obstacle::OverlapPoints overlap_points;
      overlap_points.lower_points = lower_points;
      overlap_points.upper_points = upper_points;
      obstacle->Set_Overlap_Points(overlap_points);
      obstacle->Set_has_overlap(true);
    }
    if (!lower_points.empty() && !upper_points.empty()) {
      boundary = StBoundary::GenerateStBoundary(lower_points, upper_points);
    }
  }

  if (decision.has_follow()) {
    bool ignore_obstacle_intention = (FLAGS_ignore_obstacle_acceleration_intention && obstacle->acc() >= 0) || 
                                     (FLAGS_ignore_obstacle_deceleration_intention && obstacle->acc() <= 0) || 
                                     (!FLAGS_ignore_obstacle_acceleration_intention 
                                         && obstacle->acc() > 0 && obstacle->acc() < FLAGS_ignore_obstacle_acceleration_max) || 
                                     (!FLAGS_ignore_obstacle_deceleration_intention 
                                         && obstacle->acc() < 0 && obstacle->acc() > FLAGS_ignore_obstacle_deceleration_max) ? 
                                      true : false;
    AINFO_IF(FLAGS_enable_debug_motion)<<"ignore_obstacle_intention = "<<ignore_obstacle_intention;
    if (boundary.IsEmpty()) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"boundary.IsEmpty";
      return Status::OK();
    }                                  
    std::vector<STPoint> lower_points = boundary.lower_points();
    std::vector<STPoint> upper_points = boundary.upper_points();
    if (ignore_obstacle_intention 
          && !lower_points.empty() && !upper_points.empty()) {//special process 
      AINFO_IF(FLAGS_enable_debug_motion)<<"ignore obstacle accelleration.";
      double low_s = std::fmin(planning_distance_ - 0.5, lower_points.front().s());
      double high_s = std::fmin(planning_distance_, upper_points.front().s());
      double movement_distance = planning_time_ * obstacle->speed();  
      AINFO_IF(FLAGS_enable_debug_motion)<<"low_s = "<<low_s<<", high_s = "<<high_s<<", movement_distance = "<<movement_distance;
      lower_points.clear(); 
      upper_points.clear();                    
      lower_points.emplace_back(low_s, lower_points.front().t());
      upper_points.emplace_back(high_s, lower_points.front().t());
      lower_points.emplace_back(low_s + movement_distance, planning_time_);
      upper_points.emplace_back(high_s + movement_distance, planning_time_);
    }
    if (lower_points.back().t() < planning_time_) {
      const double diff_s = lower_points.back().s() - lower_points.front().s();
      const double diff_t = lower_points.back().t() - lower_points.front().t();
      double extend_lower_s =
          diff_s / diff_t * (planning_time_ - lower_points.front().t()) +
          lower_points.front().s();
      const double extend_upper_s =
          extend_lower_s + (upper_points.back().s() - lower_points.back().s()) +
          1.0;
      upper_points.emplace_back(extend_upper_s, planning_time_);
      lower_points.emplace_back(extend_lower_s, planning_time_);
    }
    boundary = StBoundary::GenerateStBoundary(lower_points, upper_points);
  }

  if (boundary.IsEmpty()) {
    return Status::OK();
  }

  // get characteristic_length and boundary_type.
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    b_type = StBoundary::BoundaryType::FOLLOW;
  } else if (decision.has_yield()) {
    const double yield_t_buffer = 1.0;
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = boundary.ExpandByS(characteristic_length);
    AINFO_IF(FLAGS_enable_debug_motion)<<" obstacle(with decision) id :"<<obstacle->Id()<<",speed:"<<obstacle->speed()
      <<", 1) yield process st boundary (t,s1,s2) ["<<boundary.lower_points().front().x() 
      <<", "<<boundary.lower_points().front().y()<<", "<<boundary.upper_points().front().y()
      <<"]; ["<<boundary.lower_points().back().x() 
      <<", "<<boundary.lower_points().back().y()<<", "<<boundary.upper_points().back().y()<<"] . "; 
    
    if (FLAGS_enable_expand_t_in_yield_boundary && !boundary.IsEmpty()) {
      double expand_t = 8.0 - boundary.max_t();             
      if (expand_t > 0.0) {
        boundary = boundary.RightBoundaryExpandByT(expand_t);
      }
      
      AINFO_IF(FLAGS_enable_debug_motion)<<" expand_t : "<<expand_t;
      AINFO_IF(FLAGS_enable_debug_motion)<<" obstacle(with decision) id :"<<obstacle->Id()<<",speed:"<<obstacle->speed()
      <<", 2) yield process st boundary (t,s1,s2) ["<<boundary.lower_points().front().x() 
      <<", "<<boundary.lower_points().front().y()<<", "<<boundary.upper_points().front().y()
      <<"]; ["<<boundary.lower_points().back().x() 
      <<", "<<boundary.lower_points().back().y()<<", "<<boundary.upper_points().back().y()<<"] . ";             
    }              
    b_type = StBoundary::BoundaryType::YIELD;
  } else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = StBoundary::BoundaryType::OVERTAKE;
    //换道规划或处于换道过程中时，在目标车道的障碍物若决策为超越，只需要规划超越第一个st_point即可，即默认自车换到目标车道后该障碍?
	//与自车发生碰撞
    if (reference_line_.IsChangeLanePath() && 
      (obstacle->conflict_type_on_target() == 1 || obstacle->conflict_type_on_target() == 2)) {
      std::vector<STPoint> lower_points;
      std::vector<STPoint> upper_points;
      lower_points.emplace_back(boundary.lower_points().front());
      lower_points.emplace_back(boundary.lower_points().front().s(),boundary.lower_points().back().t());
      upper_points.emplace_back(boundary.upper_points().front());
      upper_points.emplace_back(boundary.upper_points().front().s(),boundary.upper_points().back().t());
      boundary = StBoundary::GenerateStBoundary(lower_points, upper_points);
    }
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  AINFO_IF(FLAGS_enable_debug_motion)<< "[StBoundaryMapper]enter MapWithDecision";
  boundary.SetBoundaryType(b_type);
  boundary.SetObstacleId(obstacle->Id(),obstacle->IsVirtual());
  boundary.SetCharacteristicLength(characteristic_length);
  obstacle->SetStBoundary(boundary);
  AINFO_IF(FLAGS_enable_debug_motion)<<" obstacle(with decision) id :"<<obstacle->Id()<<",speed:"<<obstacle->speed()
      <<",st boundary (t,s1,s2) ["<<boundary.lower_points().front().x() 
      <<", "<<boundary.lower_points().front().y()<<", "<<boundary.upper_points().front().y()
      <<"]; ["<<boundary.lower_points().back().x() 
      <<", "<<boundary.lower_points().back().y()<<", "<<boundary.upper_points().back().y()<<"] . ";
  AINFO_IF(FLAGS_enable_debug_motion)<<" obstacle(with decision) id :"<<obstacle->Id()<<", min_t = "<<boundary.min_t();
  return Status::OK();
}

bool StBoundaryMapper::CheckOverlap(const common::PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double buffer) const {
  double left_delta_l = 0.0;
  double right_delta_l = 0.0;
  if (is_change_lane_) {
    if ((adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) / 2.0 > 0.0) {
      // change to right
      left_delta_l = 1.0;
    } else {
      // change to left
      right_delta_l = 1.0;
    }
  }
  Vec2d vec_to_center =
      Vec2d((front_edge_to_center -
             back_edge_to_center) /
                2.0,
            ((left_edge_to_center + left_delta_l) -
             (right_edge_to_center + right_delta_l)) /
                2.0)
          .rotate(path_point.theta());
  Vec2d center = Vec2d(path_point.x(), path_point.y()) + vec_to_center;

  const Box2d adc_box =
      Box2d(center, path_point.theta(), vehicle_param_.length + 2 * buffer,
            vehicle_param_.car_width + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace acu
