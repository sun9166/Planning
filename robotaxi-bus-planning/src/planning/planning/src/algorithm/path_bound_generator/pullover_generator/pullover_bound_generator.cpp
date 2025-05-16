/**
 * @file procedure.cpp
 **/

#include "pullover_bound_generator.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"

namespace acu {
namespace planning {
using acu::common::Status;

PullOverBoundGenerator::PullOverBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info , 
	   const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : PathBoundGenerator("RegularBoundGenerator", frame, reference_line_info, start_frenet_state) {}

common::Status PullOverBoundGenerator::Generate(std::vector<PathBoundary>& candidate_path_boundaries) {
    
  blocked_infos_.reset();
  LaneBound path_bound;
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(&path_bound, &boundary_type_, &dynamic_bound_, &soft_boundary_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  // 2. Decide a rough boundary based on road boundary
  int check_lane_type = 1; //左正右负,0为都校验.
  int path_blocked_idx = -1;
  if (!GetBoundaryFromRoads(&path_bound, check_lane_type, path_blocked_idx)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (path_blocked_idx != -1) {
  	bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
    bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  }

  // if (FLAGS_enable_piecewise_path_debug) {
  //   bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  // }

  // 3. Fine-tune the boundary based on static obstacles
  LaneBound temp_path_bound = path_bound;
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(reference_line_info_->path_decision(),
                                      &path_bound, &blocking_obstacle_id,
                                      FLAGS_obstacle_lat_buffer, &boundary_type_, &soft_boundary_)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  // if (FLAGS_enable_piecewise_path_debug) {
  //   bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  // }

  size_t pull_over_position_index = path_bound.size();

  if (!SearchPullOverPosition(reference_line_info_->path_decision(), 
        path_bound, reference_line_info_->planning_target().stop_point().s(), pull_over_position_index)) {

  } 
  AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"path bound size = "<<path_bound.size();
  if (static_cast<int>(path_bound.size()) > 0) {
    while (static_cast<int>(path_bound.size()) - 1 > pull_over_position_index) {
      path_bound.pop_back();
    }
  }
  AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"after pop_back path bound size = "<<path_bound.size();
  bound_adjustment_ptr_->TrimPathSoftBounds(pull_over_position_index + 1, &soft_boundary_);
  
  // 4. Adjust the boundary considering dynamic obstacles (TODO)
  CalculateDynamicObstacleBound(&dynamic_bound_);

  AINFO_IF(FLAGS_enable_debug_motion) << "Completed generating pull over path boundaries.";
  if (path_bound.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }
  if (!path_bound.empty()) {
    if (adc_frenet_l_ > (std::get<2>(path_bound[0]) + 0.5) || 
       adc_frenet_l_ < (std::get<1>(path_bound[0]) - 0.5)) {
      AERROR << "init point is out of path bound !!!!!!";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "init point is out of path bound .");
    } 
  }
  GetPullOverTailPathBound(&path_bound , BehaviorParser::instance()->dis_to_curb(), reference_line_info_->path_decision());

  std::vector<std::pair<double, double>> path_bound_pair;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_pair;
  std::vector<std::pair<int, int>> path_bound_types;
  for (size_t i = 0; i < path_bound.size(); ++i) {
    if (boundary_type_.size() >= i + 1) {
      path_bound_types.push_back(boundary_type_.at(i)); 
    }
    path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
                                          std::get<2>(path_bound[i]));
    dynamic_obstacle_bound_pair.emplace_back(std::get<1>(dynamic_bound_[i]),
                                          std::get<2>(dynamic_bound_[i]));
  }
  candidate_path_boundaries.emplace_back(
      std::get<0>(path_bound[0]), FLAGS_path_bounds_decider_resolution,
      path_bound_pair);
  candidate_path_boundaries.back().set_label("regular/pullover");
  candidate_path_boundaries.back().set_boundary_type(path_bound_types);
  candidate_path_boundaries.back().set_dynamic_obstacle_boundary(dynamic_obstacle_bound_pair);
  candidate_path_boundaries.back().set_is_blocked(blocked_infos_.is_blocked);
  candidate_path_boundaries.back().set_soft_boundary(soft_boundary_);
	return Status::OK();
}

bool PullOverBoundGenerator::InitPathBoundary(LaneBound* const path_bound,
    std::vector<std::pair<int, int>>* const boundary_type, LaneBound* const dynamic_bound,
    std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  path_bound->clear();
  boundary_type->clear();
  dynamic_bound->clear();
  soft_boundary->clear(); 
  auto vehicle_param = EgoInfo::instance()->vehicle_param();
  const auto& reference_line = reference_line_info_->reference_line();
      double pull_over_front_s= reference_line_info_->planning_target().stop_point().s()+EgoInfo::instance()->vehicle_front_edge_to_center();
    AINFO<<"stop_point().s(): "<<reference_line_info_->planning_target().stop_point().s()<<"BLIND: "<<EgoInfo::instance()->vehicle_front_edge_to_center();
    common::SLPoint sl_point;
    sl_point.set_s(pull_over_front_s);
    sl_point.set_l(reference_line_info_->planning_target().stop_point().l());
    common::math::Vec2d pull_over_front_pos;
    reference_line_info_->reference_line().SLToXY(sl_point, &pull_over_front_pos);
    BehaviorParser::instance()->Set_pull_over_front_pos(pull_over_front_pos);
    AINFO<<"pull_over_front_pos . x: "<<pull_over_front_pos.x()<<" y: "<<pull_over_front_pos.y()
        <<"pull_over_front_s: "<<pull_over_front_s<<", l: "<<sl_point.l();
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info_->reference_line().cruise_speed() * //TODO
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += FLAGS_path_bounds_decider_resolution) {
    path_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
    int direction = BehaviorParser::instance()->lanechange_prepare_direction();
    if (direction <= 0) {
      dynamic_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
    } else if (direction <= 1) {
      dynamic_bound->emplace_back(curr_s, -1.75 + vehicle_param.half_wheel,
                             std::numeric_limits<double>::max());
    } else if (direction <= 2) {
      dynamic_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             1.75 + vehicle_param.half_wheel);
    } else {
      dynamic_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
    }
    
    boundary_type_.emplace_back(make_pair(0,0));
  }

  if (path_bound->empty()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}

bool PullOverBoundGenerator::GetPullOverTailPathBound(LaneBound* const path_bound, const double dis_to_curb,  
      const PathDecision* path_decision) {
  if (path_bound->empty()) return false;
  double curr_road_left_width = 0;
  double curr_road_right_width = 0;
  double pullover_l = reference_line_info_->planning_target().stop_point().l(); 
  const double pullover_s= reference_line_info_->planning_target().stop_point().s();
  const double dis_to_end = EgoInfo::instance()->dis_to_end();
  auto indexed_obstacles = path_decision->obstacles();
  double max_obstacle_l = pullover_l;
  double pullover_road_left_width = 0;
  double pullover_road_right_width = 0;
  // if (reference_line_info_->reference_line().GetMapPath().GetRoadBoundary(std::fmax(0.0,pullover_s), &pullover_road_left_width,
  //                                   &pullover_road_right_width)) {
  //   for (const auto* obstacle : indexed_obstacles.Items()) {
  //     //if (!obstacle->IsStatic() || obstacle->IsUltraStatic());
  //     const auto obstacle_sl = obstacle->PerceptionSLBoundary();
  //     //仅考虑右侧，速度低于0.2m/s，在停车点前后30m内障碍物，靠近路边的
  //     if (obstacle_sl.start_l() < 0 &&  obstacle_sl.end_l() < 0 
  //     && std::fabs(pullover_road_right_width + obstacle_sl.start_l()) < EgoInfo::instance()->vehicle_param().car_width
  //     && obstacle->speed() < 0.5 && std::fabs(reference_line_info_->planning_target().stop_point().s() -
  //     obstacle_sl.end_s()) < 30 ) {
  //       max_obstacle_l = std::fmax(max_obstacle_l, obstacle_sl.end_l());
  //     }
  //   }
  // }

  AINFO_IF(FLAGS_enable_debug_motion) << "!!!!!!max_obstacle_l:" << max_obstacle_l;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double s = static_cast<double>(i) * FLAGS_path_bounds_decider_resolution +
            std::get<0>((*path_bound)[0]);
    //终点30m以内最小的右边界，作为靠边的边界
    if (s >= pullover_s) {
    if (reference_line_info_->reference_line().GetMapPath().GetRoadBoundary(std::fmax(0.0,s), &curr_road_left_width,
                                    &curr_road_right_width)) {
      if (curr_road_right_width > 8.0) curr_road_right_width -= 2.0;
      double curr_road_right_bound = std::fmax(max_obstacle_l, - curr_road_right_width);
      pullover_l = std::fmin(-fabs(curr_road_right_width) + dis_to_curb + EgoInfo::instance()->vehicle_param().half_wheel,0.0);
    } else {
      return false;
    }
      std::get<1>((*path_bound)[i]) = pullover_l;
      std::get<2>((*path_bound)[i]) = pullover_l;
      if (reference_line_info_->IsReverse()) {
        std::get<1>((*path_bound)[i]) = 0.0;
        std::get<2>((*path_bound)[i]) = 0.0;
      }
    }
  }
  return true;
}

bool PullOverBoundGenerator::SearchPullOverPosition(
    const PathDecision* path_decision,
    const std::vector<std::tuple<double, double, double>>& path_bound,const double pull_over_start_s,
    size_t &position_index) {
  auto indexed_obstacles = path_decision->obstacles();
  const double extend_length = 1.0;
  const double pull_over_end_s = 
        pull_over_start_s + EgoInfo::instance()->vehicle_front_edge_to_center() + extend_length;
  //create pullover space as box
  auto func = [](const std::tuple<double, double, double> &tp, const double path_s) {
    return std::get<0>(tp) < path_s;
  };
  auto start_iter = std::lower_bound(path_bound.begin(), path_bound.end(), pull_over_start_s, func);
  common::math::Vec2d start_conner;
  start_conner.set_x(std::get<0>(*start_iter));
  start_conner.set_y(std::get<1>(*start_iter));
  AINFO_IF(FLAGS_enable_debug_motion)<<"start vec ("<<start_conner.x()<<", "<<start_conner.y()<<").";
  auto end_iter = std::lower_bound(path_bound.begin(), path_bound.end(), pull_over_end_s, func);
  common::math::Vec2d end_conner;
  end_conner.set_x(std::get<0>(*end_iter));
  end_conner.set_y(std::get<1>(*end_iter) + 
     BehaviorParser::instance()->dis_to_curb() + EgoInfo::instance()->vehicle_width()
     + FLAGS_static_decision_nudge_l_buffer);
  AINFO_IF(FLAGS_enable_debug_motion)<<"end vec ("<<end_conner.x()<<", "<<end_conner.y()<<").";
  common::math::Box2d pull_over_space = common::math::Box2d::CreateAABox(start_conner, end_conner);
  
  double collision_s_min = std::numeric_limits<double>::max(); 
  bool has_collosion = false;
  for (const auto* obstacle : indexed_obstacles.Items()) {
    if (!obstacle->IsStatic()
    	|| obstacle->IsVirtual() || obstacle->IsUltraStatic()) {
      continue;
    }
    common::math::Vec2d start_conner;
    start_conner.set_x(obstacle->PerceptionSLBoundary().start_s());
    start_conner.set_y(obstacle->PerceptionSLBoundary().start_l());
    AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle start vec ("<<start_conner.x()<<", "<<start_conner.y()<<").";
    common::math::Vec2d end_conner;
    end_conner.set_x(obstacle->PerceptionSLBoundary().end_s());
    end_conner.set_y(obstacle->PerceptionSLBoundary().end_l());
    AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle end vec ("<<end_conner.x()<<", "<<end_conner.y()<<").";
    common::math::Box2d obstacle_sl_box = common::math::Box2d::CreateAABox(start_conner, end_conner);
    if (obstacle_sl_box.HasOverlap(pull_over_space)) {
      if (obstacle->PerceptionSLBoundary().start_s() < collision_s_min) {
        collision_s_min = obstacle->PerceptionSLBoundary().start_s();
        has_collosion = true;
        AINFO_IF(FLAGS_enable_debug_motion)<<"collison ,sl = ("<<obstacle->PerceptionSLBoundary().start_s()<<", "<<obstacle->PerceptionSLBoundary().end_s()
         <<"), ("<<obstacle->PerceptionSLBoundary().start_l()<<", "<<obstacle->PerceptionSLBoundary().end_l()<<").";
      }
    }
  }
  if (has_collosion) {
    auto collision_iter = std::lower_bound(path_bound.begin(), path_bound.end(), collision_s_min, func);
    position_index = std::distance(path_bound.begin(), collision_iter);
  } else {
    position_index = std::distance(path_bound.begin(), end_iter);
  }
  position_index = position_index == 0 ? 0 : position_index -1;//因上述处理得到的是不小于碰撞s的第一个bound对应的index，如果不减1，生成的路径会与障碍物位置有交集，进而导致决策会持续发靠边停车。
  if (position_index < path_bound.size()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"pop index "<<position_index
      <<", s = "<<std::get<0>(path_bound.at(position_index));
  } else {
    AERROR_IF(FLAGS_enable_debug_motion)<<"pop index "<<position_index
      <<", equal or bigger than path_bound size "<<path_bound.size();
  }
  return true;  
}


bool PullOverBoundGenerator::IsWithinPullOverPathDeciderScopeObstacle(const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  const double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  if (obstacle.IsVirtual() || obstacle.IsUltraStatic()) {
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }

  const auto obstacle_sl = obstacle.PerceptionSLBoundary();
  bool is_on_right = false;
  //1.先判断是否在自车右侧
  if (obstacle_sl.start_l() < adc_frenet_l_ &&  obstacle_sl.end_l() < adc_frenet_l_) { 
    // 障碍物任何一个边都在自车的右边
    is_on_right = true;
  } else {//中轴线在自车右边，且超过半个车身
    if ((obstacle_sl.start_l() + obstacle_sl.end_l()) / 2.0 < adc_frenet_l_ + adc_half_width
      && fabs(obstacle_sl.start_l() - adc_frenet_l_) > adc_half_width) {
      is_on_right = true;
    } 
  }

  if (!is_on_right) {//不在右侧，不关注
    return false;
  }
  //2.判断是否有靠边停车安全风险
  if (is_on_right && (!obstacle.IsStatic() || !obstacle.IsUltraStatic())) {//
    double plan_end_s  = reference_line_info_->planning_target().stop_point().s();
    double curr_to_start_pos_velocity = 1e-6 + 0.5 * (reference_line_info_->pathplan_init_point().v() 
              + reference_line_info_->speed_init_point().v()) ;
    double start_plan_rel_t = plan_end_s / curr_to_start_pos_velocity;//自车从当前位置行驶到路径规划起点位置处的时间
    //在该时间内障碍物行驶到的位置
    double obstacle_pos = obstacle_sl.start_s() + obstacle.speed()* start_plan_rel_t;
    if (obstacle_pos > plan_end_s + EgoInfo::instance()->vehicle_length() + 20.0) {
        return false;
    }   
  } 
  //障碍物在终点前30m或者终点之后30m，则可不关注
  if (std::fabs(reference_line_info_->planning_target().stop_point().s() -
       obstacle_sl.end_s()) > 30) {
    return false;
  }
  return true;
}


std::vector<ObstacleEdge> PullOverBoundGenerator::SortObstaclesForSweepLine(
                          const IndexedList<std::string, Obstacle>& indexed_obstacles, 
                          const double& obstacle_lat_buffer, const LaneBound* path_boundaries) {
  AERROR<<"PullOverBoundGenerator::SortObstaclesForSweepLine";
  std::vector<ObstacleEdge> sorted_obstacles;
  const double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;

  AINFO_IF(FLAGS_enable_debug_motion)
    <<"plan_in_lane = "<<BehaviorParser::instance()->plan_in_lane()
    <<",  init v = "<<frame_->PlanningStartPoint().v();
  AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle_lat_buffer = "<<obstacle_lat_buffer;
  const auto speed_init_point = reference_line_info_->speed_init_point();
  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    double lat_buffer = obstacle_lat_buffer;
    if (!BehaviorParser::instance()->target_reference_line_change()) {//如果参考线没发生切换，都用历史的避障距离
      lat_buffer = std::fmin(obstacle_lat_buffer,obstacle->nudge_l_buffer_min());
    }
    // Only focus on those within-scope sttatic obstacles.
    if (!IsWithinPullOverPathDeciderScopeObstacle(*obstacle)) {
       continue;
    }

    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    double delta_s = 0.0;
    
    bool is_pull_over_dynamic_obstacle = false;
    if (!obstacle->IsStatic()) {//动态障碍物，区分靠边停车和换道
      is_pull_over_dynamic_obstacle = true;
      const double ego_speed_average_estimate = frame_->PlanningStartPoint().v();
      const double pull_over_time = 
          (fabs(BehaviorParser::instance()->pull_over_sl().front().l()) + fabs(adc_frenet_s_))
          / ego_speed_average_estimate ; 
      delta_s = std::max(obstacle->speed(),0.0) * pull_over_time;
      AINFO_IF(FLAGS_enable_debug_motion)<<"pull_over_time = "<<pull_over_time<<", delta_s = "<<delta_s;   
    }

   //判断是否要减小横向最小避障距离
    // double nudge_dis_available = 0.0;
    // bool is_shorten_nudge_buffer = false;//是否可以考虑减小避障最小横向距离要求
    // if ( obstacle_sl.start_s() <= FLAGS_extreme_nudge_obstacle_s_max 
    //   && obstacle_bound_process_ptr_->CheckStaticObstacleNudgeAvailable(*obstacle, path_boundaries, nudge_dis_available) 
    //   && nudge_dis_available < FLAGS_static_decision_nudge_l_buffer) {
    //     if (BehaviorParser::instance()->plan_in_lane()  
    //         && frame_->PlanningStartPoint().v() <= nudge_dis_available * 10) {
    //       is_shorten_nudge_buffer = true;
    //     }
    //     if (is_shorten_nudge_buffer) {
    //       AERROR_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()
    //         <<", set lat_buffer = "<<FLAGS_static_decision_nudge_l_buffer_min
    //         <<", l = ("<<obstacle_sl.start_l()<<", "<<obstacle_sl.end_l()<<").";
    //       reference_line_info_->path_decision()->SetNudgeBuffer(obstacle->Id(),FLAGS_static_decision_nudge_l_buffer_min);
    //       //改变限速；
    //       if (obstacle_sl.end_s() > 0.0) {
    //         double end_s = 1000.0;
            
    //         reference_line_info_->reference_line_ptr()->AddSpeedLimit(
    //             0.0, end_s, std::fmax(frame_->PlanningStartPoint().v(), nudge_dis_available * 10));
    //       }
    //     }
    //   //经过障碍物时需要减速通过，减到1.5m/s
    // }

    if (is_pull_over_dynamic_obstacle) {//靠边停车的特殊处理,经过IsWithinPullOverPathDeciderScopeObstacle筛选后的动态障碍物都需要经过这样的处理，
      double min_l = obstacle_sl.start_l() - fabs(obstacle_lat_buffer);
      double max_l = std::fmin(adc_frenet_l_ - adc_half_width - 0.1, obstacle_sl.end_l() + fabs(obstacle_lat_buffer));
      if (max_l <= min_l) {
        obstacle_sl.end_l() + fabs(obstacle_lat_buffer);
      }
      sorted_obstacles.emplace_back(1, obstacle_sl.start_s(), min_l, max_l, obstacle->Id());

      AINFO_IF(FLAGS_enable_debug_motion)<<"s = "<<std::get<1>(sorted_obstacles.back())
         <<", l = ("<<std::get<2>(sorted_obstacles.back())<<", "<<std::get<3>(sorted_obstacles.back())<<").";

      sorted_obstacles.emplace_back(0, obstacle_sl.end_s() + delta_s, min_l, max_l , obstacle->Id());
    } else {
      sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() + delta_s- FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l() - fabs(lat_buffer),
        obstacle_sl.end_l() + fabs(lat_buffer), 
        obstacle->Id());
      sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + delta_s + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l() - fabs(lat_buffer), 
        obstacle_sl.end_l() + fabs(lat_buffer), 
        obstacle->Id());
    }
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });

  return sorted_obstacles;
}

}  // namespace planning
}  // namespace acu
