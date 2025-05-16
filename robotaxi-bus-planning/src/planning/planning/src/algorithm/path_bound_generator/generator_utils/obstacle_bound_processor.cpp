/**
 * @file obstacle_bound_processor.cpp
 ObstatcleObstatcleBoundProcessor类，用于处理障碍物对bound的影响
 **/

#include "obstacle_bound_processor.h"


namespace acu {
namespace planning {

ObstacleBoundProcessor::ObstacleBoundProcessor(Frame* frame, ReferenceLineInfo* reference_line_info,
  const double& adc_frenet_s, const double& adc_frenet_l):
  frame_(frame),
  reference_line_info_(reference_line_info),
  adc_frenet_s_(adc_frenet_s),
  adc_frenet_l_(adc_frenet_l){
    bound_adjustment_ptr_ = std::make_shared<BoundAdjustment>();
  }


bool 
ObstacleBoundProcessor::IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
//   if (obstacle.IsVirtual()) {
//     return false;
//   }
  // Obstacle should not have ignore decision.
  if (obstacle.HasLongitudinalDecision() && obstacle.HasLateralDecision() &&
      obstacle.IsIgnore()) {
    return false;
  }

  if (!obstacle.IsStatic()
    && obstacle.BehaviorLongitudinalDecision() == eObjectDecisionEnum::SIDEPASS) {
    return true;
  }

  if (!reference_line_info_->IsChangeLanePath() && !obstacle.IsStatic()) {
    return false;
  }

  if (!obstacle.IsStatic() && 
      obstacle.conflict_type() == 1 && 
      (obstacle.Perception().type == 2 || obstacle.speed() < 5.0)) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"PEDESTRIAN id = "<<obstacle.Id()<<" take into scope";
    return true; 
  }

  // Obstacle should not be moving obstacle.(不高于5km/h)
  if (!obstacle.IsStatic() && obstacle.speed() > 5/3.6) {
    return false;
  }

  return true;
}


bool 
ObstacleBoundProcessor::GetBoundaryMin(const LaneBound* path_boundaries,const double& start_s, 
   const double& end_s, const int& obstacle_pos, std::pair<double, double>& boundary_min) const {
  if (end_s - start_s < 0 || path_boundaries->empty() 
    || obstacle_pos == 0 || obstacle_pos > 2) {
    AERROR_IF(FLAGS_enable_debug_motion)<<"search s error!!!";
    return false;
  }
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  double width = obstacle_pos == 2? 
   std::numeric_limits<double>::max() : std::numeric_limits<double>::lowest();
  double right_width = boundary_min.first;
  double left_width = boundary_min.second;
  const double s_resolution = 0.5;
  bool find_boundary_min = false;
  for (const auto& bound_tuple : *path_boundaries) {
    if (std::get<0>(bound_tuple) >= start_s 
       && std::get<0>(bound_tuple) <= end_s ) {
      if (obstacle_pos == 1) {//障碍物在左边，则找右侧边界最小值
        if (std::get<1>(bound_tuple) > width) {
          right_width = std::get<1>(bound_tuple);
          left_width = std::get<2>(bound_tuple);
          width = std::get<1>(bound_tuple);
          find_boundary_min = true;
        } 
      } else if (obstacle_pos == 2) {//障碍物在右边，则找左侧边界最小值
        if (std::get<2>(bound_tuple) < width) {
          right_width = std::get<1>(bound_tuple);
          left_width = std::get<2>(bound_tuple);
          width = std::get<2>(bound_tuple);
          find_boundary_min = true;
        } 
      }
      
    }
  }
  if (!find_boundary_min) {
    return false;
  }
  boundary_min = std::make_pair(right_width - adc_half_width, 
                                left_width + adc_half_width);
  
  return true;
}


bool 
ObstacleBoundProcessor::CheckStaticObstacleNudgeAvailable(const Obstacle& obstacle,
      const LaneBound* path_boundaries, double& nudge_dis_available) const {
  if (!obstacle.IsStatic()) {//不考虑动态障碍物
    AINFO_IF(FLAGS_enable_debug_motion)<<"id = "<<obstacle.Id();
    return false;
  }
   
  const auto& obstacle_sl = obstacle.PerceptionSLBoundary();
  bool is_parallel = false;
  if (obstacle_sl.start_s() * obstacle_sl.end_s() <= 0) {
    is_parallel = true;
  } else if (obstacle_sl.start_s() > 0 && obstacle_sl.start_s() < adc_frenet_s_ + 3.0) {
    is_parallel = true;
  }

  int obstacle_pos = 0;
  if (obstacle_sl.start_l() * obstacle_sl.end_l() > 0) {
    obstacle_pos = obstacle_sl.start_l() > 0 ? 1 : 2;//1:左边，2为右边
  }
  if (obstacle_pos == 0) return false;

  double check_start_s = obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer;
  double check_end_s = obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer;
  std::pair<double ,double> boundary_min = std::make_pair(-1.75,1.75);//first:右边界，负值，//second:左边界，正值
  if (!GetBoundaryMin(path_boundaries, check_start_s, check_end_s , obstacle_pos, boundary_min)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"GetBoundaryMin failed";
    return false;
  }
  switch (obstacle_pos) {
    case 1:
      nudge_dis_available = (obstacle_sl.start_l() - boundary_min.first)
                       - EgoInfo::instance()->vehicle_width();
      if (is_parallel) {
        nudge_dis_available = obstacle_sl.start_l() - (adc_frenet_l_ + EgoInfo::instance()->vehicle_width() / 2.0);  
      }
      AINFO_IF(FLAGS_enable_debug_motion)<<"is_parallel = "<<is_parallel
             <<", boundary min = ("<<boundary_min.first<<", "<< boundary_min.second<<").";                
      break;
    case 2:
      nudge_dis_available = (-obstacle_sl.end_l() + boundary_min.second)
                       - EgoInfo::instance()->vehicle_width();
      if (is_parallel) {
        nudge_dis_available = -obstacle_sl.end_l() + (adc_frenet_l_ - EgoInfo::instance()->vehicle_width() / 2.0);  
      }                 
      AINFO_IF(FLAGS_enable_debug_motion)<<"is_parallel = "<<is_parallel
             <<"boundary min = ("<<boundary_min.first<<", "<< boundary_min.second<<").";                    
      break;
    default:
      return false;
      AWARN_IF(FLAGS_enable_debug_motion) << "Unknown obstacle_pos:" << obstacle_pos;
  }

  AWARN_IF(FLAGS_enable_debug_motion)<<"nudge_dis_available = "<<nudge_dis_available;
  if (nudge_dis_available < FLAGS_static_decision_nudge_l_buffer_min) {
    return false;
  } 
  // AINFO_IF(FLAGS_enable_debug_motion)<<"return true";
  return true;
}


std::vector<ObstacleEdge> 
ObstacleBoundProcessor::SortObstaclesForSweepLine(
          const IndexedList<std::string, Obstacle>& indexed_obstacles, 
          const double& obstacle_lat_buffer, const LaneBound* path_boundaries) {
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
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"Is not Within PathDecider Scope Obstacle Id = "<< obstacle->Id();
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obstacle->PerceptionSLBoundary().end_s() < adc_frenet_s_) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"Is not Within PathDecider Scope Obstacle Id = "<< obstacle->Id();
      continue;
    }
    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->PerceptionSLBoundary();
    double delta_s = 0.0;
    
    if (!obstacle->IsStatic()) {
      double delta_v = speed_init_point.v() - obstacle->speed();
      double ttc = delta_v > 1e-3 ? (obstacle->PerceptionSLBoundary().start_s() 
        - EgoInfo::instance()->vehicle_front_edge_to_center()) / delta_v : (obstacle->PerceptionSLBoundary().start_s() 
        - EgoInfo::instance()->vehicle_front_edge_to_center()) / speed_init_point.v();
      double overlap_s = obstacle->STUpperLowerPoints().HasPoint() ?
        obstacle->STUpperLowerPoints().lower_points.front().s()
          - obstacle->PerceptionSLBoundary().start_s() : ttc*obstacle->speed();
      delta_s = std::max(0.0, overlap_s);
      double delta_s_origin = std::max(obstacle->speed() * 0.8,0.0) * 
                std::min(7.0,obstacle_sl.start_s() / frame_->PlanningStartPoint().v());
      delta_s_origin = std::min(20.0, delta_s_origin);
      AINFO_IF(FLAGS_enable_debug_motion)<<"obstacle id = "<<obstacle->Id()<<" speed "<<obstacle->speed()<<" ttc "<< ttc
        <<" delta_s = "<<delta_s<<" delta_s_origin "<<delta_s_origin<<" overlap_s_ttc "<<ttc*obstacle->speed()
        <<" has_overlap? "<<obstacle->STUpperLowerPoints().HasPoint();
      delta_s = delta_s_origin;
    }

    //判断是否要减小横向最小避障距离
    double nudge_dis_available = 0.0;
    bool is_shorten_nudge_buffer = false;//是否可以考虑减小避障最小横向距离要求
    if ( obstacle_sl.start_s() <= FLAGS_extreme_nudge_obstacle_s_max 
      && CheckStaticObstacleNudgeAvailable(*obstacle, path_boundaries, nudge_dis_available) 
      && nudge_dis_available < FLAGS_static_decision_nudge_l_buffer) {
        if (BehaviorParser::instance()->plan_in_lane() 
            && frame_->PlanningStartPoint().v() <= nudge_dis_available * 10) {
          is_shorten_nudge_buffer = true;
        }
        if (is_shorten_nudge_buffer) {
          AERROR_IF(FLAGS_enable_debug_motion)<<"No["<<obstacle->Id()
            <<", set lat_buffer = "<<FLAGS_static_decision_nudge_l_buffer_min
            <<", l = ("<<obstacle_sl.start_l()<<", "<<obstacle_sl.end_l()<<").";
          reference_line_info_->path_decision()->SetNudgeBuffer(obstacle->Id(),FLAGS_static_decision_nudge_l_buffer_min);
          //改变限速；
          if (obstacle_sl.end_s() > 0.0) {
            double end_s = 1000.0;
            
            reference_line_info_->reference_line_ptr()->AddSpeedLimit(
                0.0, end_s, std::fmax(frame_->PlanningStartPoint().v(), nudge_dis_available * 10));
          }
        }
      //经过障碍物时需要减速通过，减到1.5m/s
    }

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


bool 
ObstacleBoundProcessor::GetSoftBoundaryFromUncertainObstacles(
      const PathDecision* path_decision,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      const std::unordered_map<std::string, bool>& obs_id_to_direction,
      std::vector<std::pair<int, int>>* const boundary_type,
      std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary) {
  bool rtvalue = false;
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    std::tuple<double, double, double, double, std::string> a_soft_boundary(
        std::get<1>((*path_boundaries)[i]), 
        std::get<2>((*path_boundaries)[i]), 0, 0, "null");
    soft_boundary->push_back(a_soft_boundary);
  }
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  auto indexed_obstacles = path_decision->obstacles();
  for (const auto* obstacle : indexed_obstacles.Items()) {
    const auto perception_obstacle = obstacle->Perception();
    const auto obstacle_id = obstacle->Id();
    if (perception_obstacle.sl_polygons.empty()) {
      continue;
    }

    bool obj_avoid_direction = true;//left
    if(obs_id_to_direction.count(obstacle_id)){
      obj_avoid_direction = obs_id_to_direction.find(obstacle_id)->second;
    }else{
      continue;
    }
    for (auto& sl_polygon : perception_obstacle.sl_polygons) {
      double s_min = std::numeric_limits<double>::max();
      double s_max = 0;
      for (auto& sl_point : sl_polygon.points()) {
        if (sl_point.x() <= s_min){
          s_min = sl_point.x();
        }
        if(sl_point.x() >= s_max){
          s_max = sl_point.x();
        }
      }
      if(s_min >= s_max){
        AERROR << "polygon length smaller than 0 !!!!";
        continue;
      }
      double path_boundaries_start_s = 0;
      size_t path_boundaries_start_s_index = 0;
      double path_boundaries_end_s = 0;
      size_t path_boundaries_end_s_index = path_boundaries->size() - 1;
      for (size_t i = 1; i < path_boundaries->size(); ++i) {
        double s_temp_last = std::get<0>((*path_boundaries)[i-1]);
        double s_temp = std::get<0>((*path_boundaries)[i]);
        if (s_temp > s_min && s_temp_last <= s_min) {
          path_boundaries_start_s = s_temp_last;
          path_boundaries_start_s_index = i - 1;
        }
        if (s_temp > s_max && s_temp_last <= s_max) {
          path_boundaries_end_s = s_temp;
          path_boundaries_end_s_index = i;
          break;
        }
      }
      if (path_boundaries_start_s >= path_boundaries_end_s) {
        AERROR << "path_boundaries soft delt_l smaller than 0 !!!!";
        continue;
      }
      for (size_t i = path_boundaries_start_s_index; i <= path_boundaries_end_s_index; ++i) {
        double s_temp = std::get<0>((*path_boundaries)[i]);
        std::pair<double, double> polygon_l_range;//l_min,l_max
        double obj_occupy_delt_l = 0;
        sl_polygon.VerticalDistanceTo(s_temp, polygon_l_range);
        polygon_l_range.first -= adc_half_width;
        polygon_l_range.second += adc_half_width;
        obj_occupy_delt_l = std::fabs(polygon_l_range.second - polygon_l_range.first);
        std::pair<int, int> bound_type = std::make_pair(0,0);
        if (obj_avoid_direction) {//从左边避障
          if (polygon_l_range.second < std::get<0>(soft_boundary->at(i))) {
            continue;
          }
          std::get<0>(soft_boundary->at(i)) = std::fmax(std::get<0>(soft_boundary->at(i)), polygon_l_range.second);
          bound_type.first = 8;
          if((*boundary_type)[i].first != 7){
            bound_adjustment_ptr_->UpdateBoundaryType(i,bound_type.second,bound_type.first,boundary_type);
          }
        } else {//从右边避障
          if (polygon_l_range.first > std::get<0>(soft_boundary->at(i))) {
            continue;
          }
          std::get<1>(soft_boundary->at(i)) = std::fmin(std::get<1>(soft_boundary->at(i)), polygon_l_range.first);
          bound_type.second = 8;
          if ((*boundary_type)[i].second != 7){
            bound_adjustment_ptr_->UpdateBoundaryType(i,bound_type.second,bound_type.first,boundary_type);
          }
        }
        std::get<1>(soft_boundary->at(i)) = std::max(std::get<1>(soft_boundary->at(i)), std::get<1>((*path_boundaries)[i]));
        std::get<1>(soft_boundary->at(i)) = std::min(std::get<1>(soft_boundary->at(i)), std::get<2>((*path_boundaries)[i]));
        std::get<0>(soft_boundary->at(i)) = std::max(std::get<0>(soft_boundary->at(i)), std::get<1>((*path_boundaries)[i]));
        std::get<0>(soft_boundary->at(i)) = std::min(std::get<0>(soft_boundary->at(i)), std::get<2>((*path_boundaries)[i]));
        std::get<2>(soft_boundary->at(i)) = 1.0;//TBD 这个ploygon的概率
        std::get<3>(soft_boundary->at(i)) = obj_occupy_delt_l;
        std::get<4>(soft_boundary->at(i)) = obstacle_id;
      }
    }
  }

  const double dotted_line_min_delt_l = 0.2;
  const double solid_line_min_delt_l = 0.3;
  for (size_t i = 0; i < path_boundaries->size(); ++i) {
    if((*boundary_type)[i].first < 3){
      std::get<0>((*soft_boundary)[i]) = std::get<1>((*path_boundaries)[i]) + dotted_line_min_delt_l;
    }else if((*boundary_type)[i].first < 7){
      std::get<0>((*soft_boundary)[i]) = std::get<1>((*path_boundaries)[i]) + solid_line_min_delt_l;
    }
    if((*boundary_type)[i].second < 3){
      std::get<1>((*soft_boundary)[i]) = std::get<2>((*path_boundaries)[i]) - dotted_line_min_delt_l;
    }else if((*boundary_type)[i].second < 7){
      std::get<1>((*soft_boundary)[i]) = std::get<2>((*path_boundaries)[i]) - solid_line_min_delt_l;
    }
    // AINFO_IF(FLAGS_enable_debug_motion) << "i = " << i << " path_boundaries (" 
    //       << std::get<1>((*path_boundaries)[i]) << ", " << std::get<2>((*path_boundaries)[i])<<"). ";
    std::get<1>((*soft_boundary)[i]) = std::max(std::get<1>((*soft_boundary)[i]), std::get<1>((*path_boundaries)[i]));
    std::get<1>((*soft_boundary)[i]) = std::min(std::get<1>((*soft_boundary)[i]), std::get<2>((*path_boundaries)[i]));
    std::get<0>((*soft_boundary)[i]) = std::max(std::get<0>((*soft_boundary)[i]), std::get<1>((*path_boundaries)[i]));
    std::get<0>((*soft_boundary)[i]) = std::min(std::get<0>((*soft_boundary)[i]), std::get<2>((*path_boundaries)[i]));
  }

  // for (size_t i = 0; i < soft_boundary->size(); ++i) {
  //   AINFO_IF(FLAGS_enable_debug_motion) << "i = " << i  << " soft_boundary_ (" << std::get<0>(soft_boundary->at(i))
  //          << " , " << std::get<1>(soft_boundary->at(i))<<").";
  // }
  rtvalue = true;
  return rtvalue;
}

bool 
ObstacleBoundProcessor::GetDynamicObstacleBoxes(const uint32_t& num_of_time_stamps,
                const double& eval_time_interval, const double& total_time, 
                std::vector<std::vector<common::math::Box2d>>& dynamic_obstacle_boxes) {
  dynamic_obstacle_boxes.clear();
  if (reference_line_info_->speed_data().empty()) {
    AERROR_IF(FLAGS_enable_debug_motion)<<"There is no heuristic_speed !!";
    return true;
  }
  auto vehicle_param = EgoInfo::instance()->vehicle_param();
  auto ignore_s = EgoInfo::instance()->vehicle_front_edge_to_center();
  bool is_need_borrow_lane = reference_line_info_->GetLaneBorrowInfo() != LaneBorrowInfo::NO_BORROW;
  double ignore_s_max = // (is_need_borrow_lane ? 6:2) * // decide by THW
    6 * EgoInfo::instance()->vehicle_state_new().linear_velocity;
  const double adc_left_l =
      reference_line_info_->init_sl_point().l() + (vehicle_param.half_wheel); 
  const double adc_right_l =
      reference_line_info_->init_sl_point().l() - (vehicle_param.half_wheel);
  const double speed_buffer = 0.05;
  double ignore_speed = (1.0 + speed_buffer) * reference_line_info_->reference_line().cruise_speed();
  const std::string debug_obj_id = std::to_string(FLAGS_debug_id);
  for (const auto *ptr_obstacle : reference_line_info_->path_decision()->obstacles().Items()) {
    const bool is_side_pass = is_need_borrow_lane && 
      ptr_obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::SIDEPASS;
    if (ptr_obstacle->IsIgnore() 
      || ptr_obstacle->IsVirtual() 
      || ptr_obstacle->IsStatic()
      || ptr_obstacle->LongitudinalDecision().has_stop()) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries_ by reason 1";
      continue;
    } 
    const auto &sl_boundary = ptr_obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() <= ignore_s 
      || !is_side_pass && sl_boundary.start_s() >= ignore_s_max 
      || sl_boundary.start_l()*sl_boundary.end_l() < 0
      || ptr_obstacle->speed() >= ignore_speed) { 
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries_ by reason 2"
        <<" | "<<(sl_boundary.end_s() <= ignore_s)
        <<" | "<<(sl_boundary.start_s() >= ignore_s_max)<<" start_s "<<sl_boundary.start_s()<<" ignore_s_max "<<ignore_s_max
        <<" | "<<(sl_boundary.start_l()*sl_boundary.end_l() < 0)
        <<" | "<<(ptr_obstacle->speed() >= ignore_speed);
      continue;
    }
    double min_l = std::fmin(fabs(sl_boundary.start_l()),fabs(sl_boundary.end_l()));
    if (adc_left_l + FLAGS_dynamic_obstacle_lateral_ignore_buffer < sl_boundary.start_l() 
        || adc_right_l - FLAGS_dynamic_obstacle_lateral_ignore_buffer > sl_boundary.end_l()
        || !is_side_pass && (min_l <= vehicle_param.half_wheel && sl_boundary.start_l()*sl_boundary.end_l() >= 0)) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries_ by reason 3";
      continue;
    }
    // reserved other filter condition
    if (ptr_obstacle->Trajectory().trajectory_point().empty()) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries_ by reason 4";
      continue;
    }
    size_t back_index = ptr_obstacle->Trajectory().trajectory_point().size() - 1;
    double predition_time_max = ptr_obstacle->Trajectory().trajectory_point(back_index).relative_time();
    AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"id : "<<ptr_obstacle->Id()<<", prediction total time "<<predition_time_max;
    std::vector<Box2d> box_by_time;
    for (uint32_t t = 0; t <= num_of_time_stamps; ++t) {
      if (t * eval_time_interval > predition_time_max) {
        break;
      }
      TrajectoryPoint trajectory_point =
        ptr_obstacle->GetPointAtTime(t * eval_time_interval);
      Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
      constexpr double kBuff = 0.5;
      Box2d expanded_obstacle_box =
        Box2d(obstacle_box.center(), obstacle_box.heading(),
              obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
      box_by_time.push_back(expanded_obstacle_box);
    }
    
    dynamic_obstacle_boxes.push_back(std::move(box_by_time));
  }

  AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"dynamic_obstacle_boxes_.size = "<<dynamic_obstacle_boxes.size();

  return true;
}

bool 
ObstacleBoundProcessor::GetDynamicObstacleSLBoundaries(const uint32_t& num_of_time_stamps,
                const double& eval_time_interval, const double& total_time, 
                std::vector<std::vector<SLBoundary>>& dynamic_obstacle_sl_boundaries) {
  dynamic_obstacle_sl_boundaries.clear();
  if (reference_line_info_->speed_data().empty()) {
    AERROR_IF(FLAGS_enable_debug_motion)<<"There is no heuristic_speed !!";
    return true;
  }
  auto vehicle_param = EgoInfo::instance()->vehicle_param();
  auto ignore_s = EgoInfo::instance()->vehicle_front_edge_to_center();
  bool is_need_borrow_lane = reference_line_info_->GetLaneBorrowInfo() != LaneBorrowInfo::NO_BORROW;
  double ignore_s_max = // (is_need_borrow_lane ? 6:2) * // decide by THW
    6 * EgoInfo::instance()->vehicle_state_new().linear_velocity;
  const double adc_left_l =
      reference_line_info_->init_sl_point().l() + (vehicle_param.half_wheel); 
  const double adc_right_l =
      reference_line_info_->init_sl_point().l() - (vehicle_param.half_wheel);
  const double speed_buffer = 0.05;
  double ignore_speed = (1.0 + speed_buffer) * reference_line_info_->reference_line().cruise_speed();
  const std::string debug_obj_id = std::to_string(FLAGS_debug_id);
  for (const auto *ptr_obstacle : reference_line_info_->path_decision()->whole_obstacles().Items()) {
    const bool is_side_pass = is_need_borrow_lane && 
      ptr_obstacle->BehaviorLongitudinalDecision() == eObjectDecisionEnum::SIDEPASS;
    // AINFO_IF(FLAGS_enable_piecewise_path_debug) << "&&&Dynamic Obstacle Id :" << ptr_obstacle->Id();
    if (ptr_obstacle->IsIgnore() 
      || ptr_obstacle->IsVirtual() 
      || ptr_obstacle->IsStatic()
      || ptr_obstacle->LongitudinalDecision().has_stop()) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries by reason 1";
      continue;
    } 
    const auto &sl_boundary = ptr_obstacle->PerceptionSLBoundary();
    if (sl_boundary.end_s() <= ignore_s 
      || !is_side_pass && sl_boundary.start_s() >= ignore_s_max 
      || sl_boundary.start_l()*sl_boundary.end_l() < 0
      || ptr_obstacle->speed() >= ignore_speed) { 
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries by reason 2"
        <<" | "<<(sl_boundary.end_s() <= ignore_s)
        <<" | "<<(sl_boundary.start_s() >= ignore_s_max)<<" start_s "<<sl_boundary.start_s()<<" ignore_s_max "<<ignore_s_max
        <<" | "<<(sl_boundary.start_l()*sl_boundary.end_l() < 0)
        <<" | "<<(ptr_obstacle->speed() >= ignore_speed);
      continue;
    }
    double min_l = std::fmin(fabs(sl_boundary.start_l()),fabs(sl_boundary.end_l()));
    if (adc_left_l + FLAGS_dynamic_obstacle_lateral_ignore_buffer < sl_boundary.start_l() 
        || adc_right_l - FLAGS_dynamic_obstacle_lateral_ignore_buffer > sl_boundary.end_l()
        || !is_side_pass && (min_l <= vehicle_param.half_wheel && sl_boundary.start_l()*sl_boundary.end_l() >= 0)) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries by reason 3"
        <<" | "<<(adc_left_l + FLAGS_dynamic_obstacle_lateral_ignore_buffer < sl_boundary.start_l())
        <<" | "<<(adc_right_l - FLAGS_dynamic_obstacle_lateral_ignore_buffer > sl_boundary.end_l());
      continue;
    }
    // reserved other filter condition
    if (ptr_obstacle->Trajectory().trajectory_point().empty()) {
      AINFO_IF(FLAGS_enable_piecewise_path_debug && ptr_obstacle->Id() == debug_obj_id)
        <<"ignore create dynamic_obstacle_sl_boundaries by reason 4";
      continue;
    }
    
    std::vector<SLBoundary> sl_by_time;
    double delta_s = reference_line_info_->init_sl_point().s();

    const bool use_conservative_dynamic_boundary = true;
    double thw_buffer = std::numeric_limits<double>::max();
    double v = EgoInfo::instance()->vehicle_state_new().linear_velocity;
    if ( v > ptr_obstacle->Perception().vsabs + FLAGS_numerical_epsilon) {
      thw_buffer = (ptr_obstacle->PerceptionSLBoundary().end_s() - EgoInfo::instance()->vehicle_front_edge_to_center())
        / (v - ptr_obstacle->Perception().vsabs) + 2.0;
    }
    // 30km/h 取最小值
    // 70km/h 取最大值
    // lat*(1+(v-30)/40) 
    double lat_ratio = 1.0 + (v-8.3)/11.1;
    if (lat_ratio < 1.0) lat_ratio = 1.0;
    if (lat_ratio > 2.0) lat_ratio = 2.0;

    for (uint32_t t = 0; t <= num_of_time_stamps
        && (use_conservative_dynamic_boundary || t * eval_time_interval < thw_buffer); ++t) {
      double s_start = delta_s + ptr_obstacle->PerceptionSLBoundary().start_s() + t * eval_time_interval * ptr_obstacle->speed() ;
      double s_end = delta_s + ptr_obstacle->PerceptionSLBoundary().end_s() + t * eval_time_interval * ptr_obstacle->speed();
      double l_start = ptr_obstacle->PerceptionSLBoundary().start_l() /*- lat_ratio * FLAGS_dynamic_obstacle_lateral_buffer*/; 
      double l_end = ptr_obstacle->PerceptionSLBoundary().end_l() /*+ lat_ratio* FLAGS_dynamic_obstacle_lateral_buffer*/; 
      SLBoundary obstacle_sl;
      obstacle_sl.set_start_s(s_start);
      obstacle_sl.set_end_s(s_end);
      obstacle_sl.set_start_l(l_start);
      obstacle_sl.set_end_l(l_end);
      sl_by_time.push_back(obstacle_sl);
    }
      
    //dynamic_obstacle_sl_boundaries.push_back(std::move(sl_by_time));
  }

  AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"dynamic_obstacle_sl_boundaries.size = "<<dynamic_obstacle_sl_boundaries.size();

  return true;
}




}  // namespace planning
}  // namespace acu
