/**
 * @file procedure.cpp
 **/

#include "lite_coupling_generator.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"

namespace acu {
namespace planning {
using acu::common::Status;

LiteCouplingBoundGenerator::LiteCouplingBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
	const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : PathBoundGenerator("LiteCouplingBoundGenerator", frame, reference_line_info, start_frenet_state) {}


common::Status LiteCouplingBoundGenerator::Generate(std::vector<PathBoundary>& candidate_path_boundaries){
  enable_lite_coupling_ = (FLAGS_enable_lite_coupling_without_decision_meeting_ids 
                            || !BehaviorParser::instance()->decision_meeting_ids().empty()) && 
                               !BehaviorParser::instance()->decision_dynamic_obstacle_boxes().empty();
  if(!enable_lite_coupling_) {
    AWARN<<"Not Enable GetLiteCouplingBounds!";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "Not Enable GetLiteCouplingBounds.");
  }
  AWARN<<"GetLiteCouplingBounds";
  LaneBorrowInfo lane_borrow_info = LaneBorrowInfo::NO_BORROW;//TODO
  lane_borrow_info = reference_line_info_->GetLaneBorrowInfo();
  LaneBound regular_path_bound;
  std::string blocking_obstacle_id = "";//TODO
  std::string borrow_lane_type = "";//TODO
  Status ret = GenerateRegularLiteCouplingPathBound(
      lane_borrow_info, &regular_path_bound,
      &blocking_obstacle_id, &borrow_lane_type);
  if (!ret.ok()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Cannot generate a regular Lite-Coupling path bound.";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "Cannot generate a regular Lite-Coupling path bound.");
  }
  if (regular_path_bound.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Cannot generate a regular Lite-Coupling path bound.";
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "Cannot generate a regular Lite-Coupling path bound.");
  }
  if (adc_frenet_l_ > std::get<2>(regular_path_bound[0]) || 
       adc_frenet_l_ < std::get<1>(regular_path_bound[0])) {
    AERROR << "init point is out of path Lite-Coupling bound !!!!!!"<<adc_frenet_l_
            <<", "<<std::get<2>(regular_path_bound[0])<<", "<<std::get<1>(regular_path_bound[0]);
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "init point is out of path Lite-Coupling bound.");
  } 
  // Update the path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> regular_path_bound_pair;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_pair;
  double lane_change_length = frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer;
  lane_change_length = std::fmax(70.0, lane_change_length);
  double limited_length = std::fmin(lane_change_length + reference_line_info_->init_sl_point().s(),
                  reference_line_info_->reference_line().Length());
  AINFO_IF(FLAGS_enable_debug_motion)<<"regular Lite-Coupling path_bound = "<<regular_path_bound.size();
  for (size_t i = 0; i < regular_path_bound.size(); ++i) {
    double s = static_cast<double>(i) * FLAGS_path_bounds_decider_resolution +
               std::get<0>(regular_path_bound[0]);
    if (reference_line_info_->reference_line().GetMapPath().is_parking_path()
        && std::get<0>(regular_path_bound.back()) - s < EgoInfo::instance()->vehicle_front_edge_to_center() ) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"is_parking , set end 0 , 0";
      regular_path_bound_pair.emplace_back(0.0,0.0);
    } else {
      if (!BehaviorParser::instance()->plan_in_lane() && i >= regular_path_bound.size() - 1) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"borrow_lane , set 0 , 0";
        regular_path_bound_pair.emplace_back(0.0,0.0);
      } else if (s >= limited_length 
           && (FLAGS_enable_force_end_state_constraint 
           || BehaviorParser::instance()->CurrentLateralAction().first == eActionEnum::LANE_CHANGE)) {//TODO @pqg
        regular_path_bound_pair.emplace_back(0.0,0.0);
      } else {
        regular_path_bound_pair.emplace_back(std::get<1>(regular_path_bound[i]),
                                           std::get<2>(regular_path_bound[i]));
      }
    }
    dynamic_obstacle_bound_pair.emplace_back(std::get<1>(dynamic_bound_[i]),
                                         std::get<2>(dynamic_bound_[i]));   
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"regular_path_bound_pair = "<<regular_path_bound_pair.size();
  candidate_path_boundaries.emplace_back(std::get<0>(regular_path_bound[0]),
                                         FLAGS_path_bounds_decider_resolution,
                                         regular_path_bound_pair);
  std::string path_label = "";
  switch (lane_borrow_info) {
    case LaneBorrowInfo::LEFT_BORROW:
      path_label = "left";
      break;
    case LaneBorrowInfo::RIGHT_BORROW:
      path_label = "right";
      break;
    default:
      path_label = "self";
      break;
  }

  std::string label = "/lite_coupling";
  if(blocking_obstacle_id != ""){
    //如果有阻塞，说明此考虑了横纵向耦合动态障碍物的边界没有合适的解
    //为了增加对于动态障碍物的偏移，将标签复用为本该下回合使用的的标签，
    //使得piecewise-jerk在使用第一个边界规划时，就增大对于动态障碍物的权重
    label = "/has_litecoupling";
    AERROR<<"using has_litecoupling in the first round: "<<label;
  }

  candidate_path_boundaries.back().set_label(
      common::util::StrCat("regular/", path_label, "/", borrow_lane_type, label));
  candidate_path_boundaries.back().set_blocking_obstacle_id(blocking_obstacle_id);
  candidate_path_boundaries.back().set_boundary_type(boundary_type_);
  candidate_path_boundaries.back().set_dynamic_obstacle_boundary(dynamic_obstacle_bound_pair);
  // candidate_path_boundaries.back().set_is_blocked(blocked_infos_.is_blocked);
  candidate_path_boundaries.back().set_is_blocked(false);
  candidate_path_boundaries.back().set_soft_boundary(soft_boundary_);
  return Status::OK();
}


Status LiteCouplingBoundGenerator::GenerateRegularLiteCouplingPathBound(
    const LaneBorrowInfo& lane_borrow_info, LaneBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type) {
  boundary_type_.clear();
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(path_bound, &boundary_type_, &dynamic_bound_, &soft_boundary_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  // 2. Decide a rough boundary based on lane info and ADC's position
  int path_blocked_idx = -1;
  if (reference_line_info_->DecisionPathBound().empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"decision bound is empty.";
    if (!GetBoundaryFromLanesAndADC(lane_borrow_info, 0.1,
                                    path_bound, borrow_lane_type, path_blocked_idx)) {
      const std::string msg = 
          "Failed to decide a rough boundary based on "
          "road information.";
      AERROR_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
  } else {
    auto ret = GetBoundary(0.1, path_bound, &boundary_type_, &soft_boundary_); 
    if (!ret.ok()) {   
      const std::string msg =
        "GetBoundary failed : " + ret.error_message();
      AERROR_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
  }
  
  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(*path_bound, (int)__LINE__);
  }
  const double kBorrowOffset = 0.5;
  for (auto& bound:*path_bound) {
    if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
      get<2>(bound) = get<2>(bound) + kBorrowOffset;
    } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
      get<1>(bound) = get<1>(bound) - kBorrowOffset;
    }
  }
  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(*path_bound, (int)__LINE__);
  }

  if(enable_lite_coupling_){
    //简化版横纵向耦合动态障碍物会车拓宽边界
    const double kDynamicBoxesOffset = BehaviorParser::instance()->decision_expand_l();
    if(kDynamicBoxesOffset > 0.0 && kDynamicBoxesOffset < 1e3){
      if(boundary_type_.size() != path_bound->size()){
        AERROR<<"Invalid type size: "<<boundary_type_.size()<<" != "<<path_bound->size();
      }else{
        for(int i = 0; i < path_bound->size(); ++i){
          // if(boundary_type_.at(i).first < 3){
          //   get<2>((*path_bound).at(i)) = get<2>((*path_bound).at(i)) + kDynamicBoxesOffset;
          // }
          //会车只拓展右边界
          if(boundary_type_.at(i).first < 3){
            get<1>((*path_bound).at(i)) = get<1>((*path_bound).at(i)) - kDynamicBoxesOffset;
          }
        }
      }
    }
  }

  // 3. Fine-tune the boundary based on static obstacles
  double obstacle_lat_buffer = FLAGS_obstacle_lat_buffer;
  LaneBound temp_path_bound = *path_bound;
  if (!GetBoundaryFromStaticObstacles(reference_line_info_->path_decision(),
                                  path_bound, blocking_obstacle_id,
                                  obstacle_lat_buffer, &boundary_type_, &soft_boundary_)){
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(*path_bound, (int)__LINE__);
  }

  if(enable_lite_coupling_){
    AWARN<<"Enable dynamic_sl_generation!";
    if (!GetBoundaryFromDecisionDynamicBoxObstacles(path_bound, blocking_obstacle_id)) {
      const std::string msg =
          "Failed to decide fine tune the boundaries after "
          "taking into consideration all decision boxes obstacles.";
      AERROR_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }

    if (FLAGS_enable_piecewise_path_debug) {
      bound_adjustment_ptr_->PathBoundsDebugString(*path_bound, (int)__LINE__);
    }
  }
  
  GetFreeSpaceBoundary(boundary_type_, path_bound, path_blocked_idx);
  bool is_blocked = path_blocked_idx == -1 ? false : true;
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while ((!blocking_obstacle_id->empty() || is_blocked) &&
         path_bound->size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    // path_bound->push_back(temp_path_bound[path_bound->size()]);
    const double extra_right_bound = std::fmax(std::get<1>(temp_path_bound[path_bound->size()]), std::get<1>(path_bound->back()));
    const double extra_left_bound = std::fmin(std::get<2>(temp_path_bound[path_bound->size()]), std::get<2>(path_bound->back()));
    if (extra_right_bound > extra_left_bound) {
      break;
    } else {
      path_bound->emplace_back(std::get<0>(temp_path_bound[path_bound->size()]), 
                               extra_right_bound, 
                               extra_left_bound );
      AINFO_IF(FLAGS_enable_debug_motion)<<"back bound s = "<<std::get<0>(path_bound->back());
      boundary_type_.push_back(make_pair(0,0));
      counter++;
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"back bound s = "<<std::get<0>(path_bound->back());
    int soft_boundary_size = soft_boundary_.size();
    std::tuple<double, double, double, double, std::string> a_soft_boundary(
        std::get<0>(soft_boundary_[soft_boundary_size - 1]) , std::get<1>(soft_boundary_[soft_boundary_size - 1]), 
        std::get<2>(soft_boundary_[soft_boundary_size - 1]), std::get<3>(soft_boundary_[soft_boundary_size - 1]), 
        std::get<4>(soft_boundary_[soft_boundary_size - 1]));
    if(std::get<1>(a_soft_boundary) - std::get<0>(a_soft_boundary) <= 0){
      double extend_boundary_length = std::fabs(std::get<1>(a_soft_boundary) - std::get<0>(a_soft_boundary));
      std::get<1>(a_soft_boundary) = std::get<1>(a_soft_boundary) + extend_boundary_length / 2 + 0.01;
      std::get<0>(a_soft_boundary) = std::get<0>(a_soft_boundary) - extend_boundary_length / 2 - 0.01;
    }
    soft_boundary_.push_back(a_soft_boundary);
    counter++;
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(*path_bound, (int)__LINE__);
  }

  // 4. Adjust the boundary considering dynamic obstacles (TODO)
  CalculateDynamicObstacleBound(&dynamic_bound_);
  bound_adjustment_ptr_->PathBoundsDebugString(dynamic_bound_, (int)__LINE__);
  AINFO_IF(FLAGS_enable_debug_motion) << "Completed generating path boundaries. size = "<<path_bound->size();
  return Status::OK();
}

//利用决策环节得到的简化版横纵向耦合动态障碍物结果，修改硬约束边界
bool LiteCouplingBoundGenerator::GetBoundaryFromDecisionDynamicBoxObstacles(
    LaneBound* const path_boundaries,
    std::string* const blocking_obstacle_id) {
  // Preprocessing.
  std::vector<std::pair<int, int>> bound_types(path_boundaries->size(),make_pair(0,0));
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  auto sorted_obstacles = SortBehaviorDynamicBoxes();
  AINFO_IF(FLAGS_enable_debug_motion) << "There are " << sorted_obstacles.size() << " dynamic boxes.";// perception static obstacle size * 2
  if(sorted_obstacles.empty()) {return false;}
  double center_line = adc_frenet_l_;
  std::vector<std::pair<double, bool>> center_line_vec(1, std::make_pair(center_line, false));
  size_t obs_idx = 0;
  int path_blocked_idx = -1;
  std::multiset<double, std::greater<double>> right_bounds;//从大到小排序
  right_bounds.insert(std::numeric_limits<double>::lowest());
  std::multiset<double> left_bounds;//默认从小到大排序
  left_bounds.insert(std::numeric_limits<double>::max());
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    bound_types.at(i) = bound_types.at(i - 1);
    bound_adjustment_ptr_->UpdateBoundaryType(i, bound_types.at(i).second,bound_types.at(i).first,&boundary_type_);
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
    // The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        if (std::get<0>(curr_obstacle) == 1) {
          // A new obstacle enters into our scope:
          //   - Decide which direction for the ADC to pass.
          //   - Update the left/right bound accordingly.
          //   - If boundaries blocked, then decide whether can side-pass.
          //   - If yes, then borrow neighbor lane to side-pass.
          if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
            // Obstacle is to the right of center-line, should pass from left.
            if (std::get<1>((*path_boundaries)[i]) < curr_obstacle_l_max + adc_half_width) {
              bound_types.at(i).first = 7;
              bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first,&boundary_type_);
            }
            
            AINFO_IF(FLAGS_enable_debug_motion)<<"should pass from left, id = "<<curr_obstacle_id;
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
            if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              *blocking_obstacle_id = curr_obstacle_id;
              break;
            }
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            if (std::get<2>((*path_boundaries)[i]) > curr_obstacle_l_min - adc_half_width) {
              bound_types.at(i).second = 7;
              bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first,&boundary_type_);
            }

            AINFO_IF(FLAGS_enable_debug_motion)<<"should pass from right, id "<<curr_obstacle_id;
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);


            if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
                    i, *left_bounds.begin(), *right_bounds.begin(),
                    path_boundaries, &center_line)) {
              path_blocked_idx = static_cast<int>(i);
              *blocking_obstacle_id = curr_obstacle_id;
              break;
            }
          }
        } else {
          // An existing obstacle exits our scope.
          bound_types.at(i).first = 0;
          bound_types.at(i).second = 0;
          bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first,&boundary_type_);

          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);

          if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
                  i, *left_bounds.begin(), *right_bounds.begin(),
                  path_boundaries, &center_line)) {
            path_blocked_idx = static_cast<int>(i);
            if (!obs_id_to_direction.empty()) {
              *blocking_obstacle_id = obs_id_to_direction.begin()->first;
            }
            break;
          } else {
            bool is_obstacle = false, is_worse_option = false;
            bound_adjustment_ptr_->UpdateCenterLineVec(center_line_vec, center_line, is_obstacle, is_worse_option);
          }
        }
        ++obs_idx;
      }//@while loop
    } else {
      if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
              i, *left_bounds.begin(), *right_bounds.begin(),
              path_boundaries, &center_line)) {
        path_blocked_idx = static_cast<int>(i);
        if (!obs_id_to_direction.empty()) {
          *blocking_obstacle_id = obs_id_to_direction.begin()->first;
        }
      } else {
        bool is_obstacle = false, is_worse_option = false;
        bound_adjustment_ptr_->UpdateCenterLineVec(center_line_vec, center_line, is_obstacle, is_worse_option);
      }
    }

    // Early exit if path is blocked.
    if (path_blocked_idx != -1) {
      if(path_blocked_idx > 0){
        std::get<1>((*path_boundaries)[path_blocked_idx]) = std::get<1>((*path_boundaries)[path_blocked_idx - 1]);
        std::get<2>((*path_boundaries)[path_blocked_idx]) = std::get<2>((*path_boundaries)[path_blocked_idx - 1]);
      }
      break;
    }
  }
  // TrimPathBounds(path_blocked_idx, path_boundaries);
  // TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
  // TrimPathBoundTypes(path_blocked_idx,&boundary_type_);
  // if (path_blocked_idx != -1) {
  //   blocked_infos_.is_blocked = true;
  //   blocked_infos_.index = path_blocked_idx;
  //   blocked_infos_.s = std::get<0>((*path_boundaries)[path_blocked_idx]);
  //   int end_index = std::max(0, path_blocked_idx - 1);
  // }
  return true;
}


//给决策环节得到的动态障碍物排序，按照S由小至大的顺序
std::vector<ObstacleEdge> LiteCouplingBoundGenerator::SortBehaviorDynamicBoxes(){
  std::vector<ObstacleEdge> rtvalue;
  const std::vector<Box2d> dynamic_boxes = BehaviorParser::instance()->decision_dynamic_obstacle_boxes();
  if(dynamic_boxes.empty()) {
    AERROR<<"dynamic_boxes is empty";
    return rtvalue;
  }
  int dynamic_box_id = 0;
  const double lat_buffer = FLAGS_obstacle_lat_buffer;
  for(const auto& box : dynamic_boxes){
    dynamic_box_id++;
    SLBoundary sl_boundary;
    if(!reference_line_info_->reference_line().GetSLBoundary(box, &sl_boundary)) {
      AERROR_IF(FLAGS_enable_debug_motion)<<"GetSLBoundary failed";
      continue;
    }
    //一个障碍物变成两个ObstacleEdge（起终点各一个）
    std::string obs_id = "dynamic_box_" + std::to_string(dynamic_box_id);
    rtvalue.emplace_back(
        1, sl_boundary.start_s(),
        sl_boundary.start_l() - fabs(lat_buffer),
        sl_boundary.end_l() + fabs(lat_buffer), 
        obs_id);
    rtvalue.emplace_back(
        0, sl_boundary.end_s(),
        sl_boundary.start_l() - fabs(lat_buffer), 
        sl_boundary.end_l() + fabs(lat_buffer), 
        obs_id);
  }
    // Sort.
  std::sort(rtvalue.begin(), rtvalue.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });
  return rtvalue;
}


}  // namespace planning
}  // namespace acu
