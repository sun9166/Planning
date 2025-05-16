/**
 * @file procedure.cpp
 **/

#include "lane_change_bound_generator.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"

namespace acu {
namespace planning {
using acu::common::Status;

LaneChangeBoundGenerator::LaneChangeBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
	   const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : PathBoundGenerator("RegularBoundGenerator", frame, reference_line_info, start_frenet_state) {}

common::Status LaneChangeBoundGenerator::Generate(std::vector<PathBoundary>& candidate_path_boundaries) {

  LaneBound path_bound;
  blocked_infos_.reset();
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(&path_bound, &boundary_type_, &dynamic_bound_, &soft_boundary_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);  
  }

  // 2. Decide a rough boundary based on lane info and ADC's position
  auto ret = GetBoundary(0.1, &path_bound, &boundary_type_, &soft_boundary_);
  if (!ret.ok()) {    
    const std::string msg =
        "GetBoundary failed : " + ret.error_message();
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }

  LaneBound temp_path_bound = path_bound;
  std::string blocking_obstacle_id;
  if (!GetBoundaryFromStaticObstacles(reference_line_info_->path_decision(),
                                      &path_bound, &blocking_obstacle_id , 
                                      FLAGS_obstacle_lat_buffer_max, &boundary_type_, &soft_boundary_)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }
  
  int path_blocked_idx = -1;
  GetFreeSpaceBoundary(boundary_type_, &path_bound, path_blocked_idx);
  // Append some extra path bound points to avoid zero-length path data.
  bool is_blocked = path_blocked_idx == -1 ? false : true;
  if (is_blocked) {
  	bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
    bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  }
  
  int counter = 0;
  //路径拥堵的情况下，尾部再扩充20个点
  while ((!blocking_obstacle_id.empty() || is_blocked) &&
         path_bound.size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    path_bound.emplace_back(std::get<0>(temp_path_bound[path_bound.size()]), 
           std::get<1>(path_bound.back()), std::get<2>(path_bound.back()));
    AINFO_IF(FLAGS_enable_debug_motion)<<"back bound s = "<<std::get<0>(path_bound.back());
    boundary_type_.push_back(make_pair(0,0));

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

  //special prccess 
  // double characteristic_distance = numeric_limits<double>::max();
  // const auto characteristic_obstacle_id = BehaviorParser::instance()->characteristic_obstacle_id();
  // if (characteristic_obstacle_id > 0) {//for congestion lane change
  //   auto obstacle = reference_line_info_->path_decision()->Find(std::to_string(characteristic_obstacle_id)); 
  //   double estimate_accel = -0.8;
  //   if (obstacle != nullptr) {
  //     characteristic_distance = obstacle->PerceptionSLBoundary().start_s() 
  //                   + 0.5 * obstacle->speed() *obstacle->speed() / fabs(estimate_accel);
  //     characteristic_distance = std::fmax(characteristic_distance, 5.0); 
  //     AINFO_IF(FLAGS_enable_debug_motion)<<"characteristic_distance = "<<characteristic_distance;            
  //   }
  // }
  // double lane_change_length = frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer;
  // lane_change_length = std::fmax(70.0, lane_change_length);
  // double limited_length = std::fmin(lane_change_length + reference_line_info_->init_sl_point().s(),
  //                   reference_line_info_->reference_line().Length());
  // for (size_t i = 0; i < path_bound.size(); ++i) {
  //   double s = static_cast<double>(i) * FLAGS_path_bounds_decider_resolution +
  //              std::get<0>(path_bound[0]); 
  //   if (i >= path_bound.size() - 1) {
  //   	std::get<1>(path_bound[i]) = -0.05;
  //   	std::get<2>(path_bound[i]) = 0.05;
  //   } else if ((s >= limited_length && FLAGS_enable_force_end_state_constraint)
  //       || s >= characteristic_distance) {//TODO @pqg
  //     std::get<1>(path_bound[i]) = 0.0;
  //   	std::get<2>(path_bound[i]) = 0.0;
  //   }
  // }

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
  // Update the fallback path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> path_bound_pair;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_pair;
  for (size_t i = 0; i < path_bound.size(); ++i) {
    path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
                                            std::get<2>(path_bound[i]));
    dynamic_obstacle_bound_pair.emplace_back(std::get<1>(dynamic_bound_[i]),
                                            std::get<2>(dynamic_bound_[i]));            
  }
  candidate_path_boundaries.emplace_back(
               std::get<0>(path_bound[0]), FLAGS_path_bounds_decider_resolution,
               path_bound_pair);
  candidate_path_boundaries.back().set_label("regular/lanechange");
  candidate_path_boundaries.back().set_boundary_type(boundary_type_);
  candidate_path_boundaries.back().set_dynamic_obstacle_boundary(dynamic_obstacle_bound_pair);
  candidate_path_boundaries.back().set_is_blocked(blocked_infos_.is_blocked);
  candidate_path_boundaries.back().set_soft_boundary(soft_boundary_);
  // RecordDebugInfo(path_bound, "", reference_line_info);
  AINFO_IF(FLAGS_enable_debug_motion) << "Completed lanechange and fallback path boundaries generation.";
  AINFO_IF(FLAGS_enable_debug_motion) << "Completed generating path boundaries.";
  return Status::OK();
}
}  // namespace planning
}  // namespace acu
