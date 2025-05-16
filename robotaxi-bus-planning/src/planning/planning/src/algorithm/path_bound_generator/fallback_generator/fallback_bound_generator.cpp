/**
 * @file procedure.cpp
 **/

#include "fallback_bound_generator.h"

namespace acu {
namespace planning {
using acu::common::Status;

FallbackBoundGenerator::FallbackBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
	   const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : PathBoundGenerator("RegularBoundGenerator", frame, reference_line_info, start_frenet_state) {}

common::Status FallbackBoundGenerator::Generate(std::vector<PathBoundary>& candidate_path_boundaries) {

  LaneBound path_bound;
  blocked_infos_.reset();
  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(&path_bound, &boundary_type_, &dynamic_bound_, &soft_boundary_)) {
    const std::string msg = "Failed to initialize fallback path boundaries.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  // 2. Decide a rough boundary based on decision info and ADC's position
  auto ret = GetBoundary(0.5, &path_bound, &boundary_type_, &soft_boundary_);
  if (!ret.ok()) {    
    const std::string msg =
        "GetBoundary failed : " + ret.error_message();
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }

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
  candidate_path_boundaries.emplace_back(std::get<0>(path_bound[0]),
                                         FLAGS_path_bounds_decider_resolution,
                                         path_bound_pair);
  candidate_path_boundaries.back().set_label("fallback");
  std::vector<std::pair<int, int>> bound_types(path_bound.size(),make_pair(0,0));
  candidate_path_boundaries.back().set_boundary_type(bound_types);
  candidate_path_boundaries.back().set_dynamic_obstacle_boundary(dynamic_obstacle_bound_pair);
  candidate_path_boundaries.back().set_is_blocked(blocked_infos_.is_blocked);
  candidate_path_boundaries.back().set_soft_boundary(soft_boundary_);
  AINFO_IF(FLAGS_enable_debug_motion) << "Completed generating fallback path boundaries.";
	return Status::OK();
}

}  // namespace planning
}  // namespace acu
