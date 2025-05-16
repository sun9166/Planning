/**
 * @file procedure.cpp
 **/

#include "regular_bound_generator.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"

namespace acu {
namespace planning {
using acu::common::Status;

RegularBoundGenerator::RegularBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
	const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : PathBoundGenerator("RegularBoundGenerator", frame, reference_line_info, start_frenet_state) {}

common::Status RegularBoundGenerator::Generate(std::vector<PathBoundary>& candidate_path_boundaries) {

  LaneBorrowInfo lane_borrow_info = LaneBorrowInfo::NO_BORROW;//TODO
  lane_borrow_info = reference_line_info_->GetLaneBorrowInfo();
  LaneBound path_bound;
  blocked_infos_.reset();
  std::string blocking_obstacle_id = "";
  std::string borrow_lane_type = "";

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!InitPathBoundary(&path_bound, &boundary_type_, &dynamic_bound_, &soft_boundary_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }

  // 2. Decide a rough boundary based on lane info and ADC's position
  if (reference_line_info_->DecisionPathBound().empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"decision bound is empty.";
    int path_blocked_idx = -1;
    if (!GetBoundaryFromLanesAndADC(lane_borrow_info, 0.1,
                                    &path_bound, &borrow_lane_type, path_blocked_idx)) {
      const std::string msg =
          "Failed to decide a rough boundary based on "
          "road information.";
      AERROR_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
    if (path_blocked_idx != -1) {
      bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
      bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
    }
  } else {
    auto ret = GetBoundary(0.1, &path_bound, &boundary_type_, &soft_boundary_); 
    if (!ret.ok()) {   
      const std::string msg =
        "GetBoundary failed : " + ret.error_message();
      AERROR_IF(FLAGS_enable_debug_motion) << msg;
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
    }
  }
  
  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }
  const double kBorrowOffset = 0.5;
  for (auto& bound : path_bound) {
    if (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW) {
      get<2>(bound) = get<2>(bound) + kBorrowOffset;
    } else if (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW) {
      get<1>(bound) = get<1>(bound) - kBorrowOffset;
    }
  }
  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }

  // 3. Fine-tune the boundary based on static obstacles
  LaneBound temp_path_bound = path_bound;
  bool use_quasi_potential_field_path_optimizer = reference_line_info_->HasUncertainObstacles();//TBD ZT
  double obstacle_lat_buffer = FLAGS_obstacle_lat_buffer;
  if (use_quasi_potential_field_path_optimizer == true) {
    obstacle_lat_buffer +=0.2;
  }
  if (!GetBoundaryFromStaticObstacles(reference_line_info_->path_decision(),
                                      &path_bound, &blocking_obstacle_id,
                                      obstacle_lat_buffer, &boundary_type_, &soft_boundary_)) {
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
  bool is_blocked = path_blocked_idx == -1 ? false : true;
  if (is_blocked) {
  	bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
    bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  }
  // Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while ((!blocking_obstacle_id.empty() || is_blocked) &&
         path_bound.size() < temp_path_bound.size() &&
         counter < kNumExtraTailBoundPoint) {
    // path_bound.push_back(temp_path_bound[path_bound.size()]);
    path_bound.emplace_back(std::get<0>(temp_path_bound[path_bound.size()]), 
                   std::get<1>(path_bound.back()), std::get<2>(path_bound.back()));
    AINFO_IF(FLAGS_enable_debug_motion)<<"back bound s = "<<std::get<0>(path_bound.back());
    boundary_type_.push_back(make_pair(0,0));
	  int soft_boundary_size = soft_boundary_.size();
    if (soft_boundary_size > 0) {
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
    }
    counter++;
  }

  if (FLAGS_enable_piecewise_path_debug) {
    bound_adjustment_ptr_->PathBoundsDebugString(path_bound, (int)__LINE__);
  }

  // 4. Adjust the boundary considering dynamic obstacles (TODO)
  CalculateDynamicObstacleBound(&dynamic_bound_);
  // bound_adjustment_ptr_->PathBoundsDebugString(dynamic_bound_, (int)__LINE__);

  AINFO_IF(FLAGS_enable_debug_motion) << "Completed generating path boundaries. size = "<<path_bound.size();
  

  double lane_change_length = frame_->PlanningStartPoint().v() * FLAGS_lane_change_time_buffer;
  lane_change_length = std::fmax(70.0, lane_change_length);
  double limited_length = std::fmin(lane_change_length + reference_line_info_->init_sl_point().s(),
                  reference_line_info_->reference_line().Length());
  AINFO_IF(FLAGS_enable_debug_motion)<<"path_bound = "<<path_bound.size();
  for (size_t i = 0; i < path_bound.size(); ++i) {
    double s = static_cast<double>(i) * FLAGS_path_bounds_decider_resolution +
               std::get<0>(path_bound[0]);
    if (reference_line_info_->reference_line().GetMapPath().is_parking_path()
        && std::get<0>(path_bound.back()) - s < EgoInfo::instance()->vehicle_front_edge_to_center() ) {
      // 泊车路的延长路段让边界为0，但这个是local_lane ?
      AINFO_IF(FLAGS_enable_debug_motion)<<"is_parking , set end 0 , 0";
      std::get<1>(path_bound[i]) = 0.0;
      std::get<2>(path_bound[i]) = 0.0;
    } else {
      if (!BehaviorParser::instance()->plan_in_lane() && i >= path_bound.size() - 1) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"borrow_lane , set 0 , 0";
        std::get<1>(path_bound[i]) = 0.0;
        std::get<2>(path_bound[i]) = 0.0;
      } else if (s >= limited_length 
           && (FLAGS_enable_force_end_state_constraint 
           || BehaviorParser::instance()->CurrentLateralAction().first == eActionEnum::LANE_CHANGE)) {//TODO @pqg
        std::get<1>(path_bound[i]) = 0.0;
        std::get<2>(path_bound[i]) = 0.0;
      }
    }  
  }

  if (path_bound.empty()) {
    const std::string msg = "Failed to get a valid fallback path boundary";
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }
  if (!path_bound.empty()) {
    if (adc_frenet_l_ > (std::get<2>(path_bound[0]) + 0.5) || 
       adc_frenet_l_ < (std::get<1>(path_bound[0]) - 0.5))  {
      AERROR << "init point is out of path bound !!!!!!";
      return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, "init point is out of path bound .");
    } 
  }

  std::string has_lite_coupling_path_label = "";
  if(!candidate_path_boundaries.empty()){
    AWARN<<"candidate_path_boundaries is not empty, maybe already has a lite-coupling boundary?";
    for (const auto& path_boundary : candidate_path_boundaries) {
      if (path_boundary.label().find("lite_coupling") != std::string::npos) {
        has_lite_coupling_path_label = "/has_litecoupling";
        AWARN<<"set has_lite_coupling_path_label: "<<has_lite_coupling_path_label;
      }
    }
  }

  // Update the path boundary into the reference_line_info.
  std::vector<std::pair<double, double>> path_bound_pair;
  std::vector<std::pair<double, double>> dynamic_obstacle_bound_pair;
  
  for (size_t i = 0; i < path_bound.size(); ++i) {
    path_bound_pair.emplace_back(std::get<1>(path_bound[i]),
                                           std::get<2>(path_bound[i]));
    dynamic_obstacle_bound_pair.emplace_back(std::get<1>(dynamic_bound_[i]),
                                            std::get<2>(dynamic_bound_[i]));   
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"path_bound_pair = "<<path_bound_pair.size();
  candidate_path_boundaries.emplace_back(std::get<0>(path_bound[0]),
                                         FLAGS_path_bounds_decider_resolution,
                                         path_bound_pair);
  
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
  // RecordDebugInfo(path_bound, "", reference_line_info);
  candidate_path_boundaries.back().set_label(
      common::util::StrCat("regular/", path_label, "/", borrow_lane_type, has_lite_coupling_path_label));
  candidate_path_boundaries.back().set_blocking_obstacle_id(blocking_obstacle_id);
  candidate_path_boundaries.back().set_boundary_type(boundary_type_);
  candidate_path_boundaries.back().set_dynamic_obstacle_boundary(dynamic_obstacle_bound_pair);
  candidate_path_boundaries.back().set_is_blocked(blocked_infos_.is_blocked);
  candidate_path_boundaries.back().set_soft_boundary(soft_boundary_);
  AINFO_IF(FLAGS_enable_debug_motion) << "Completed regular and fallback path boundaries generation.";
	return Status::OK();
}

}  // namespace planning
}  // namespace acu
