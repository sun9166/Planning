/**
 * @file procedure.cpp
 **/

#include "path_bound_generator.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "../generator_utils/decision_path_bound_provider.h"

namespace acu {
namespace planning {
using acu::common::Status;

PathBoundGenerator::PathBoundGenerator(const std::string& name, Frame* frame, ReferenceLineInfo* reference_line_info, 
     const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state) 
     : name_(name),
       frame_(frame),
       reference_line_info_(reference_line_info),
       adc_frenet_s_(start_frenet_state.first[0]),
       adc_frenet_l_(start_frenet_state.second[0]),
       adc_frenet_sd_(start_frenet_state.first[1]),
       adc_frenet_ld_(start_frenet_state.second[1] * adc_frenet_sd_) {
  bound_adjustment_ptr_ = std::make_shared<BoundAdjustment>();
  obstacle_bound_process_ptr_ = std::make_shared<ObstacleBoundProcessor>(frame, reference_line_info, 
                                                                         adc_frenet_s_, adc_frenet_l_);
 }

const std::string& 
PathBoundGenerator::Name() const { return name_; }

bool 
PathBoundGenerator::InitPathBoundary(LaneBound* const path_bound,
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
  for (double curr_s = adc_frenet_s_;
       curr_s < std::fmin(adc_frenet_s_ +
                              std::fmax(kPathBoundsDeciderHorizon,
                                        reference_line_info_->reference_line().cruise_speed() * //TODO
                                            FLAGS_trajectory_time_length),
                          reference_line.Length());
       curr_s += FLAGS_path_bounds_decider_resolution) {
    if (reference_line_info_->LaterAction().first == eActionEnum::PULL_OVER 
       && !BehaviorParser::instance()->has_pull_over_request()) {
      if (curr_s > reference_line_info_->planning_target().stop_point().s() 
         + EgoInfo::instance()->vehicle_front_edge_to_center()) {
         break;
      }
    }
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
    
    boundary_type->emplace_back(make_pair(0,0));
  }

  if (path_bound->empty()) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Empty path boundary in InitPathBoundary";
    return false;
  }
  return true;
}



bool 
PathBoundGenerator::GetSolidLineStartS(const int& search_direction, double& solid_line_start_s) {
  vector<pair<double, int> > bd_types = search_direction == 1 ? 
        reference_line_info_->reference_line().GetMapPath().raw_refrence_line()->mapinfo.left_bd_types
        : reference_line_info_->reference_line().GetMapPath().raw_refrence_line()->mapinfo.right_bd_types;
  
  if (bd_types.empty()) {
    return false;
  }
  for (size_t i = 0;  i < bd_types.size(); ++i) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"s = "<<bd_types.at(i).first<<", type = "<<bd_types.at(i).second;
    if (bd_types.at(i).second > 2 ) {
      if (i > 0) {
        solid_line_start_s = bd_types.at(i - 1).first - EgoInfo::instance()->vehicle_front_edge_to_center();
      } else {
        solid_line_start_s = 0;
      }
      return true;
    }
  }
  return false;
}

bool 
PathBoundGenerator::IsInRoadBoundary(const double& adc_left, 
                   const double& adc_right, const double& start_s, double& boundary_error) const {
  double left_width = 0.0;
  double right_width = 0.0;
  const double kBoundaryBuff = 0.0;
  const double kDrivingAvailableWidth= EgoInfo::instance()->vehicle_param().half_wheel + kBoundaryBuff;
  if (reference_line_info_->reference_line().GetRoadWidth(start_s, &left_width, &right_width)) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"2) curr_adc_road bound = ("<<left_width
                        <<", "<<right_width<<") . adc_frenet_l_ = "<<adc_frenet_l_;
    if (fabs(left_width) <= kDrivingAvailableWidth ||
       fabs(right_width) <= kDrivingAvailableWidth) {
      AERROR<<"GetRoadWidth = ("<<left_width<<", "<<right_width
             <<"), <= kDrivingAvailableWidth , Get path bound failed !";
      boundary_error = std::fmax(kDrivingAvailableWidth - fabs(left_width), kDrivingAvailableWidth - fabs(right_width));        
      return false;       
    }
    if ((adc_frenet_l_ > 0 && 
            adc_left > fabs(left_width) - kBoundaryBuff)) {
      boundary_error = fabs(adc_left - fabs(left_width) + kBoundaryBuff);  
      return false;
    } 
    if ((adc_frenet_l_ < 0 && 
            fabs(adc_right) > fabs(right_width) - kBoundaryBuff)) {
      boundary_error = fabs(fabs(adc_right) - fabs(right_width) + kBoundaryBuff);  
      return false;
    } 
  }
  return true;
}


bool 
PathBoundGenerator::IsInDecisionBound(const double& adc_left,  const double& adc_right, 
             const double& start_s) const {
  if (!reference_line_info_->DecisionPathBound().empty()) {
    DecisionPathBoundProvider decision_path_bound_provider;
    double decision_left_bound = 0.0;
    double decision_right_bound = 0.0;
    auto first_bound_tuple = reference_line_info_->DecisionPathBound().at(0);
    bool find_init_s_bound = false;
    for (auto& bound_tuple : reference_line_info_->DecisionPathBound()) {
      if (adc_frenet_s_ <= get<0>(bound_tuple)) {
        first_bound_tuple = bound_tuple;
        find_init_s_bound = true;
        AINFO_IF(FLAGS_enable_debug_motion)<<"init_s = "<<adc_frenet_s_ 
                <<" , path bound = ("<<get<1>(first_bound_tuple)<<", "<<get<2>(first_bound_tuple)<<").";
        break;
      }
    }
    int left_type = -1;
    int right_type = -1;
    if (find_init_s_bound && reference_line_info_->reference_line().GetMapPath().
        GetLaneBoundaryType(left_type, right_type, std::max(0.0,start_s))) {
      if (decision_path_bound_provider.GetDecisionPathBound(decision_left_bound, decision_right_bound,
                                  std::fmax(start_s, 0.0), 
                                  get<1>(first_bound_tuple), get<2>(first_bound_tuple), reference_line_info_) ) {
        if (adc_frenet_l_ > 0) {
          if (left_type < 3 && left_type >=0) {
            if (fabs(adc_left) < decision_left_bound + kDottedBoundBuffer) return true;
          } else {
            if (fabs(adc_left) < decision_left_bound) return true;
          }
        } else {
            if (right_type < 3 && right_type >=0) {
            if (fabs(adc_right) < fabs(decision_right_bound) + kDottedBoundBuffer) return true;
          } else {
            if (fabs(adc_right) < fabs(decision_right_bound)) return true;
          }
        }
        
        // if ( (adc_frenet_l_ > 0 && 
        //         fabs(adc_left) > decision_left_bound)
        //     || (adc_frenet_l_ < 0 && 
        //         fabs(adc_right) > fabs(decision_right_bound))) {
        //   AINFO_IF(FLAGS_enable_debug_motion)<<"init point bound = ("<<-decision_right_bound<<", "<<decision_left_bound<<"). ";
        //   return false;
        // } else {
        //   return true;
        // }
      }
    } else {
      AERROR_IF(FLAGS_enable_debug_motion)<<"cannot find init_s path bound !!!!!";
      return false;
    }
  }
  return false;//没有决策边界时，默认return false;
}

bool 
PathBoundGenerator::GetBoundaryFromDecision(const double& curr_s , 
      double& left_bound, double& right_bound, int& left_bound_id, int& right_bound_id) {
  DecisionPathBoundProvider decision_path_bound_provider;
  if (!reference_line_info_->DecisionPathBound().empty()) { //如果决策边界不为空
    auto first_bound = reference_line_info_->DecisionPathBound().front();
    for (auto& bound_tuple : reference_line_info_->DecisionPathBound()) {
      if (curr_s <= get<0>(bound_tuple)) {
        double decision_left_bound = 0.0;
        double decision_right_bound = 0.0;
        left_bound_id = get<1>(bound_tuple);
        right_bound_id = get<2>(bound_tuple);//<0>：距离； 后面两位是边界值
        if (decision_path_bound_provider.GetDecisionPathBound(decision_left_bound, decision_right_bound,curr_s, 
                                           get<1>(bound_tuple), get<2>(bound_tuple), reference_line_info_)) {
          left_bound = decision_left_bound;
          right_bound = decision_right_bound;
        }
        break;
      }
    }
  } else {
    return false;
  } 
  return true;
}

bool 
PathBoundGenerator::GetBoundaryFromLanesAndADC(
    const LaneBorrowInfo& lane_borrow_info, const double& ADC_buffer,
    LaneBound* const path_bound, std::string* const borrow_lane_type, int& path_blocked_idx) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  CHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info_->reference_line();

  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  const double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  double adc_lane_left_width = 2.0;
  double adc_lane_right_width = 2.0;
  reference_line.GetLaneWidth(adc_frenet_s_, &adc_lane_left_width,
                                   &adc_lane_right_width);
  double past_lane_left_width = adc_lane_left_width;
  double past_lane_right_width = adc_lane_right_width;
  bool borrowing_reverse_lane = false;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(std::max(0.0,curr_s), &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      double offset_to_lane_center = 0.0;
      // reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);//TODO
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }

    // 2. Get the neighbor lane widths at the current point.
    double curr_neighbor_lane_width = 0.0;

    // 3. Calculate the proper boundary based on lane-width, ADC's position,
    //    and ADC's velocity.
    static constexpr double kMaxLateralAccelerations = 1.5;
    double offset_to_map = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &offset_to_map);//TODO

    double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                              adc_frenet_ld_ * adc_frenet_ld_ /
                              kMaxLateralAccelerations / 2.0;

    double curr_left_bound_lane =
        curr_lane_left_width + (lane_borrow_info == LaneBorrowInfo::LEFT_BORROW
                                    ? curr_neighbor_lane_width
                                    : 0.0);
    double curr_left_bound_adc =
        std::fmax(adc_frenet_l_,
                  adc_frenet_l_ + ADC_speed_buffer) +
        adc_half_width + ADC_buffer;
    double curr_left_bound =
        std::fmax(curr_left_bound_lane, curr_left_bound_adc) - offset_to_map;

    double curr_right_bound_lane =
        -curr_lane_right_width -
        (lane_borrow_info == LaneBorrowInfo::RIGHT_BORROW
             ? curr_neighbor_lane_width
             : 0.0);
    double curr_right_bound_adc =
        std::fmin(adc_frenet_l_,
                  adc_frenet_l_ + ADC_speed_buffer) -
        adc_half_width - ADC_buffer;
    double curr_right_bound =
        std::fmin(curr_right_bound_lane, curr_right_bound_adc) - offset_to_map;

    // 4. Update the boundary.
    double dummy = 0.0;
    if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound,
                                         path_bound, &dummy)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  bound_adjustment_ptr_->TrimPathBounds(path_blocked_idx, path_bound);
  bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
  bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  if (lane_borrow_info == LaneBorrowInfo::NO_BORROW) {
    *borrow_lane_type = "";
  } else {
    *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  }

  return true;
}

common::Status PathBoundGenerator::CheckAbandonLCValid(double& k_length){
  common::Status rtvalue = Status::OK();
  if (reference_line_info_->LaterAction().first == eActionEnum::ABANDON_LANE_CHANGE) {
    k_length = 100;
    double solid_line_start_s = k_length;
    int search_direction = adc_frenet_l_ >= 0 ? 1 : 2;
    if (GetSolidLineStartS(search_direction, solid_line_start_s)) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"solid_line_start_s = "<<solid_line_start_s;
      k_length = std::fmin(k_length, solid_line_start_s - adc_frenet_s_);
      if (solid_line_start_s < adc_frenet_s_) {
        const std::string msg =
                "solid line start_s is less than adc_frenet_s_ .";
        AERROR_IF(FLAGS_enable_debug_motion) << msg;
        return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
      } else {
        double dl = fabs(adc_frenet_l_ - 1.5 ) / (adc_frenet_s_ - solid_line_start_s);
        if (dl > FLAGS_lateral_derivative_bound_default) {
          const std::string msg =
                "solid line start_s is too closed .";
          AERROR_IF(FLAGS_enable_debug_motion) << msg;
          return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
        }
      }
    }
  }
  return rtvalue;
}

common::Status PathBoundGenerator::CalculateSingleBoundWithDecision(
      const double& curr_s, const bool& is_in_decision_bound, 
      double& curr_left_bound_lane, double& curr_right_bound_lane, 
      int& left_bound_id, int& right_bound_id){

  common::Status rtvalue = Status::OK();
  if (!GetBoundaryFromDecision(curr_s, curr_left_bound_lane, curr_right_bound_lane, left_bound_id, right_bound_id)) {
    const std::string msg =
          "GetBoundaryFromDecision failed s : " + std::to_string(curr_s);
    AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  } else if (is_in_decision_bound && left_bound_id == right_bound_id){//如果为车道内避障，并且为虚线，把认知减掉的buffer加回来。
    int left_type = -1;
    int right_type = -1;
    if (reference_line_info_->reference_line().GetMapPath().GetLaneBoundaryType(left_type, right_type, std::max(0.0,curr_s))) {
      if (left_type < 3) {
        curr_left_bound_lane = curr_left_bound_lane + FLAGS_boundary_width + kDottedBoundBuffer;
      }
      if (right_type < 3) {
        curr_right_bound_lane = curr_right_bound_lane - FLAGS_boundary_width - kDottedBoundBuffer;
      }
    } else {
      AWARN_IF(FLAGS_enable_debug_motion)<<"GetLaneBoundaryType failed!!";
    }
  }
  if (left_bound_id == 50) {//borrow lane 
    double road_left_width = 0.0;
    double road_right_width = 0.0;
    if (reference_line_info_->reference_line().GetRoadWidth(std::fmax(curr_s, 0.0), &road_left_width, &road_right_width)){
      if (curr_left_bound_lane - road_left_width > EgoInfo::instance()->vehicle_param().car_width) {
        curr_left_bound_lane = road_left_width + EgoInfo::instance()->vehicle_param().car_width;
      } else {
        curr_left_bound_lane = curr_left_bound_lane - 0.5;
      }
    } else {
      curr_left_bound_lane = curr_left_bound_lane - 0.5;
    }    
  }
  
  return rtvalue;
}

common::Status PathBoundGenerator::CalculateSingleBoundWithoutDecision(
    const double& curr_s, const size_t& index,
    const double& curr_right_bound_adc, const double& curr_left_bound_adc, 
    double& curr_left_bound_lane, double& curr_right_bound_lane){

  common::Status rtvalue = Status::OK();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  int left_type = -1;
  int right_type = -1;
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  if (adc_frenet_l_ > 0 && adc_frenet_l_ - adc_half_width >= 0) {
    if (reference_line.GetMapPath().GetLaneBoundaryType(left_type, right_type, std::max(0.0, curr_s))) {
      if (left_type > 2) {//solid line
        double lane_left_width = 0;
        double lane_right_width = 0;
        if (reference_line.GetLaneWidth(std::max(0.0,curr_s), &lane_left_width,&lane_right_width)) {
          if (index < 1 && adc_frenet_l_ - adc_half_width > lane_left_width) {
            const std::string msg =
                  "planning init point is out of solid line s : " + std::to_string(curr_s);
            AERROR_IF(FLAGS_enable_debug_motion) << msg;
            return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
          } else {
            curr_left_bound_lane = lane_left_width;
          }
        } else {
          AWARN_IF(FLAGS_enable_debug_motion)<<"get lane width failed !!";
        }
      }
    } else {
      AWARN_IF(FLAGS_enable_debug_motion)<<"get lane type failed !!";
    }
  } else if (adc_frenet_l_ < 0 && adc_frenet_l_ + adc_half_width <= 0) {
    if (reference_line.GetMapPath().GetLaneBoundaryType(left_type,right_type,std::max(0.0,curr_s))) {
      if (right_type > 2) {//SOLID LINE
        double lane_left_width = 0;
        double lane_right_width = 0;
        if (reference_line.GetLaneWidth(std::max(0.0,curr_s), &lane_left_width,&lane_right_width)) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"lane bound = ("<<lane_left_width<<", "<<lane_right_width<<")";
          if (index < 1 && adc_frenet_l_ + adc_half_width < -lane_right_width) {
            const std::string msg =
                  "planning init point is out of solid line s : " + std::to_string(curr_s);
            AERROR_IF(FLAGS_enable_debug_motion) << msg;
            return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
          } else {
            curr_right_bound_lane = -lane_right_width;
          }
        } else {
          AWARN_IF(FLAGS_enable_debug_motion)<<"get lane width failed !!";
        }
      }
    } else {
      AWARN_IF(FLAGS_enable_debug_motion)<<"get lane width failed !!";
    }
  } else {
    double lane_left_width = 0;
    double lane_right_width = 0;
    if (reference_line.GetLaneWidth(std::max(0.0,curr_s), &lane_left_width,&lane_right_width)) {
      curr_left_bound_lane = lane_left_width;
      curr_right_bound_lane = -lane_right_width;
      if (index < 1) {
        curr_left_bound_lane = std::fmax(curr_left_bound_lane, curr_left_bound_adc);
        curr_right_bound_lane = std::fmin(curr_right_bound_lane, curr_right_bound_adc);
      }
    } else {
      AWARN_IF(FLAGS_enable_debug_motion)<<"get lane width failed !!";
    }
  }
  return rtvalue;
}


common::Status PathBoundGenerator::LimitSingleBoundByRoadWidth(
      const double& curr_s, const bool& is_in_road_boundary, const size_t& index,
      const double& s_max_consider_adc_l, const int& left_bound_id,
      const double& curr_left_bound_lane, const double& curr_right_bound_lane, 
      const double& curr_right_bound_adc, const double& curr_left_bound_adc, 
      double& curr_left_bound, double& curr_right_bound){

  common::Status rtvalue = Status::OK();
  const double kBoundaryBuff = 0.0;
  const double kDrivingAvailableWidth = EgoInfo::instance()->vehicle_param().half_wheel + kBoundaryBuff;
  curr_left_bound = curr_left_bound_lane;  
  curr_right_bound = curr_right_bound_lane;
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  if (!is_in_road_boundary && (curr_s < s_max_consider_adc_l + adc_frenet_s_)) {
    if (adc_frenet_l_ < 0 ) {
      curr_right_bound = curr_right_bound_adc;
    } else {
      curr_left_bound = curr_left_bound_adc;
    }
  } else {
    if (reference_line_info_->reference_line().GetRoadWidth(std::fmax(curr_s, 0.0), &road_left_width, &road_right_width)) {
      if (fabs(road_left_width) + fabs(road_right_width) <= 2*kDrivingAvailableWidth) {
        const std::string msg = "s : " + std::to_string(curr_s)
                + "Road width : (" + std::to_string(road_left_width) 
                + ", " + std::to_string(road_right_width) + "<= 2*kDrivingAvailableWidth. ";
        AERROR_IF(FLAGS_enable_debug_motion) << msg;
        return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);     
      }
      if (left_bound_id != 50) {
        curr_left_bound = std::fmin(curr_left_bound, fabs(road_left_width) - kBoundaryBuff);
      }
      if (BehaviorParser::instance()->IsAutoDriveStart()) {
        //起步进自动时，把认知减的边界加上
        curr_right_bound = std::fmax(curr_right_bound - FLAGS_boundary_width, -fabs(road_right_width) + kBoundaryBuff);
      } else {
        curr_right_bound = std::fmax(curr_right_bound, -fabs(road_right_width) + kBoundaryBuff);
      }
      if (index < 1 && curr_right_bound > curr_right_bound_adc && adc_frenet_l_ < 0 ) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"init point is out of road bound !!";
        curr_right_bound = curr_right_bound_adc;
      }
    } else {
      AWARN_IF(FLAGS_enable_debug_motion)<<"can't not get road width. !";
    }
  }
  return rtvalue;
}

common::Status PathBoundGenerator::GetBoundary(const double& ADC_buffer, 
    LaneBound* const path_bound, std::vector<std::pair<int, int>>* const boundary_type,
    std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary) {
  int path_blocked_idx = -1;
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  CHECK(!path_bound->empty());
  double kLength = 20;
  //校验放弃换道可行性
  auto status = CheckAbandonLCValid(kLength);
  if(!status.ok()) {return status;}

  const double kAbandonLcBoundBuffer = 1.0;
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  // Go through every point, update the boundary based on desion info and
  // ADC's position.
  static constexpr double kMaxLateralAccelerations = 1.5;
  double ADC_speed_buffer = (adc_frenet_ld_ > 0 ? 1.0 : -1.0) *
                            adc_frenet_ld_ * adc_frenet_ld_ /
                            kMaxLateralAccelerations / 2.0;
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  double curr_left_bound_adc = adc_frenet_l_ + adc_half_width + ADC_buffer; //pqg
  double curr_right_bound_adc = adc_frenet_l_ - adc_half_width -ADC_buffer;//pqg
  //车辆当前车身的左右边界
  AINFO_IF(FLAGS_enable_debug_motion)<<"curr_left_bound_adc = "<<curr_left_bound_adc
                          <<", curr_right_bound_adc = "<<curr_right_bound_adc;

  //判断是不是在road边界以内
  double start_s = std::fmax(0.0, std::get<0>((*path_bound)[0])); 
  double boundary_error = 0;                        
  bool is_in_road_boundary = true;
  if (!BehaviorParser::instance()->plan_in_lane() 
      && !reference_line_info_->IsChangeLanePath()) {//借道避障时不认为不在road边界里。
    is_in_road_boundary = true;//TODO
  } else {
    is_in_road_boundary = IsInRoadBoundary(curr_left_bound_adc, curr_right_bound_adc, start_s, boundary_error);
  }
  if (!is_in_road_boundary && boundary_error > FLAGS_boundary_error_max) {
    const std::string msg =
                "plan start point is not in road , and boundary_error : " 
                + std::to_string(boundary_error)
                + " > " + std::to_string(FLAGS_boundary_error_max);
          AERROR_IF(FLAGS_enable_debug_motion) << msg;
    return Status(ErrorCode::PLANNING_PATHPLAN_SOFTWARE_0_ERROR, msg);
  }
  //判断是不是在决策给出的边界内；
  bool is_in_decision_bound = IsInDecisionBound(curr_left_bound_adc, curr_right_bound_adc, start_s);
  AINFO_IF(FLAGS_enable_debug_motion)<<"is_in_road_boundary = "<<is_in_road_boundary
                                     <<", is_in_decision_bound = "<<is_in_decision_bound; 
  //如果不在边界内，延长一定距离再受到边界约束，以下为确定该距离的逻辑                                    
  double s_max_consider_adc_l = kLength;
  if (!reference_line_info_->DecisionPathBound().empty()) {
    auto first_bound_s = get<0>(reference_line_info_->DecisionPathBound().front());
    if (first_bound_s >= adc_frenet_s_) {
      s_max_consider_adc_l = std::max(0.0, std::min(kLength, first_bound_s));
    }
  }  
  //遍历，找到所有s对应的bound
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = std::get<0>((*path_bound)[i]);
    // 1. Get the decison path bound at current point.
    double curr_left_bound_lane = std::fmax(1.75, curr_left_bound_adc);
    double curr_right_bound_lane = std::fmin(-1.75, curr_right_bound_adc);
    double curr_left_bound = curr_left_bound_lane;  
    double curr_right_bound = curr_right_bound_lane;
    int left_bound_id = 10;
    int right_bound_id = 10;
    if ((!is_in_road_boundary || !is_in_decision_bound)
       && curr_s <= s_max_consider_adc_l + adc_frenet_s_) {//不在road或者决策给出的边界以里，并且当前s < s_max_consider_adc_l
      if (adc_frenet_l_ > 0 ) {
        curr_left_bound_lane = curr_left_bound_adc;
        if (reference_line_info_->LaterAction().first == eActionEnum::ABANDON_LANE_CHANGE) {
          curr_left_bound_lane = curr_left_bound_lane + kAbandonLcBoundBuffer;
        }
      } else {
        curr_right_bound_lane = curr_right_bound_adc;
        if (reference_line_info_->LaterAction().first == eActionEnum::ABANDON_LANE_CHANGE) {
          curr_right_bound_lane = curr_right_bound_lane - kAbandonLcBoundBuffer;
        }
      }
    } else {//在边界以里或者不在边界以里但s>s_max_consider_adc_l
      if (!reference_line_info_->DecisionPathBound().empty()) { //如果决策边界不为空
        auto status = CalculateSingleBoundWithDecision(curr_s, is_in_decision_bound, 
                        curr_left_bound_lane, curr_right_bound_lane, left_bound_id, right_bound_id);
        if(!status.ok()) {return status;}
      } else { //决策边界为空
        auto status = CalculateSingleBoundWithoutDecision(curr_s, i, 
                          curr_right_bound_adc, curr_left_bound_adc, 
                          curr_left_bound_lane, curr_right_bound_lane);
        if(!status.ok()) {return status;}
      }
    }
     
    //加入road边界的双重约束
    auto status = LimitSingleBoundByRoadWidth(curr_s, is_in_road_boundary, i, 
                    s_max_consider_adc_l, left_bound_id, 
                    curr_left_bound_lane, curr_right_bound_lane, curr_right_bound_adc, 
                    curr_left_bound_adc, curr_left_bound, curr_right_bound);
    if(!status.ok()) {return status;}
    
    // 3. if lane change , considerate solid line, reduce lane boundary(solid line side)  
    int left_type = -1;
    int right_type = -1;
    const double mimimum_l = 0.1 + adc_half_width;
    if (reference_line.GetMapPath().GetLaneBoundaryType(left_type,right_type,std::max(0.0,curr_s))) {
      AINFO_IF(FLAGS_enable_debug_motion && i < 1)<<"left_type = "<<left_type<<", right_type = "<<right_type;
      if (reference_line_info_->IsChangeLanePath() 
         && FLAGS_enable_solid_line_constraint_in_path_decider) {
        if (reference_line_info_->Position() == LanePosition::LEFT && left_type > 2) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"i = "<<i<<"left_type = "<<left_type<<", curr_left_bound = "<<curr_left_bound;
          curr_left_bound = std::fmin(curr_left_bound, mimimum_l);
        } else if (reference_line_info_->Position() == LanePosition::RIGHT && right_type > 2) {
          curr_right_bound = std::fmax(curr_right_bound, -mimimum_l);
        }
      }
    } else {
      AINFO_IF(FLAGS_enable_debug_motion)<<"get lane type failed!!";
    }

    bound_adjustment_ptr_->UpdateBoundaryType(i,left_type,right_type,boundary_type);
    
    // 4. Update the boundary.consider vehicle width
    double dummy = 0.0;
    if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound, path_bound, &dummy)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  bound_adjustment_ptr_->TrimPathBounds(path_blocked_idx, path_bound);
  bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, soft_boundary);
  bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, boundary_type);
  return Status::OK();

}


bool 
PathBoundGenerator::GetFreeSpaceBoundary(const std::vector<std::pair<int, int>>& boundary_type, 
     LaneBound* const path_boundaries, int& path_blocked_idx) {
  if (path_boundaries->empty()) {
    return false;
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"Check freespace!";
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  for (size_t i = 0; i < path_boundaries->size();++i ) {
    double width_l = 1.75;
    double width_r = 1.75;
    if (reference_line_info_->reference_line().GetMapPath().GetFreeSpaceboundary(
                    std::get<0>((*path_boundaries).at(i)), &width_l, &width_r)) {
      if (std::get<0>((*path_boundaries).at(i)) > 90) {//超过90m,api返回的值是错误值（即api目前仅支持100m以内的查询）
        break;
      }
      // AWARN_IF(FLAGS_enable_debug_motion)<<"i = "<<i<<", fs_w = ("<<-width_r<<", "<<width_l<<").";
      const double r_bound_fs = -width_r + FLAGS_distance_to_freespace;
      const double l_bound_fs = width_l - FLAGS_distance_to_freespace;
      if (r_bound_fs >= 0 || l_bound_fs <= 0 ) {
        AERROR_IF(r_bound_fs >= 0)
           <<"r_bound_fs = "<<r_bound_fs<<", is less than "<<FLAGS_distance_to_freespace;
        AERROR_IF(l_bound_fs <= 0)<<"l_bound_fs = "<<l_bound_fs<<", is less than "<<FLAGS_distance_to_freespace;
        continue;
      }

      const double adc_r_bound_fs = r_bound_fs + adc_half_width;
      const double adc_l_bound_fs = l_bound_fs - adc_half_width;
      // AWARN_IF(FLAGS_enable_debug_motion)<<"i = "<<i<<", adc_w = ("<<adc_r_bound_fs<<", "<<adc_l_bound_fs
      //              <<"). prev ("<<std::get<1>((*path_boundaries).at(i))<<", "<<std::get<2>((*path_boundaries).at(i)) <<").";
      if (std::get<1>((*path_boundaries).at(i)) <= 0 ) {
        std::get<1>((*path_boundaries).at(i)) = 
           std::fmax(adc_r_bound_fs,std::get<1>((*path_boundaries).at(i))); 
      }
      if (std::get<2>((*path_boundaries).at(i)) >= 0 ) {
        std::get<2>((*path_boundaries).at(i)) = 
           std::fmin(adc_l_bound_fs,std::get<2>((*path_boundaries).at(i))); 
      }
      // AWARN_IF(FLAGS_enable_debug_motion)<<"i = "<<i
      //              <<" after ("<<std::get<1>((*path_boundaries).at(i))<<", "<<std::get<2>((*path_boundaries).at(i)) <<").";

      if (std::get<1>((*path_boundaries).at(i)) > std::get<2>((*path_boundaries).at(i))) {
        AINFO_IF(FLAGS_enable_debug_motion) << "Path is blocked at idx = " << i << " need modify";
        if (!ReCalculatePathBoundBaesdOnLaneType(boundary_type, path_boundaries, i , width_l, width_r)) {
          AINFO_IF(FLAGS_enable_debug_motion) << "PathBound re_calculate failed , Path is blocked at " << i << ". ";
          path_blocked_idx = i;
          break; 
        }
      }
    } else {
      // AINFO_IF(FLAGS_enable_debug_motion)<<"can not get fs boundary";
    }
  }

  bound_adjustment_ptr_->TrimPathBounds(path_blocked_idx, path_boundaries);
  bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
  bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  return true;
}


// ReCalculatePathBoundBaesdOnLaneType when blocked
bool 
PathBoundGenerator::ReCalculatePathBoundBaesdOnLaneType(
            const std::vector<std::pair<int, int>>& boundary_type, 
            LaneBound* const path_boundaries, 
            const size_t& idx, const double& fs_width_l, const double& fs_width_r) const {
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  const int max_width_side = fs_width_l > fs_width_r ? 1 : -1;
  const double r_bound_fs = -fs_width_r + FLAGS_distance_to_freespace;
  const double l_bound_fs = fs_width_l - FLAGS_distance_to_freespace;
  bool is_l_blocked = false;
  bool is_r_blocked = false;
  const double error = std::get<1>((*path_boundaries).at(idx)) - std::get<2>((*path_boundaries).at(idx)) + 0.001;
  if (boundary_type.size() > idx) {
    if (boundary_type.at(idx).second >= 3 && boundary_type.at(idx).first >= 3) {//两边都是实线
      if (error > FLAGS_over_distance_to_solid_line) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"error = "<<error<<", too large.";
        return false;
      } else {
        const double modify_l_bound = std::get<2>((*path_boundaries).at(idx)) 
            + std::fmin(FLAGS_over_distance_to_solid_line, error);
        if (max_width_side > 0) {//change left
          if (modify_l_bound + adc_half_width < l_bound_fs) {
            std::get<2>((*path_boundaries).at(idx)) = modify_l_bound;
          } else {//修改后的边界出fs边界了
            return false;
            AWARN_IF(FLAGS_enable_debug_motion)<<"modify result not in freespace . "
                  <<"check left bound type or freespace , i = "<<idx;
          }
        } else {// change right
          const double modify_r_bound = std::get<1>((*path_boundaries).at(idx)) 
              - std::fmin(FLAGS_over_distance_to_solid_line, error);
          if (modify_r_bound - adc_half_width > r_bound_fs) {
            std::get<1>((*path_boundaries).at(idx)) = modify_r_bound;
          } else {//修改后的边界出fs边界了
            return false;
            AWARN_IF(FLAGS_enable_debug_motion)<<"modify result not in freespace . "
                <<"check right bound type or freespace , i = "<<idx;
          }
        }
        AWARN_IF(FLAGS_enable_debug_motion)<<"over solid line ";
      }
    } else {//至少有一边为虚线
      if (boundary_type.at(idx).second < 3) {//虚线
        //0.2是说左边界与右边界至少有0.2m的间隙
        const double modify_l_bound = 
        std::get<2>((*path_boundaries).at(idx)) + error + 0.2 < l_bound_fs - adc_half_width? 
               std::get<2>((*path_boundaries).at(idx)) + error + 0.2 :
                      std::fmax(std::get<2>((*path_boundaries).at(idx)) + error, 
                         l_bound_fs - adc_half_width);
        //如果保持0.2间隙也不出fs边界，则选择保持0.2的边界，否则使用fs与不保持0.2间隙的最大值              
        if (modify_l_bound + adc_half_width < l_bound_fs) {
          std::get<2>((*path_boundaries).at(idx)) = modify_l_bound;
        } else {
          is_l_blocked = true;
          AWARN_IF(FLAGS_enable_debug_motion)<<"modify result not in freespace . "
                <<"check left bound type or freespace , i = "<<idx;
        }
      } else {
        is_l_blocked = true;
      }
      if (boundary_type.at(idx).second >= 3 || is_l_blocked) {//左边没有空间修正
        if (boundary_type.at(idx).first < 3) {
          const double modify_r_bound = 
                std::get<1>((*path_boundaries).at(idx)) - error - 0.2 > r_bound_fs + adc_half_width?
                std::get<1>((*path_boundaries).at(idx)) - error - 0.2 : 
                std::fmin(std::get<1>((*path_boundaries).at(idx)) - error, 
                          r_bound_fs + adc_half_width);
          if (modify_r_bound - adc_half_width > r_bound_fs) {
            std::get<1>((*path_boundaries).at(idx)) = modify_r_bound;
          } else {
            is_r_blocked = true;
            AWARN_IF(FLAGS_enable_debug_motion)<<"modify result not in freespace . "
                <<"check right bound type or freespace , i = "<<idx;
          }
        } else {
          is_r_blocked = true;
        }
      }
    }
  }
  if (is_l_blocked && is_r_blocked) {//无法重新计算，则依旧保持blocked状态
    return false;
  }

  return true;
}


bool 
PathBoundGenerator::GetBoundaryFromRoads(LaneBound* const path_bound, const int check_lane_type, int& path_blocked_idx) {
  // Sanity checks.
  CHECK_NOTNULL(path_bound);
  CHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;

  // Go through every point, update the boudnary based on the road boundary.
  double adc_lane_left_width = 2.0;
  double adc_lane_right_width = 2.0;
  reference_line.GetLaneWidth(adc_frenet_s_, &adc_lane_left_width,
                                   &adc_lane_right_width);
  double past_road_left_width = adc_lane_left_width;
  double past_road_right_width = adc_lane_right_width;
  for (size_t i = 0; i < path_bound->size(); ++i) {
    // 1. Get road boundary.
    double curr_s = std::get<0>((*path_bound)[i]);
    double curr_road_left_width = 0.0;
    double curr_road_right_width = 0.0;
    double refline_offset_to_lane_center = 0.0;
    // reference_line.GetOffsetToMap(curr_s, &refline_offset_to_lane_center);//TODO
    if (!reference_line.GetMapPath().GetRoadBoundary(std::max(0.0,curr_s), &curr_road_left_width,
                                     &curr_road_right_width)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to get lane width at s = " << curr_s;
      curr_road_left_width = past_road_left_width;
      curr_road_right_width = past_road_right_width;
    } else {
      curr_road_left_width += refline_offset_to_lane_center;
      curr_road_right_width -= refline_offset_to_lane_center;
      past_road_left_width = curr_road_left_width;
      past_road_right_width = curr_road_right_width;
    }
    double curr_left_bound = curr_road_left_width;
    double curr_right_bound = -curr_road_right_width;

    AINFO_IF(FLAGS_enable_debug_motion && i < 1)<<"first road bound ("
                 <<curr_road_left_width<<", "<<curr_road_right_width<<").";

    int left_type = -1;
    int right_type = -1;
    const double bound_buffer = 0.1;
    const bool lane_left_boundary_constraint = true;//使能道路左车道线约束，限制靠边停车时左边界
    if (reference_line_info_->reference_line().GetMapPath().GetLaneBoundaryType(left_type,right_type,std::max(0.0,curr_s))) {
      double left_lane_width = 1.75;
      double right_lane_width = 1.75;
      if (left_type > 2 || right_type > 2) {
        AINFO_IF(FLAGS_enable_debug_motion && i < 1)<<"first s lane type ("<<left_type<<", "<<right_type<<").";
        if (!reference_line.GetLaneWidth(std::max(0.0,curr_s), &left_lane_width,
                              &right_lane_width)) {
          left_lane_width = 1.75;
          right_lane_width = 1.75;
        }
      }
      if (left_type > 2 && check_lane_type >= 0) {//SOLID LINE
        double adc_left_pos = adc_frenet_l_ + adc_half_width;
        if (adc_frenet_l_ > 0 &&  adc_left_pos> left_lane_width && i < 1) {//起点在实线外
          AERROR<<"curr_s = "<<curr_s <<" is out of solid line ! make pathbound = (adc_left_pos , left lane bound)";
          curr_left_bound = adc_left_pos + bound_buffer;
          curr_right_bound = left_lane_width;
        } else {
          curr_left_bound = left_lane_width;
        }
      } else if (lane_left_boundary_constraint) {
        curr_left_bound = std::max(left_lane_width ,adc_frenet_l_ + adc_half_width + bound_buffer);
      }
      if (right_type > 2 && check_lane_type <= 0 && i < 1) {
        double adc_right_pos = adc_frenet_l_ - adc_half_width;
        if (adc_frenet_l_ < 0 &&  adc_right_pos < -right_lane_width) {
          AERROR<<"curr_s = "<<curr_s <<" is out of solid line ! make pathbound = (right lane bound , adc_right_pos)";
          curr_left_bound = -right_lane_width;
          curr_right_bound = adc_right_pos - bound_buffer;
        } else {
          curr_right_bound = -right_lane_width;
        }
      }
    }

    // 2. Update into path_bound.
    double dummy = 0.0;
    if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(i, curr_left_bound, curr_right_bound,
                                         path_bound, &dummy)) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      break;
    }
  }

  bound_adjustment_ptr_->TrimPathBounds(path_blocked_idx, path_bound);
  bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, &soft_boundary_);
  bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, &boundary_type_);
  return true;
}


std::vector<ObstacleEdge> 
PathBoundGenerator::SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles, 
      const double& obstacle_lat_buffer, const LaneBound* path_boundaries) {
  return obstacle_bound_process_ptr_->SortObstaclesForSweepLine(indexed_obstacles, obstacle_lat_buffer, path_boundaries);
}


bool 
PathBoundGenerator::GetBoundaryFromStaticObstacles(
    const PathDecision* path_decision, LaneBound* const path_boundaries,
    std::string* const blocking_obstacle_id, const double& obstacle_lat_buffer,
    std::vector<std::pair<int, int>>* const boundary_type,
    std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary) {
  // Preprocessing.
  std::vector<std::pair<int, int>> bound_types(path_boundaries->size(),make_pair(0,0));
  
  double adc_half_width = EgoInfo::instance()->vehicle_param().half_wheel;
  auto indexed_obstacles = path_decision->obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(indexed_obstacles, obstacle_lat_buffer, path_boundaries);
  AINFO_IF(FLAGS_enable_debug_motion) << "There are " << sorted_obstacles.size() << " static obstacles.";// perception static obstacle size * 2
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
  // Maps obstacle ID's to the decision of whether side-pass on this obstacle
  // is allowed. If allowed, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_sidepass_decision;

  // Step through every path point.
  for (size_t i = 1; i < path_boundaries->size(); ++i) {
    bound_types.at(i) = bound_types.at(i - 1);
    bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first,boundary_type);
    double curr_s = std::get<0>((*path_boundaries)[i]);
    // Check and see if there is any obstacle change:
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
          if(center_line_vec.size() > 2 || center_line_vec.size() < 1){
            AERROR<<"error center_line_vec size: "<<center_line_vec.size();
          }
          bool is_all_center_line_block = false;
          AINFO_IF(FLAGS_enable_debug_motion)<<"[1]center_line_vec size: "<<center_line_vec.size();
          for(size_t j = 0; j < center_line_vec.size(); ++j){
            center_line = center_line_vec.at(j).first;
            AINFO_IF(FLAGS_enable_debug_motion)<<"curr_obstacle_l_min: "<<curr_obstacle_l_min<<", curr_obstacle_l_max: "<<curr_obstacle_l_max
              <<", center_line: "<<center_line<<", j: "<<j;
            if (curr_obstacle_l_min + curr_obstacle_l_max < center_line * 2) {
              // Obstacle is to the right of center-line, should pass from left.
              if (std::get<1>((*path_boundaries)[i]) < curr_obstacle_l_max + adc_half_width) {
                bound_types.at(i).first = 7;
                bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first, boundary_type);
              }
            
              AINFO_IF(FLAGS_enable_debug_motion)<<"should pass from left, id = "<<curr_obstacle_id;
              obs_id_to_direction[curr_obstacle_id] = true;
              obs_id_to_sidepass_decision[curr_obstacle_id] = true;
              right_bounds.insert(curr_obstacle_l_max);
              AINFO_IF(FLAGS_enable_debug_motion)<<"insert right_bounds: "<<curr_obstacle_l_max;
              if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
                      i, *left_bounds.begin(), *right_bounds.begin(),
                      path_boundaries, &center_line)) {
                if(j == (center_line_vec.size() - 1)){
                  path_blocked_idx = static_cast<int>(i);
                  *blocking_obstacle_id = curr_obstacle_id;
                  is_all_center_line_block = true;
                }
                //如果不合适，需要擦除，否则可能导致right_bounds/left_bounds没有更新，错误阻塞
                if(obs_id_to_direction.count(curr_obstacle_id)){
                  if(obs_id_to_direction[curr_obstacle_id] && right_bounds.count(curr_obstacle_l_max)){
                    right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
                    AINFO_IF(FLAGS_enable_debug_motion)<<"erase right_bounds: "<<curr_obstacle_l_max;
                  }else{
                    AERROR<<"ERROR LOGIC!!!";
                  }
                  obs_id_to_direction.erase(curr_obstacle_id);
                }else{
                  AERROR<<"ERROR LOGIC!!!";
                }
                continue;
              }
              bool is_obstacle = true;
              bool is_worse_option = (center_line_vec.size() > 1) ? (j == (center_line_vec.size() - 1)) : false;
              bound_adjustment_ptr_->UpdateCenterLineVec(center_line_vec, center_line, is_obstacle, is_worse_option);
            } else {
              // Obstacle is to the left of center-line, should pass from right.
              if (std::get<2>((*path_boundaries)[i]) > curr_obstacle_l_min - adc_half_width) {
                 bound_types.at(i).second = 7;
                bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first,boundary_type);
              }

              AINFO_IF(FLAGS_enable_debug_motion)<<"should pass from right, id "<<curr_obstacle_id;
              obs_id_to_direction[curr_obstacle_id] = false;
              obs_id_to_sidepass_decision[curr_obstacle_id] = false;
              left_bounds.insert(curr_obstacle_l_min);

              if (!bound_adjustment_ptr_->UpdatePathBoundaryAndCenterLine(
                      i, *left_bounds.begin(), *right_bounds.begin(),
                      path_boundaries, &center_line)) {
                if(j == (center_line_vec.size() - 1)){
                  path_blocked_idx = static_cast<int>(i);
                  *blocking_obstacle_id = curr_obstacle_id;
                  is_all_center_line_block = true;
                }
                //如果不合适，需要擦除，否则可能导致right_bounds/left_bounds没有更新，错误阻塞
                if(obs_id_to_direction.count(curr_obstacle_id)){
                  if(!obs_id_to_direction[curr_obstacle_id] && left_bounds.count(curr_obstacle_l_min)){
                    left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
                    AINFO_IF(FLAGS_enable_debug_motion)<<"erase left_bounds: "<<curr_obstacle_l_min;
                  }else{
                    AERROR<<"ERROR LOGIC!!!";
                  }
                  obs_id_to_direction.erase(curr_obstacle_id);
                }else{
                  AERROR<<"ERROR LOGIC!!!";
                }
                continue;
              }
              bool is_obstacle = true;
              bool is_worse_option = (center_line_vec.size() > 1) ? (j == (center_line_vec.size() - 1)) : false;
              bound_adjustment_ptr_->UpdateCenterLineVec(center_line_vec, center_line, is_obstacle, is_worse_option);
            }
            if(!is_all_center_line_block) break;
          }//end for-loop
          if(is_all_center_line_block) break;//exit while-loop
        } else {
          // An existing obstacle exits our scope.
          bound_types.at(i).first = 0;
          bound_types.at(i).second = 0;
          bound_adjustment_ptr_->UpdateBoundaryType(i,bound_types.at(i).second,bound_types.at(i).first, boundary_type);

          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
          }
          obs_id_to_direction.erase(curr_obstacle_id);

          // Update the bounds and center_line.
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
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
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
      break;
    }
  }
  bound_adjustment_ptr_->TrimPathBounds(path_blocked_idx, path_boundaries);
  bound_adjustment_ptr_->TrimPathSoftBounds(path_blocked_idx, soft_boundary);
  bound_adjustment_ptr_->TrimPathBoundTypes(path_blocked_idx, boundary_type);
  // if (path_blocked_idx != -1) {
  //   blocked_infos_.is_blocked = true;
  //   blocked_infos_.index = path_blocked_idx;
  //   blocked_infos_.s = std::get<0>((*path_boundaries)[path_blocked_idx]);
  //   int end_index = std::max(0, path_blocked_idx - 1);
  // }
  obstacle_bound_process_ptr_->GetSoftBoundaryFromUncertainObstacles(path_decision, 
                                 path_boundaries, obs_id_to_sidepass_decision,
                                 boundary_type, soft_boundary);
  
  if (FLAGS_enable_piecewise_path_debug) {
    for (size_t i = 0; i < boundary_type->size(); ++i) {
      AWARN_IF(FLAGS_enable_debug_motion) << "idx " << i 
            << "; right_type = " << (*boundary_type)[i].first
            << "; left_type = " << (*boundary_type)[i].second;
    }
  }
  return true;
}


bool 
PathBoundGenerator::CalculateDynamicObstacleBound(LaneBound* const dynamic_bound) { 
  std::vector<common::SpeedPoint> heuristic_speed;
  std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes;
  std::vector<std::vector<SLBoundary>> dynamic_obstacle_sl_boundaries;
  
  double eval_time_interval = 0.1;
  double total_time = 7.0;
  uint32_t num_of_time_stamps = static_cast<uint32_t>(
                          std::floor(total_time / eval_time_interval));
  if (FLAGS_use_predict_info_in_piecewise_jerk_path) {
    obstacle_bound_process_ptr_->GetDynamicObstacleBoxes(num_of_time_stamps, eval_time_interval, 
                                                          total_time, dynamic_obstacle_boxes);
  } else {
    obstacle_bound_process_ptr_->GetDynamicObstacleSLBoundaries(num_of_time_stamps, eval_time_interval, 
                                                          total_time, dynamic_obstacle_sl_boundaries);
  }
  
  if (!dynamic_obstacle_boxes.empty() || !dynamic_obstacle_sl_boundaries.empty()) {
    heuristic_speed.clear();
    double time_stamp = 0.0;
    for (size_t index = 0; index < num_of_time_stamps;
       ++index, time_stamp += eval_time_interval) {
      common::SpeedPoint speed_point;
      reference_line_info_->speed_data().EvaluateByTime(time_stamp, &speed_point);
      // AINFO_IF(FLAGS_enable_debug_motion)<<"ts("<<speed_point.t()<<", "<<speed_point.s();
      heuristic_speed.push_back(speed_point);
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"dynamic_obstacle_boxes_.size = "<<dynamic_obstacle_boxes.size()
    <<", dynamic_obstacle_sl_boundaries.size = "<<dynamic_obstacle_sl_boundaries.size();
  if ((dynamic_obstacle_boxes.empty() && FLAGS_use_predict_info_in_piecewise_jerk_path) ||
    (dynamic_obstacle_sl_boundaries.empty() && !FLAGS_use_predict_info_in_piecewise_jerk_path)) {
    CalculateBehaviorDynamicObstacleBound();
    return true;
  }

  double time_stamp = 0.0;
  auto init_sl_point = reference_line_info_->init_sl_point();
  double s_buffer = 1.0;
  for (auto& bound : *dynamic_bound) {
    double current_s = std::get<0>(bound);
    for (size_t index = 0; index < num_of_time_stamps;
         ++index, time_stamp += eval_time_interval) {

      common::SpeedPoint speed_point;
      speed_point = heuristic_speed.at(index);
      double ref_s = speed_point.s() + init_sl_point.s();
      
      if (ref_s < current_s - s_buffer) {
        continue;
      }
      if (ref_s > current_s + s_buffer) {
        break;
      }
      if (FLAGS_use_predict_info_in_piecewise_jerk_path) {
        for (const auto &obstacle_trajectory : dynamic_obstacle_boxes) {
          if (index > obstacle_trajectory.size() - 1) continue;
          SLBoundary sl_boundary;
          if (reference_line_info_->reference_line().GetSLBoundary(obstacle_trajectory.at(index),
                       &sl_boundary)) {
            
            if (sl_boundary.end_s() < current_s || sl_boundary.start_s() > current_s) {
              continue;
            }  
            // AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"ref_s = "<<ref_s<<", current_s = "<<current_s; 
            // AWARN_IF(FLAGS_enable_piecewise_path_debug) <<"s =("<<sl_boundary.start_s()<<", "<<sl_boundary.end_s()<<"), "
            //              <<"l =("<<sl_boundary.start_l()<<", "<<sl_boundary.end_l()<<").";          
            if (sl_boundary.start_l()*sl_boundary.end_l() <= 0 ) {
              continue;
            } else if (sl_boundary.end_l() < 0) {
              std::get<1>(bound) = std::fmax(std::get<1>(bound),sl_boundary.end_l() 
                      + EgoInfo::instance()->vehicle_param().half_wheel); 
            } else {
              std::get<2>(bound) = std::fmin(std::get<2>(bound),sl_boundary.start_l() 
                      - EgoInfo::instance()->vehicle_param().half_wheel); 
            }           
          } else {
            AWARN_IF(FLAGS_enable_piecewise_path_debug)<<"get sl failed!!!";
          }
        }
      } else {
        for (const auto &obstacle_sl : dynamic_obstacle_sl_boundaries) {
          if (index > obstacle_sl.size() - 1) continue;
          SLBoundary sl_boundary;
          sl_boundary = obstacle_sl.at(index); 
          // AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"index = "<<index 
          //   <<", s =("<<sl_boundary.start_s()<<", "<<sl_boundary.end_s()<<"), "
          //                <<"l =("<<sl_boundary.start_l()<<", "<<sl_boundary.end_l()<<")."; 
          // if (sl_boundary.end_s() < current_s || sl_boundary.start_s() > current_s) {
          //   continue;
          // }  
          // AINFO_IF(FLAGS_enable_piecewise_path_debug)<<"ref_s = "<<ref_s<<", current_s = "<<current_s; 
          // AWARN_IF(FLAGS_enable_piecewise_path_debug) <<"s =("<<sl_boundary.start_s()<<", "<<sl_boundary.end_s()<<"), "
          //              <<"l =("<<sl_boundary.start_l()<<", "<<sl_boundary.end_l()<<").";          
          if (sl_boundary.start_l() * sl_boundary.end_l() <= 0 ) {
            continue;
          } else if (sl_boundary.end_l() < 0) {
            std::get<1>(bound) = std::fmax(std::get<1>(bound),sl_boundary.end_l() 
                 + EgoInfo::instance()->vehicle_param().half_wheel); 
          } else {
            std::get<2>(bound) = std::fmin(std::get<2>(bound),sl_boundary.start_l() 
                 - EgoInfo::instance()->vehicle_param().half_wheel); 
          }           
          
        }
      }
    }
  }
  CalculateBehaviorDynamicObstacleBound();
  if (FLAGS_enable_piecewise_path_debug) {
    std::cout<<"dynamic_bound================================"<<std::endl;
    bound_adjustment_ptr_->PathBoundsDebugString(*dynamic_bound, __LINE__);
  }
  return true;
}

bool 
PathBoundGenerator::CalculateBehaviorDynamicObstacleBound() { 
  const auto& behavior_slt_boxes = BehaviorParser::instance()->decision_dynamic_obstacle_boxes();
  AINFO_IF(FLAGS_enable_debug_motion)<<"behavior_slt_boxes: "<<behavior_slt_boxes.size();
  if(behavior_slt_boxes.empty()){return true;}
  std::vector<SLBoundary> sl_boundary_vec;
  for (const auto &obstacle : behavior_slt_boxes) {
    SLBoundary sl_boundary;
    if (reference_line_info_->reference_line().GetSLBoundary(obstacle, &sl_boundary)) {
      sl_boundary_vec.emplace_back(sl_boundary);
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"sl_boundary_vec: "<<sl_boundary_vec.size();
  AINFO_IF(FLAGS_enable_debug_motion)<<"dynamic_bound_: "<<dynamic_bound_.size();
  for (auto& bound : dynamic_bound_) {
    double current_s = std::get<0>(bound);
    //优化目标的动态障碍边界里补充规划slt的结果<其实可以将决策结果按照时间排序，这里就找对应时间的就好了，不需要遍历>
    for (const auto &sl_boundary : sl_boundary_vec) {
      if (sl_boundary.end_s() < current_s || sl_boundary.start_s() > current_s) {
        // AINFO<<"current_s: "<<current_s<<", "<<sl_boundary.start_s()<<", "<<sl_boundary.end_s();
        continue;
      }          
      if (sl_boundary.start_l() * sl_boundary.end_l() <= 0 ) {
        // AINFO<<"current_s: "<<current_s<<", "<<sl_boundary.start_l()<<", "<<sl_boundary.end_l();
        std::get<2>(bound) = std::fmin(std::get<2>(bound),sl_boundary.end_l() - EgoInfo::instance()->vehicle_param().half_wheel); 
        // continue;
      } else if (sl_boundary.end_l() < 0) {
        std::get<1>(bound) = std::fmax(std::get<1>(bound),sl_boundary.start_l() + EgoInfo::instance()->vehicle_param().half_wheel); 
      } else {
        std::get<2>(bound) = std::fmin(std::get<2>(bound),sl_boundary.end_l() - EgoInfo::instance()->vehicle_param().half_wheel); 
      }           
    }
    //如果决策希望路径尽量向右侧，则在没有动态障碍物的位置，将左侧边界赋值，以达到向右侧优化目标的目的
    // AINFO<<"current_s: "<<current_s<<", decision_expect_right_most: "<<BehaviorParser::instance()->decision_expect_right_most();
    if(BehaviorParser::instance()->decision_expect_right_most()){
      if(fabs(std::get<1>(bound)) > 1e3){//如果右侧没有动态障碍物造成的边界变化
        std::get<2>(bound) = std::fmin(std::get<2>(bound), 5.0); //5.0需要小于kernel里面设置的阈值，目前是10.0
      }
    }
  } 
    
  return true;
}


}  // namespace planning
}  // namespace acu
