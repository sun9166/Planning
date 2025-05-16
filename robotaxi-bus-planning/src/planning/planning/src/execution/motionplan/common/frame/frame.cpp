/**
 * @file frame.cpp
 **/

#include <algorithm>
#include <limits>
#include "frame.h"
#include "common/math/vec2d.h"
#include "src/execution/motionplan/common/path/path.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/cognition/struct_cognition/conf/cognition_gflags.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "src/execution/motionplan/common/new_path_manager.h"

namespace acu {
namespace planning {

using acu::common::ErrorCode;
using acu::common::Status;
using acu::common::math::Box2d;
using acu::common::math::Polygon2d;

constexpr double kMathEpsilon = 1e-8;

FrameHistory::FrameHistory()
    : IndexedQueue<uint32_t, Frame>(FLAGS_max_history_frame_num) {}

Frame::Frame(uint32_t sequence_num)
    : sequence_num_(sequence_num){
  init_data_ = false;
}

Frame::Frame(uint32_t sequence_num, 
             const std::vector<common::TrajectoryPoint> &stitching_trajectory,
             const std::vector<common::TrajectoryPoint> &stitching_trajectory_speedplan,
             int start_point_type,
             const double start_time, const VehicleState &vehicle_state)
    : sequence_num_(sequence_num), 
      stitching_trajectory_for_path_plan_(stitching_trajectory),
      planning_start_point_type_(start_point_type),
      start_time_(start_time),
      vehicle_state_(vehicle_state),
      path_stitching_start_index_(stitching_trajectory_speedplan.size()),
      init_data_(true) {
  if (stitching_trajectory.size() > 0) {
    if (FLAGS_enable_two_stitching_trajectory 
       && stitching_trajectory.size() > path_stitching_start_index_) {
      stitching_trajectory_ = stitching_trajectory_speedplan;
    } else {
      stitching_trajectory_ = stitching_trajectory;
    }
    planning_start_point_ = stitching_trajectory_.back();

    size_t ego_index = 0;
    double buffer = 1.0e-6;
    double dist_sqr_min = std::numeric_limits<double>::max();
    for (size_t i = 0; i < stitching_trajectory_.size(); ++i) {
      const common::math::Vec2d curr_point(stitching_trajectory_[i].path_point().x(),
                                           stitching_trajectory_[i].path_point().y());
      const common::math::Vec2d ego_position(vehicle_state.x, vehicle_state.y);
      const double dist_sqr = curr_point.DistanceSquareTo(ego_position);
      if (dist_sqr < dist_sqr_min + buffer) {
        dist_sqr_min = dist_sqr;
        ego_index = i;
      }
    }
    planning_start_point_time_ = stitching_trajectory_.back().relative_time() - stitching_trajectory_.at(ego_index).relative_time();
  } else {
    planning_start_point_time_ = 0.0;
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"planning_start_point_time_ = "<<planning_start_point_time_;
}

const common::TrajectoryPoint &Frame::PlanningStartPoint() const {
  return planning_start_point_;
}

size_t Frame::PathStitchingStartIndex() const {
  return path_stitching_start_index_;
}


double Frame::PlanningStartPointTime() const {
  return planning_start_point_time_;
}

const VehicleState &Frame::vehicle_state() const {
  return vehicle_state_;
}

bool Frame::CreateReferenceLineInfo(
    const vector<ReferenceLineFrame>& ref_lines){
  reference_line_info_ptr_.reset();
  bool has_valid_reference_line = false;
  int count_reference_line = 0;
  for (const auto& ref_line : ref_lines) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"ref_line["<<count_reference_line<<"] path size "
                      <<ref_line.mapinfo.path_points.size();
    ++count_reference_line;
    if (ref_line.mapinfo.path_points.size() < 3) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"intention !! cognition input path points size < 3 !";
      continue;
    }
    bool is_change_path = false;

    std::pair<eActionEnum,eDirectionEnum> lateral_action = 
          std::make_pair(eActionEnum::DEFAULT_VALUE,eDirectionEnum::DEFAULT_VALUE);
    LanePosition position = LanePosition::CURRENT; 
    auto iter = BehaviorParser::instance()->reference_lines().find(ref_line.reference_lane_id);
    if (iter != BehaviorParser::instance()->reference_lines().end()) {
       position = iter->second;
    }

    if (ref_line.reference_lane_id == 
        BehaviorParser::instance()->target_reference_line_id()) {
      lateral_action = BehaviorParser::instance()->CurrentLateralAction();
    }

    if (BehaviorParser::instance()->target_reference_line_id() 
        != BehaviorParser::instance()->current_reference_line_id()) {
      is_change_path = true;
    }
    bool reverse_flag = !ref_line.mapinfo.path_points.empty()
          && ref_line.mapinfo.path_points.front().reverse;
    AERROR_IF(FLAGS_enable_debug_motion)<<">>>>>>>>>>>>> reverse_flag "<<reverse_flag;
    reference_line_info_ptr_.reset(new ReferenceLineInfo(vehicle_state_, planning_start_point_,
                                      stitching_trajectory_for_path_plan_.back(),
                                      ReferenceLine(hdmap::Path(&ref_line),is_change_path,
                                      ref_line.mapinfo.expected_speeds),
                                      std::to_string(ref_line.reference_lane_id),
                                      position,lateral_action, planning_start_point_time_,reverse_flag));
    reference_line_info_ptr_->SetLaneBorrowInfo(BehaviorParser::instance()->GetLaneBorrowInfo());
    constexpr double kPriorityHigh = 1.0;
    constexpr double kPriorityLow = 0.0;
    
    if (ref_line.reference_lane_id == 
         BehaviorParser::instance()->target_reference_line_id()) {
      reference_line_info_ptr_->SetDecisionPathBound(BehaviorParser::instance()->path_bound());
      if (!BehaviorParser::instance()->pull_over_sl().empty()) {
        StopPoint pull_over_point ;
        pull_over_point.set_s(BehaviorParser::instance()->pull_over_sl().front().s()); 
        pull_over_point.set_l(BehaviorParser::instance()->pull_over_sl().front().l()); 
        reference_line_info_ptr_->SetStopPoint(pull_over_point);
      }
      reference_line_info_ptr_->SetPriority(kPriorityHigh);
    } else {
      reference_line_info_ptr_->SetPriority(kPriorityLow);
    }

    if (ref_line.mapinfo.expected_speeds.empty()) {
      reference_line_info_ptr_->SetCruiseSpeed(8.3);
      AWARN_IF(FLAGS_enable_debug_motion)<<"ref_line.mapinfo.expected_speeds is empty !!!!"; 
    } else {
      reference_line_info_ptr_->SetCruiseSpeed(ref_line.mapinfo.expected_speeds.front().second);
    }
    reference_line_info_ptr_->reference_line_ptr()->CurvatureSpeedLimit();
    AINFO_IF(FLAGS_enable_debug_motion) << " reference line cruise speed: " 
            <<reference_line_info_ptr_->reference_line().cruise_speed()
            << ", length: " 
            << reference_line_info_ptr_->reference_line().Length();
    std::vector<std::string> target_lane_ids;
    if (!ref_line.mapinfo.front_lane_ids.empty()) {
      for (auto& lane_id : ref_line.mapinfo.front_lane_ids) {
        target_lane_ids.push_back(lane_id);
      }
      reference_line_info_ptr_->SetTargetLaneId(target_lane_ids);
    }
    
    ThreadSafeIndexedObstacles obstacles_ref_line, whole_obstacles_ref_line;
    double cruise_speed = reference_line_info_ptr_->reference_line().cruise_speed();
    std::vector<LineObject> focus_obstacle_list;
    for (auto it = ref_line.objects_.begin(); it != ref_line.objects_.end(); it++) {
      focus_obstacle_list.push_back(it->second);
    }

    for (auto &ptr :
      Obstacle::CreateObstacles(focus_obstacle_list, planning_start_point_time_)) {
      whole_obstacles_ref_line.Add((*ptr).Id(), *ptr);
    }

    size_t num = 0;
    for (const auto& keep_clear_zone : ref_line.mapinfo.distance_to_forbid_areas) {
      std::string virtual_obstacle_id = "keep_clear_"+std::to_string(num);
      BuildKeepClearObstacle(virtual_obstacle_id, keep_clear_zone.first,keep_clear_zone.second);
      num++;
    }

    if (!FLAGS_using_all_obstacle) {
      GetFocusObstacleList(focus_obstacle_list,ref_line.objects_,cruise_speed);
    } 

    //避免重复获取stmap
    const STmap* st_map = &(BehaviorParser::instance()->target_reference_line().st_map);
    if (!BehaviorParser::instance()->is_local_path_in_current()) {
      st_map = &(BehaviorParser::instance()->reference_lines_data().front().st_map);
      AWARN_IF(FLAGS_enable_debug_motion)<<"use local st_map";
    }
    vector<vector<STMapPointStruct>> st_map_data = st_map->map_data();
    
    for (auto &ptr :
      Obstacle::CreateObstacles(focus_obstacle_list,planning_start_point_time_)) {
      ObjectDecisionType longitudinal_decision_last;
      FindStopDecisionHistory(longitudinal_decision_last,ptr->Id());
      if (longitudinal_decision_last.has_stop()) {
        ptr->SetStopDistanceHistory(-1.0 * longitudinal_decision_last.stop().distance_s());
        AINFO_IF(FLAGS_enable_debug_motion)<<"id = "<<ptr->Id()
               <<" stop dis history = "<<longitudinal_decision_last.stop().distance_s();
      }

      if (ptr->IsStatic()) {  
        ptr->SetNudgeLatBufferMin(FindNudgeBufferHistory(ptr->Id()));
      }
      
      ptr->SetSTUpperLowerPoints(GetObstacleOverlapPointsOnSTMap(ptr->Id(), st_map_data));
      AddObstacle(*ptr);
      obstacles_ref_line.Add((*ptr).Id(), *ptr);
    }
    if (!reference_line_info_ptr_->Init(obstacles_ref_line.Items())) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to init reference line";
      continue;
    } else {
      has_valid_reference_line = true;
      reference_line_info_ptr_->AddWholeObstacles(whole_obstacles_ref_line.Items());
    }
  }

  return has_valid_reference_line;
}


bool Frame::CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
    const vector<ReferenceLineFrame>& ref_lines) {
  reference_line_info_ptr_.reset();
  bool has_valid_reference_line = false;
  int count_reference_line = 0;
  AINFO_IF(FLAGS_enable_debug_motion)<<"smoothed reference_lines size = "<<reference_lines.size();
  for (const auto& ref_line : reference_lines) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"smoothed ref_line["<<count_reference_line<<"] path size "
                      <<ref_line.reference_points().size();
    if (ref_line.reference_points().size() < 3) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"intention !! smoothed reference_line points size < 3 !";
      return false;
    }
    // bool is_change_path = false; // unused here, comment by @cc 1215

    // 1. assign lane position
    LanePosition position = LanePosition::CURRENT; 
    auto current_reference_line_id = ref_lines.at(count_reference_line).reference_lane_id;
    auto iter = BehaviorParser::instance()->reference_lines().find(current_reference_line_id);
    if (iter != BehaviorParser::instance()->reference_lines().end()) {
       position = iter->second;
    }
    // 2. assign lateral_action
    std::pair<eActionEnum,eDirectionEnum> lateral_action = 
          std::make_pair(eActionEnum::DEFAULT_VALUE,eDirectionEnum::DEFAULT_VALUE);
    if (current_reference_line_id == 
        BehaviorParser::instance()->target_reference_line_id()) {
      lateral_action = BehaviorParser::instance()->CurrentLateralAction();
    }

    // 3. assign reverse flag
    bool reverse_flag = !ref_lines.at(count_reference_line).mapinfo.path_points.empty()
      && ref_lines.at(count_reference_line).mapinfo.path_points.front().reverse;
    AERROR_IF(FLAGS_enable_debug_motion)<<">>>>>>>>>>>>> reverse_flag "<<reverse_flag;

    // 4. construct ReferenceLineInfo object
    reference_line_info_ptr_.reset(new ReferenceLineInfo(vehicle_state_, planning_start_point_,
                                                    stitching_trajectory_for_path_plan_.back(),
                                                    ref_line,
                                                    std::to_string(current_reference_line_id),
                                                    position,lateral_action, planning_start_point_time_,reverse_flag));

    // 5. set some feature for ReferenceLineInfo object
    reference_line_info_ptr_->SetLaneBorrowInfo(BehaviorParser::instance()->GetLaneBorrowInfo());
    constexpr double kPriorityHigh = 1.0;
    constexpr double kPriorityLow = 0.0;

    if (current_reference_line_id == 
         BehaviorParser::instance()->target_reference_line_id()) {
      reference_line_info_ptr_->SetDecisionPathBound(BehaviorParser::instance()->path_bound());
      if (!BehaviorParser::instance()->pull_over_sl().empty()) {
        StopPoint pull_over_point ;
        pull_over_point.set_s(BehaviorParser::instance()->pull_over_sl().front().s()); 
        pull_over_point.set_l(BehaviorParser::instance()->pull_over_sl().front().l()); 
        reference_line_info_ptr_->SetStopPoint(pull_over_point);
      }
      reference_line_info_ptr_->SetPriority(kPriorityHigh);
    } else {
      reference_line_info_ptr_->SetPriority(kPriorityLow);
    }

    if (ref_lines.at(count_reference_line).mapinfo.expected_speeds.empty()) {
      reference_line_info_ptr_->SetCruiseSpeed(8.3);
      AWARN_IF(FLAGS_enable_debug_motion)<<"ref_line.mapinfo.expected_speeds is empty !!!!"; 
    } else {
      reference_line_info_ptr_->SetCruiseSpeed(ref_lines.at(count_reference_line).mapinfo.expected_speeds.front().second);
    }
     reference_line_info_ptr_->reference_line_ptr()->CurvatureSpeedLimit();
    AINFO_IF(FLAGS_enable_debug_motion) << " reference line cruise speed: " 
            <<reference_line_info_ptr_->reference_line().cruise_speed()
            << ", length: " 
            << reference_line_info_ptr_->reference_line().Length();      
    std::vector<std::string> target_lane_ids;
    if (!ref_lines.at(count_reference_line).mapinfo.front_lane_ids.empty()) {
      for (auto& lane_id : ref_lines.at(count_reference_line).mapinfo.front_lane_ids) {
        target_lane_ids.push_back(lane_id);
      }
      reference_line_info_ptr_->SetTargetLaneId(target_lane_ids);
    }

    // 6. add obstacle info
    ThreadSafeIndexedObstacles obstacles_ref_line, whole_obstacles_ref_line;
	 double cruise_speed = reference_line_info_ptr_->reference_line().cruise_speed();
    std::vector<LineObject> focus_obstacle_list;
    for (auto it = ref_lines.at(count_reference_line).objects_.begin();
              it != ref_lines.at(count_reference_line).objects_.end(); it++) {
      focus_obstacle_list.push_back(it->second);
    }

    for (auto &ptr :
      Obstacle::CreateObstacles(focus_obstacle_list, planning_start_point_time_)) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"whole_obstacles_ref_line add: "<<ptr->Id();
      whole_obstacles_ref_line.Add((*ptr).Id(), *ptr);
    }

    size_t num = 0;
    for (const auto& keep_clear_zone : ref_lines.at(count_reference_line).mapinfo.distance_to_forbid_areas) {
      std::string virtual_obstacle_id = "keep_clear_"+std::to_string(num);
      BuildKeepClearObstacle(virtual_obstacle_id, keep_clear_zone.first,keep_clear_zone.second);
      num++;
    }

    if (!FLAGS_using_all_obstacle) {
      GetFocusObstacleList(focus_obstacle_list,ref_lines.at(count_reference_line).objects_,cruise_speed);
    }

    //避免重复获取stmap
    const STmap* st_map = &(BehaviorParser::instance()->target_reference_line().st_map);
    if (!BehaviorParser::instance()->is_local_path_in_current()) {
      st_map = &(BehaviorParser::instance()->reference_lines_data().front().st_map);
      AWARN_IF(FLAGS_enable_debug_motion)<<"use local st_map";
    }
    vector<vector<STMapPointStruct>> st_map_data = st_map->map_data();
	
    for (auto &ptr : Obstacle::CreateObstacles(focus_obstacle_list, planning_start_point_time_)) {

      ObjectDecisionType longitudinal_decision_last;
      FindStopDecisionHistory(longitudinal_decision_last,ptr->Id());
      if (longitudinal_decision_last.has_stop()) {
        ptr->SetStopDistanceHistory(-1.0 * longitudinal_decision_last.stop().distance_s());
        AINFO_IF(FLAGS_enable_debug_motion)<<"id = "<<ptr->Id()
               <<" stop dis history = "<<longitudinal_decision_last.stop().distance_s();
      }
      if (ptr->IsStatic()) {
        ptr->SetNudgeLatBufferMin(FindNudgeBufferHistory(ptr->Id()));
      }
      //对于动静态障碍物，横向让行距离更新
      if (ptr->IsMovable()) {
        double nudge_l_buffer = 0.05 * EgoInfo::instance()->vehicle_state_new().linear_velocity;//与车速相关
        nudge_l_buffer = std::max(nudge_l_buffer, FLAGS_static_decision_nudge_l_buffer);
        nudge_l_buffer = std::min(nudge_l_buffer, FLAGS_moveable_obstacle_l_buffer);
        ptr->SetNudgeLatBufferMin(nudge_l_buffer);
      }
      
      ptr->SetSTUpperLowerPoints(GetObstacleOverlapPointsOnSTMap(ptr->Id(), st_map_data));
      AddObstacle(*ptr);
      obstacles_ref_line.Add((*ptr).Id(), *ptr);
    }

    ++count_reference_line;

    // 7. Init ReferenceLineInfo object
    if (!reference_line_info_ptr_->Init(obstacles_ref_line.Items())) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to init reference line";
      return false;
    } else {
      has_valid_reference_line = true;
      reference_line_info_ptr_->AddWholeObstacles(whole_obstacles_ref_line.Items());
    }

    break;//just get front;
  }

  return has_valid_reference_line;
}

Obstacle::OverlapPoints Frame::GetObstacleOverlapPointsOnSTMap(const string& obstacle_id,
                                      const vector<vector<STMapPointStruct>>& st) const {
  Obstacle::OverlapPoints st_overlap_points;
  auto last_speed_data = GetHistorySpeed();
  STPoint upper_point, lower_point;
  common::SpeedPoint speed_point;
  speed_point.set_v(EgoInfo::instance()->vehicle_state_new().linear_velocity);
  for (int t = 0; t < st.size(); t++) {
    double time = t * FLAGS_scale_t;
    if ( time < 4 ) {
      if( !last_speed_data.EvaluateByTime(time, &speed_point) ) {
        speed_point.set_s(speed_point.s() + speed_point.v() * FLAGS_scale_t);
      }
    } else {
      speed_point.set_s(speed_point.s() + speed_point.v() * FLAGS_scale_t);
    }
    int s_max_index = std::max(0.0, ceil( (speed_point.s() + EgoInfo::instance()->vehicle_front_edge_to_center())/FLAGS_scale_s) );
    int s_min_index = std::max(0.0, floor( (speed_point.s() - EgoInfo::instance()->vehicle_back_edge_to_center())/FLAGS_scale_s) );
    for (int s = s_min_index; s < st.at(t).size() && s < s_max_index; s++) {
      if (st.at(t).at(s).p <= FLAGS_p_thd_delete) continue;
      for (const auto&obj_pair:st.at(t).at(s).objs) {
        if (obj_pair.second.p <= FLAGS_p_thd_delete) continue;
        const std::string id = std::to_string(obj_pair.first);
        if (obstacle_id == id) {
          double s_upper = s * FLAGS_scale_s;
          double s_lower = std::max(0.0, s * FLAGS_scale_s);
          upper_point.set_s(s_upper);
          upper_point.set_t(time);
          lower_point.set_s(s_lower);
          lower_point.set_t(time);
          st_overlap_points.AddPoint(upper_point);
          st_overlap_points.AddPoint(lower_point);
        } 
      }
    }
  }
  return st_overlap_points;
}

bool Frame::FindStopDecisionHistory(ObjectDecisionType& stop_desicion_last, 
                     const std::string& id) const {
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last frame is empty";
  } else {
    const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
    if (!last_reference_line_info) {
      AWARN_IF(FLAGS_enable_debug_motion) << "last reference line info is empty";
    } else {
      auto last_obstacle = last_reference_line_info->path_decision().Find(id);
      if (!last_obstacle) {
        return false;
      //cannot find this obstalce in history;
      } else {
        if (last_obstacle->HasLongitudinalDecision()) {
          auto lon_desicion_last = last_obstacle->LongitudinalDecision();
          if(lon_desicion_last.has_stop()){
            stop_desicion_last = lon_desicion_last;
            return true;
          }
        }
      }
    }
  }
  return false;
}

// speed info maybe replace as vehicle speed point when no last_frame or last_reference_line_info @cc1222 TBD
const SpeedInfo Frame::GetHistorySpeed() const {
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last frame is empty";
    return SpeedInfo();
  }
  const ReferenceLineInfo* last_reference_line_info =
    last_frame->DriveReferenceLineInfo();
  if (!last_reference_line_info) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last reference line info is empty";
    return SpeedInfo();
  }
  // AINFO_IF(FLAGS_enable_debug_motion)<< "speed size "<<last_reference_line_info->speed_data().size();
  return last_reference_line_info->speed_data();
}

double Frame::FindNudgeBufferHistory(const std::string& id) const {
  const auto* last_frame = FrameHistory::instance()->Latest();
  if (!last_frame) {
    AWARN_IF(FLAGS_enable_debug_motion) << "last frame is empty";
    return FLAGS_static_decision_nudge_l_buffer;
  } else {
    const ReferenceLineInfo* last_reference_line_info =
      last_frame->DriveReferenceLineInfo();
    if (!last_reference_line_info) {
      AWARN_IF(FLAGS_enable_debug_motion) << "last reference line info is empty";
    } else {
      auto last_obstacle = last_reference_line_info->path_decision().Find(id);
      if (!last_obstacle) {
        return FLAGS_static_decision_nudge_l_buffer;
      //cannot find this obstalce in history;
      } else {
        return last_obstacle->nudge_l_buffer_min();
      }
    }
  }
  return FLAGS_static_decision_nudge_l_buffer;
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_s) {
  if (reference_line_info == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion) << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  const double box_center_s = obstacle_s + FLAGS_virtual_stop_wall_length / 2.0;

  auto box_center = reference_line.GetReferencePoint(box_center_s);

  double heading = reference_line.GetReferencePoint(obstacle_s).heading();

  constexpr double kStopWallWidth = 4.0;
  Box2d stop_wall_box{box_center, heading, FLAGS_virtual_stop_wall_length,
                      kStopWallWidth};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
  //create a static virtual obstacle
}

/**
 * @brief: create static virtual object with lane width,
 *         mainly used for virtual stop wall
 */
const Obstacle *Frame::CreateStopObstacle(const std::string &obstacle_id,
                                          const std::string &lane_id,
                                          const double lane_s) {
  if (!hdmap_) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Invalid HD Map.";
    return nullptr;
  }
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(lane_id));
  if (!lane) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to find lane[" << lane_id << "]";
    return nullptr;
  }

  double dest_lane_s = std::max(0.0, lane_s);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  lane->GetWidth(dest_lane_s, &lane_left_width, &lane_right_width);

  Box2d stop_wall_box{{dest_point.x(), dest_point.y()},
                      lane->Heading(dest_lane_s),
                      FLAGS_virtual_stop_wall_length,
                      lane_left_width + lane_right_width};

  return CreateStaticVirtualObstacle(obstacle_id, stop_wall_box);
}

/**
 * @brief: create static virtual object with lane width,
 */
const Obstacle *Frame::CreateStaticObstacle(
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info_ptr_ == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion) << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info_ptr_->reference_line();

  // start_xy
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to get start_xy from sl ";
    return nullptr;
  }

  // end_xy
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to get end_xy from sl ";
    return nullptr;
  }

  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }

  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};

  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}

const Obstacle *Frame::CreateStaticVirtualObstacle(const std::string &id,
                                                   const Box2d &box) {
  const auto *object = obstacles_.Find(id);
  if (object) {
    AWARN_IF(FLAGS_enable_debug_motion) << "obstacle " << id << " already exist.";
    return object;
  }
  auto *ptr =
      obstacles_.Add(id, *Obstacle::CreateStaticVirtualObstacles(id, box));
  if (!ptr) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create virtual obstacle " << id;
  }
  return ptr;
}

bool Frame::Init(
    const vector<ReferenceLineFrame>& ref_lines) {

  if (ref_lines.empty()) {
    return false;
  }
  if (!CreateReferenceLineInfo(ref_lines)) {
    const std::string msg = "Failed to init reference line info.";
    AWARN_IF(FLAGS_enable_debug_motion) << msg;
    return false;
  }

  auto status = FLAGS_enable_check_collision_on_stitching_trajectory? 
                CheckCollisionOnStitchingTrajectory() : CheckCollision();
  if (!status) {
    AWARN_IF(FLAGS_enable_debug_motion) << "failed to init frame , maybe cause collision ";
    return false;
  }

  AddDestinationStopWall();
  AddVirtualStopWall();

  return true;
}

bool Frame::Init(const std::list<ReferenceLine> &reference_lines,
    const vector<ReferenceLineFrame>& raw_ref_lines) {
  if (raw_ref_lines.empty() || reference_lines.empty()) {
    return false;
  }
  if (!CreateReferenceLineInfo(reference_lines, raw_ref_lines)) {
    const std::string msg = "Failed to init reference line info.";
    AWARN_IF(FLAGS_enable_debug_motion) << msg;
    return false;
  }

  auto status = FLAGS_enable_check_collision_on_stitching_trajectory? 
                CheckCollisionOnStitchingTrajectory() : CheckCollision();
  if (!status) {
    AWARN_IF(FLAGS_enable_debug_motion) << "failed to init frame , maybe cause collision ";
    return false;
  }

  AddDestinationStopWall();
  AddVirtualStopWall();

  return true;
}

bool Frame::CheckCollision() const {
  if (FLAGS_enable_collision_detection && planning_start_point_.v() < 1e-3) {
    const auto *collision_obstacle = FindCollisionObstacle();
    if (collision_obstacle != nullptr) {
      std::string err_str =
          "Found collision with obstacle: " + collision_obstacle->Id();
      AWARN_IF(FLAGS_enable_debug_motion) << err_str;
      return false;
    }
  }
  return true;
}
bool Frame::CheckCollisionOnStitchingTrajectory() const {
  if (stitching_trajectory_.size() <= 1) return true;
  const auto *collision_obstacle = FindCollisionObstacleOnStitchingTrajectory();
  if (collision_obstacle != nullptr) {
    std::string err_str =
        "Found collision with obstacle: " + collision_obstacle->Id();
    AWARN_IF(FLAGS_enable_debug_motion) << err_str;
    return false;
  }

  return true;
}

bool Frame::GetNearestPoint(const std::vector<common::TrajectoryPoint>& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral) const {
  if (accumulate_s == nullptr || lateral == nullptr || points.empty()) {
    return false;
  }
  std::vector<common::math::LineSegment2d> segments;
  for (int i = 0; i + 1 < points.size(); ++i) {
    segments.emplace_back(common::math::Vec2d(points[i].path_point().x(),points[i].path_point().y()), 
                          common::math::Vec2d(points[i + 1].path_point().x(),points[i + 1].path_point().y()));
  }
  if (segments.empty()) {
    return false;
  }

  double num_segments = (int)points.size() - 1;
  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments; ++i) {
    const double distance = segments[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  min_distance = std::sqrt(min_distance);
  const auto& nearest_seg = segments[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == num_segments - 1) {
    *accumulate_s = points[min_index].path_point().s() + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    *accumulate_s = points[min_index].path_point().s() +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  return true;
}


void Frame::AddDestinationStopWall() {
  dis_2_virtual_wall_=9999;
  const double kDeceleration = 0.5;
  const double kPathLength = 90;
  double preview_distance = 0.5 * vehicle_state_.linear_velocity * vehicle_state_.linear_velocity / kDeceleration + 5;
  preview_distance = std::fmax(preview_distance,30.0);
  preview_distance = std::fmin(preview_distance,kPathLength);
  bool is_pull_over_path = ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::PULL_OVER ? true : false;
  common::SLPoint sl_point;
  Vec2d car_pos(vehicle_state_.x, vehicle_state_.y);
  reference_line_info_ptr_->reference_line().XYToSL(car_pos,&sl_point);
  const double vehicle_s = sl_point.s();
  AERROR_IF(vehicle_s > 20.0)<<"vehicle_s is farther than ref_line start point!!!! ,s = "<<vehicle_s;
  // common::math::Vec2d pull_over_front_pos =BehaviorParser::instance()->Get_pull_over_front_pos();
  // common::SLPoint sl_pull_over;
  // reference_line_info_ptr_->reference_line().XYToSL(pull_over_front_pos,&sl_pull_over);
  // AINFO<<"***********sl_pull_over.s: "<<sl_pull_over.s();

  if (reference_line_info_ptr_->reference_line().Length() < preview_distance || is_pull_over_path || vehicle_s > 20.0) {
    is_near_destination_ = true;
    AINFO_IF(FLAGS_enable_debug_motion)<< " is closed to destination.";
    common::math::Vec2d destination_point = reference_line_info_ptr_->reference_line().GetMapPath().path_points().back();
    common::SLPoint sl_destination;
    reference_line_info_ptr_->reference_line().XYToSL(destination_point,&sl_destination);
    // AINFO<<"***********sl_pull_over.s: "<<sl_destination.s()<<" dis_2_pull_over= "<<sl_pull_over.s() - sl_destination.s();

    Box2d destination_wall_box{destination_point,
                               reference_line_info_ptr_->reference_line().GetMapPath().path_points().back().heading(),
                               FLAGS_virtual_stop_wall_length,//0.1
                               3.5};
    auto* obstacle = CreateStaticVirtualObstacle(FLAGS_destination_obstacle_id, destination_wall_box);
    Obstacle* stop_obstacle = reference_line_info_ptr_->AddObstacle(obstacle);
    if (!stop_obstacle) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create obstacle for: " << FLAGS_destination_obstacle_id;
    } else {
      const double stop_distance = -FLAGS_pull_over_extend_s;//0.01;
      auto stop_point = reference_line_info_ptr_->reference_line().GetReferencePoint(
                          stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance);
      ObjectDecisionType stop;
      auto stop_decision = stop.mutable_stop();
      stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);//所有停车行为都当成红灯停车
      stop_decision->set_distance_s(-stop_distance);
      stop_decision->set_stop_heading(stop_point.heading());
      stop_decision->mutable_stop_point()->set_x(stop_point.x());
      stop_decision->mutable_stop_point()->set_y(stop_point.y());
      stop_decision->mutable_stop_point()->set_z(0.0);
      auto* path_decision = reference_line_info_ptr_->path_decision();
      path_decision->AddLongitudinalDecision("destination stop ", FLAGS_destination_obstacle_id, stop);
    }
  }
}

void Frame::AddVirtualStopWall() {
  auto bp_ptr = BehaviorParser::instance();
  if (!bp_ptr->stop_wall_infos().empty()) {
    for (auto& stop_wall : bp_ptr->stop_wall_infos()) {
      SLBoundary stop_wall_sl;
      if (reference_line_info_ptr_->reference_line().GetSLBoundary(stop_wall.obstacle_box, 
                                     &stop_wall_sl)) {
        double min_l = std::fmin(std::fabs(stop_wall_sl.start_l()), std::fabs(stop_wall_sl.end_l()));
        if (stop_wall_sl.start_l() * stop_wall_sl.end_l() > 0 && min_l > 4.0) {
          continue;
        }
      } else {
        continue;
      }
      auto* obstacle = CreateStaticVirtualObstacle(stop_wall.id, stop_wall.obstacle_box);
      Obstacle* stop_obstacle = reference_line_info_ptr_->AddObstacle(obstacle);
      if (!stop_obstacle) {
        AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create obstacle for: " << FLAGS_destination_obstacle_id;
      } else {
        AINFO<<"VirtualStop_start_s: "<<stop_obstacle->PerceptionSLBoundary().start_s();
        const double stop_distance = 0.01;
        auto stop_point = reference_line_info_ptr_->reference_line().GetReferencePoint(
                            stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance);
        dis_2_virtual_wall_=std::fmin(stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance,dis_2_virtual_wall_);
        ObjectDecisionType stop;
        auto stop_decision = stop.mutable_stop();
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_SIGNAL);//所有停车行为都当成红灯停车
        stop_decision->set_distance_s(-stop_distance);
        stop_decision->set_stop_heading(stop_point.heading());
        stop_decision->mutable_stop_point()->set_x(stop_point.x());
        stop_decision->mutable_stop_point()->set_y(stop_point.y());
        stop_decision->mutable_stop_point()->set_z(0.0);
        auto* path_decision = reference_line_info_ptr_->path_decision();
        path_decision->AddLongitudinalDecision("Red light stop ",stop_wall.id, stop);
      }
    }
  }

  if (!bp_ptr->remote_stop().empty()) {
    for (auto& stop_wall : bp_ptr->remote_stop()) {
      std::string stop_id = stop_wall.first == 1? "SlightlyBrake":"HardBrake";
      auto* obstacle = CreateStaticVirtualObstacle(stop_id, stop_wall.second);
      Obstacle* stop_obstacle = reference_line_info_ptr_->AddObstacle(obstacle);
      if (!stop_obstacle) {
        AWARN_IF(FLAGS_enable_debug_motion) << "Failed to create obstacle for: " << stop_id;
      } else {
        const double stop_distance = 0.01;
        auto stop_point = reference_line_info_ptr_->reference_line().GetReferencePoint(
                            stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance);
        dis_2_virtual_wall_=std::fmin(stop_obstacle->PerceptionSLBoundary().start_s() - stop_distance,dis_2_virtual_wall_);
        ObjectDecisionType stop;
        auto stop_decision = stop.mutable_stop();
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_SIGNAL);//所有停车行为都当成红灯停车
        stop_decision->set_distance_s(-stop_distance);
        stop_decision->set_stop_heading(stop_point.heading());
        stop_decision->mutable_stop_point()->set_x(stop_point.x());
        stop_decision->mutable_stop_point()->set_y(stop_point.y());
        stop_decision->mutable_stop_point()->set_z(0.0);
        auto* path_decision = reference_line_info_ptr_->path_decision();
        path_decision->AddLongitudinalDecision("remote stop ",stop_id, stop);
      }
    }
  }
  EgoInfo::instance()->UpdateDisToVirtualWall(dis_2_virtual_wall_);
}

const Obstacle *Frame::FindCollisionObstacle() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }

  const auto &adc_polygon = Polygon2d(EgoInfo::instance()->ego_box());
  for (const auto &obstacle : obstacles_.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }

    const auto &obstacle_polygon = obstacle->PerceptionPolygon();
    if (obstacle_polygon.HasOverlap(adc_polygon)) {
      return obstacle;
    }
  }
  return nullptr;
}

const Obstacle *Frame::FindCollisionObstacleOnStitchingTrajectory() const {
  if (obstacles_.Items().empty()) {
    return nullptr;
  }
  double stitching_trajectory_length = stitching_trajectory_.back().path_point().s() - 
                                       stitching_trajectory_.front().path_point().s();
  AINFO_IF(FLAGS_enable_debug_motion)<<"stitching_trajectory_length = "<<stitching_trajectory_length;
  for (const auto &obstacle : obstacles_.Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (obstacle->PerceptionSLBoundary().start_s() > stitching_trajectory_length 
      || obstacle->PerceptionSLBoundary().end_s() < 0) {
      continue;
    }
    double start_s(std::numeric_limits<double>::max());
    double end_s(std::numeric_limits<double>::lowest());
    double start_l(std::numeric_limits<double>::max());
    double end_l(std::numeric_limits<double>::lowest());

    for (const auto& point : obstacle->PerceptionPolygon().points()) {
      double nearest_s = 0.0;
      double nearest_l = 0.0;
      if (!GetNearestPoint(stitching_trajectory_, point, &nearest_s, &nearest_l)) {
        AWARN_IF(FLAGS_enable_debug_motion)<< "failed to get projection for point: "
                        << " on stitching_trajectory.";
        continue;
      }
      start_s = std::fmin(start_s, nearest_s);
      end_s = std::fmax(end_s, nearest_s);
      start_l = std::fmin(start_l, nearest_l);
      end_l = std::fmax(end_l, nearest_l);
    }
    AWARN_IF(FLAGS_enable_debug_motion)<< "FindCollisionObstacleOnStitchingTrajectory s = ("
                        <<start_s<<", "<<end_s <<"), l =("<<start_l<< ", "<<end_l<<").";
    if (start_s > stitching_trajectory_length || end_s < 0 ) {
      continue;
    }
    if (start_l * end_l <= 0 || 
        (start_l * end_l > 0 && std::min(fabs(start_l),fabs(end_l)) 
            <= EgoInfo::instance()->vehicle_param().half_wheel + FLAGS_static_decision_nudge_l_buffer)) {
      return obstacle;
    }
  }
  return nullptr;
}

uint32_t Frame::SequenceNum() const { return sequence_num_; }

Obstacle *Frame::Find(const std::string &id) { return obstacles_.Find(id); }

void Frame::AddObstacle(const Obstacle &obstacle) {
  obstacles_.Add(obstacle.Id(), obstacle);
}

const ReferenceLineInfo *Frame::FindDriveReferenceLineInfo() {
  drive_reference_line_info_ =  reference_line_info_ptr_.get();
  return drive_reference_line_info_;
}

const ReferenceLineInfo *Frame::DriveReferenceLineInfo() const {
  return drive_reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
  return obstacles_.Items();
}

bool Frame::GetFocusObstacleList(vector<LineObject>& obstacle_list, 
    const std::map<int, LineObject>& obstacle_input, const double cruise_speed) const {
  obstacle_list.clear();
  const double speed_buffer = 0.05;
  bool need_new_path = (BehaviorParser::instance()->has_lateral_behavior() 
                          || !NewPathManager::instance()->stitch_state())? true : false;
  CarModel vehicle_param = EgoInfo::instance()->vehicle_param(); 
  auto ignore_s_min = EgoInfo::instance()->vehicle_front_edge_to_center();
  double ignore_s_max = EgoInfo::instance()->vehicle_state_new().linear_velocity * 2.0;
  double ignore_speed = (1.0 + speed_buffer) * cruise_speed;
  const double half_width = vehicle_param.half_wheel;
  if (obstacle_input.empty()) {
    return true;
  }
  for (const auto& it : obstacle_input) {
    auto &obstacle = it.second;
    auto bp_ptr = BehaviorParser::instance();
    const double path_length = 
       bp_ptr->reference_lines_data().empty() ? 0.0 : bp_ptr->reference_lines_data().front().Length(); 

    auto iterater = bp_ptr->object_decision().find(obstacle.id);
    if ((iterater == bp_ptr->object_decision().end() || (*iterater).second == eObjectDecisionEnum::IGNORE) 
        && !obstacle.is_static) {//decision ignore and is dynamic 
      if (FLAGS_enable_realtime_avoid_obstacle_in_lane && bp_ptr->has_lateral_behavior()) {
        double min_l = std::fmin(fabs(obstacle.sl_boundary.max_l),fabs(obstacle.sl_boundary.min_l));
        if (obstacle.speed >= ignore_speed
           || obstacle.sl_boundary.max_s <= ignore_s_min || obstacle.sl_boundary.min_s >= ignore_s_max //Longitudinal filter
           || obstacle.sl_boundary.max_l*obstacle.sl_boundary.min_l < 0 ////Lateral filter
           || (FLAGS_dynamic_obstacle_lateral_ignore_buffer + 1.0) + half_width < obstacle.sl_boundary.min_l
           || - half_width - (FLAGS_dynamic_obstacle_lateral_ignore_buffer + 1.0) > obstacle.sl_boundary.max_l
           || (min_l <= half_width && obstacle.sl_boundary.max_l*obstacle.sl_boundary.min_l >= 0)) {
          if (bp_ptr->CurrentLateralAction().first == eActionEnum::PULL_OVER) {//靠边停车时考虑所有动态障碍物
            obstacle_list.push_back(obstacle);
          } else {
            if (obstacle.sl_polygons.empty()) {
              continue;
            } else {
              obstacle_list.push_back(obstacle);
            }
          }
        } else {
          obstacle_list.push_back(obstacle);
        }
      } else {
        if (obstacle.sl_polygons.empty()) {
          continue;
        } else {
          obstacle_list.push_back(obstacle);
        }
      }
    } else if (obstacle.is_static) {//is static
      if (!need_new_path) {
        double min_l = std::fmin(fabs(obstacle.sl_boundary.max_l),fabs(obstacle.sl_boundary.min_l));
        double nudge_l_buffer = obstacle.is_moveble ? 
                                FLAGS_lateral_ignore_buffer
                                : FLAGS_static_decision_nudge_l_buffer;
        if (obstacle.sl_boundary.max_s <= 0.0 || obstacle.sl_boundary.min_s > path_length + FLAGS_exceed_path_length_buffer) {
          if (obstacle.sl_polygons.empty()) {
            continue;
          } else {
            obstacle_list.push_back(obstacle);
          }
        } else if (obstacle.sl_boundary.max_l * obstacle.sl_boundary.min_l >= 0 
              && min_l > half_width + nudge_l_buffer + 0.1) {
          if (obstacle.sl_polygons.empty() && (obstacle.id >= 0) ) {
            continue;
          } else {
            obstacle_list.push_back(obstacle);
          }
        } else {
          obstacle_list.push_back(obstacle);
        }
      } else {
        double min_l = std::fmin(fabs(obstacle.sl_boundary.max_l),fabs(obstacle.sl_boundary.min_l));
        if (obstacle.sl_boundary.max_s <= 0.0 ) {
          if (obstacle.sl_polygons.empty()) {
            continue;
          } else {
            obstacle_list.push_back(obstacle);
          }
        } else if (obstacle.sl_boundary.max_l * obstacle.sl_boundary.min_l >= 0 
              && min_l > half_width + FLAGS_lateral_ignore_buffer + 0.1
              && !(bp_ptr->CurrentLateralAction().first == eActionEnum::PULL_OVER)) {
          if (obstacle.sl_polygons.empty()) {
            continue;
          } else {
            obstacle_list.push_back(obstacle);
          }
        }else if((bp_ptr->CurrentLateralAction().first == eActionEnum::PULL_OVER)
              && obstacle.sl_boundary.max_l * obstacle.sl_boundary.min_l >= 0
              && min_l > fabs(BehaviorParser::instance()->pull_over_sl()[0].l()) + half_width){
          continue;
        } else {
          obstacle_list.push_back(obstacle);
        }
      }
    } else { //is dynamic
      obstacle_list.push_back(obstacle);
    }
  }
  return true;
}

bool Frame::BuildKeepClearObstacle(
  const std::string& virtual_obstacle_id, const double keep_clear_start_s,
  const double keep_clear_end_s) {

  // check
  const double min_pass_s_distance = 3.0;//最多允许停过路口3m
  const double adc_front_edge_s = reference_line_info_ptr_->AdcSlBoundary().end_s();
  if (adc_front_edge_s - keep_clear_start_s > min_pass_s_distance) {
    AINFO_IF(FLAGS_enable_debug_motion) << "adc inside keep_clear zone[" << virtual_obstacle_id << "] s["
           << keep_clear_start_s << ", " << keep_clear_end_s
           << "] adc_front_edge_s[" << adc_front_edge_s
           << "]. skip this keep clear zone";
    return false;
  }

  ADEBUG << "keep clear obstacle: [" << keep_clear_start_s << ", "
         << keep_clear_end_s << "]";
  // create virtual static obstacle
  auto* obstacle =
      CreateStaticObstacle(virtual_obstacle_id,  keep_clear_start_s, keep_clear_end_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  auto* path_obstacle = reference_line_info_ptr_->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << virtual_obstacle_id;
    return false;
  }
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::KEEP_CLEAR;
  path_obstacle->SetReferenceLineStBoundaryType(b_type);

  return true;
}

}  // namespace planning
}  // namespace acu
