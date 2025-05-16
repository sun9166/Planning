/**
 * @file behavior_parser.h
 **/

#pragma once

#include <vector>
#include <map>
#include "vectormap.h"
#include "src/execution/motionplan/common/macro.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/datatype.h"
#include "datapool/include/decision_typedef.h"
#include "common/math/vec2d.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"
using namespace acu::vectormap;
namespace acu {
namespace planning {

class BehaviorParser {
 public:
  ~BehaviorParser() = default;

  void Reset();

  void ResetDecisionHistory();

  bool Update(const DecisionInfo& decision,const StructReferenceLineInfo* cognition_info);

  bool GetDecisionSpeedlimit(int giveway_id, const double speed_limit, const double init_speed);

  void GetRemoteDriveInfo(const common::TrajectoryPoint& init_point);
  
  bool AddVirtualObstacle(const std::string stop_wall_id,const bool enable_relative_distance,
                const common::math::Box2d& stop_wall_box,const double stop_distance,
                const StopReasonCode reason);
  bool AddVirtualObstacle(const VirtualObstacle stop_wall);
  bool DeleteVirtualObstacle(const StopReasonCode stop_reason);
  bool DeleteVirtualObstacle(const std::string stop_wall_id);              
  bool UpdateVirtualObstacleList(const vector<VirtualObstacle> stop_wall_infos);
  bool has_yield_or_overtake_behavior() const;
  bool has_follow_behavior() const;

  vector<VirtualObstacle> stop_wall_infos() const;
  int current_reference_line_id() const {
    return current_reference_line_id_;
  }

  int target_reference_line_id() const {
    return target_reference_line_id_;
  }

  bool is_restart() const{
    return is_restart_;
  }

  bool is_restart_by_green_light() const{
    return restart_by_green_light_;
  }

  bool has_lateral_behavior() const;

  bool has_decision_lateral_behavior() const;

  const std::map<int,LanePosition>& reference_lines() const {
    return reference_lines_;
  }

  const vector<ReferenceLineFrame>& reference_lines_data() const {
    return reference_lines_data_;
  }

  const vector<ReferenceLineFrame>* reference_lines_data_ptr() const {
    return &reference_lines_data_;
  }

  std::pair<eActionEnum,eDirectionEnum> CurrentLateralAction() const {
    return lateral_action_;
  }

  const PathBound& path_bound() const {
    return decision_path_bound_;
  }

  const std::vector<common::SLPoint>& pull_over_sl() const {
    return pull_over_sl_;
  }

  const std::vector<common::math::Box2d>& pull_over_box() const {
    return pull_over_box_;
  }

  void set_pull_over_box(const common::math::Box2d& box) {
    pull_over_box_.push_back(box);
  }

  const std::unordered_map<int, eObjectDecisionEnum>& object_decision() const {
    return obstacle_decisions_;
  }
  vector<tuple<double,double,double>> decision_speed_limits() const {
    return decision_speed_limits_;
  }
  int meeting_target_id() const {
    return meeting_id_;
  }

  std::map<int, common::math::Box2d> remote_stop() const{
    return remote_stop_boxes_;
  }
  void Set_pull_over_front_pos(common::math::Vec2d pull_over_front_pos){
    pull_over_front_pos_ = pull_over_front_pos;
   }
  common::math::Vec2d Get_pull_over_front_pos(){
    return  pull_over_front_pos_;
  }

  bool add_remote_stop(int label,const common::math::Box2d& stop);
  void cancel_remote_stop() {
    remote_stop_boxes_.clear();
  }

  bool IsObstacleDecisionFailed() {
    if (is_decision_success_ < 0) {
      return true;
    }
    return false;
  }

  int ObstacleDecisionState() {
    return is_decision_success_;
  }

  bool IsAutoDriveStart() {
    return is_start_auto_drive_;
  }

  std::string error_msg() const {
    return behavior_parse_failed_msg_;
  }

  bool is_fisrt_time_slow_down();
  std::pair<double,double> closed_dynamic_obstacle() const; 

  bool target_reference_line_change() const {
    return target_ref_line_change_;
  }

  int lanechange_prepare_direction() const {
    return lanechange_prepare_direction_;
  }

  std::pair<int, int> TargetGap() const {
    return lane_change_target_gap_;
  }

  const ReferenceLineFrame& target_reference_line() const {
    return target_reference_line_;
  }

  bool congestion_senario() const {
    return is_congestion_;
  }

  bool is_local_path_in_current() const {
    return last_local_path_in_current_;
  }

  int characteristic_obstacle_id() const {
    return characteristic_obstacle_id_;
  }

  double dis_to_curb() const {
    return dis_to_curb_;
  }
  double pull_over_modified_l() const{
    return pull_over_modified_l_;
  }
  double set_pull_over_modified_l(double pull_over_modified_l) {
    return pull_over_modified_l_ = pull_over_modified_l;
  }
  bool plan_in_lane() const {
    return plan_in_lane_;
  }

  const LaneBorrowInfo GetLaneBorrowInfo() const {
    return lane_borrow_info_;
  }

  NewPathType local_path_type () const {
    return local_path_type_; 
  }

  int abandon_lc_level() const {
    return abandon_lc_level_;
  }
  bool has_pull_over_request() const {
    return pullover_line_id_ > 0 ? false : true; 
  }
  
  const std::vector<common::math::Box2d>& decision_dynamic_obstacle_boxes() const {
    return decision_dynamic_obstacle_boxes_;
  }

  const double& decision_expand_l() const {
    return decision_expand_l_;
  }

  const double& decision_recommended_a() const {
    return decision_recommended_a_;
  }

  const bool& decision_expect_right_most() const {
    return decision_expect_right_most_;
  }

  const std::set<int>& decision_meeting_ids() const {
    return decision_meeting_ids_;
  }
  bool is_stop_or_suspend_task() const {
    return is_stop_or_suspend_task_ ; 
  }


 private:
  bool CreateReferenceLineData(const StructReferenceLineInfo* ref_line_info); 
  bool GetNearestPoint(const geometry::SiteVec& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral);
  bool FindTargetRefLine(const StructReferenceLineInfo* ref_line_info);
  void SetDirection();
  void ExtendPath( geometry::SiteVec& ref_path_points,
    const bool reverse_flag, const double& extend_length);
  void GetRemoteDriveInfo();
  void AddMissionPointStopWall();
  bool AvoidObstacleDecision(const ReferenceLineFrame* local_ref_line, eActionEnum& action_reason) ;
  void GetNewPathType();
  int GetTargetLineId(const DecisionInfo& decision);

  vector<VirtualObstacle> stop_walls_;
  int current_reference_line_id_ = 0;
  int target_reference_line_id_ = 0;
  int target_reference_line_id_history_ = 0;
  bool target_ref_line_change_ = false;
  bool is_restart_ = false ;
  bool restart_by_green_light_ = false;
  bool is_congestion_ = false;
  int characteristic_obstacle_id_ = -1;
  std::map<int,LanePosition> reference_lines_;
  ReferenceLineFrame target_reference_line_;
  LanePosition target_position_ = LanePosition::CURRENT;
  vector<ReferenceLineFrame> reference_lines_data_;
  std::pair<eActionEnum,eDirectionEnum> lateral_action_= 
      std::make_pair(eActionEnum::DEFAULT_VALUE,eDirectionEnum::DEFAULT_VALUE);
  PathBound decision_path_bound_;
  LaneBorrowInfo lane_borrow_info_;
  std::vector<common::SLPoint> pull_over_sl_;
  double pull_over_modified_l_ = 99;
  std::vector<common::math::Box2d> pull_over_box_;
  std::map<int, common::math::Box2d> remote_stop_boxes_;
  bool is_stop_history_ = false;
  std::unordered_map<int, eObjectDecisionEnum> obstacle_decisions_;  
  double speed_limit_ = 100; 
  geometry::Site speed_limit_point_;
  double decision_speed_limit_ = 100;
  vector<tuple<double,double,double>> decision_speed_limits_;
  int meeting_id_ = -1; 
  int is_decision_success_ = 1;
  bool is_start_auto_drive_ = false;
  std::string behavior_parse_failed_msg_ = ""; 
  double current_line_block_dis_ = 1000;
  double history_line_block_dis_ = 1000; 
  int block_obs_id_ = -1;
  int history_block_obs_id_ = -1;
  bool last_local_path_in_current_ = false;
  bool has_decision_lateral_behavior_ = false;
  int lanechange_prepare_direction_ = 0;
  std::pair<int, int> lane_change_target_gap_ = std::make_pair(-2,-1);
  double dis_to_curb_ = 0.5;
  bool plan_in_lane_ = true;//是否为车道内避障 
  NewPathType local_path_type_ = NewPathType::NONE;
  int abandon_lc_level_ = 0;
  int pullover_line_id_ = 100;
  //简化版动态障碍物决策结果
  std::vector<common::math::Box2d> decision_dynamic_obstacle_boxes_;
  double decision_expand_l_ = 0.0, decision_recommended_a_ = std::numeric_limits<double>::max();
  bool decision_expect_right_most_;//决策是否希望尽量向右侧偏移
  std::set<int> decision_meeting_ids_;
  bool last_stop_or_suspend_task_=false; //上一帧是否为暂停任务/暂停任务
  bool is_stop_or_suspend_task_=false;//上一帧不是暂停/停止任务，当前帧为暂停/停止任务时，该值为true
  common::math::Vec2d pull_over_front_pos_;

  DECLARE_SINGLETON(BehaviorParser)
};

}  // namespace planning
}  // namespace acu
