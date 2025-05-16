/**
 * @file reference_line_info.h
 **/

#pragma once

#include <limits>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pnc_point.pb.h"
#include "motionplanning.pb.h"
#include "sl_boundary.pb.h"
#include "planning_internal_info.pb.h"

#include "datapool/include/data_pool.h"
#include "datapool/include/decision_typedef.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/macro.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"
#include "src/execution/motionplan/common/path_decision.h" 
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/path/path_boundary.h"
#include "src/execution/motionplan/common/trajectory/discretized_trajectory.h"

namespace acu {
namespace planning {

/**
 * @class ReferenceLineInfo
 * @brief ReferenceLineInfo holds all data for one reference line.
 */

class ReferenceLineInfo {
 public:
  ReferenceLineInfo() = default;
  explicit ReferenceLineInfo(const VehicleState& vehicle_state,
                             const common::TrajectoryPoint& adc_planning_point,
                             const common::TrajectoryPoint& pathplan_init_point,
                             const ReferenceLine& reference_line,
                             const std::string id,
                             const LanePosition position,
                             const std::pair<eActionEnum,eDirectionEnum> action,
                             const double start_time,
                             const bool reverse_flag);

  bool Init(const std::vector<const Obstacle*>& obstacles);

  bool IsInited() const;

  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  bool AddWholeObstacles(const std::vector<const Obstacle*>& obstacles);
  Obstacle* AddObstacle(const Obstacle* obstacle);

  PathDecision* path_decision();
  const PathDecision& path_decision() const;
  const ReferenceLine& reference_line() const;
  ReferenceLine* reference_line_ptr();

  common::SLPoint init_sl_point() const {
    return start_sl_point_;
  }
  

  double SDistanceToDestination() const;
  bool ReachedDestination() const;

  double DistanceToStopPoint() const;

  void SetTrajectory(const DiscretizedTrajectory& trajectory);

  const DiscretizedTrajectory& trajectory() const;
  double TrajectoryLength() const;

  double Cost() const { return cost_; }
  std::string id() const { return id_;}
  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }
  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  void SetStopPoint(const StopPoint& stop_point);
  void SetCruiseSpeed(double speed);
  const PlanningTarget& planning_target() const { return planning_target_; }

  /**
   * @brief check if current reference line is started from another reference
   *line info line. The method is to check if the start point of current
   *reference line is on previous reference line info.
   * @return returns true if current reference line starts on previous reference
   *line, otherwise false.
   **/
  bool IsStartFrom(const ReferenceLineInfo& previous_reference_line_info) const;

  planning_internal::Debug* mutable_debug() { return &debug_; }
  const planning_internal::Debug& debug() const { return debug_; }
  LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const LatencyStats& latency_stats() const { return latency_stats_; }

  const PathInfo& path_data() const;
  const SpeedInfo& speed_data() const;
  PathInfo* mutable_path_data();
  SpeedInfo* mutable_speed_data();
  const RSSInfo& rss_info() const;
  RSSInfo* mutable_rss_info();
  // aggregate final result together by some configuration
  bool CombinePathAndSpeedProfile(
      const double relative_time, const double start_s,
      DiscretizedTrajectory* discretized_trajectory);

  const SLBoundary& AdcSlBoundary() const;
  const SLBoundary& VehicleSlBoundary() const;

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool IsChangeLanePath() const; //用于判断当前参考线是否是一条改变车道的参考线，即车辆当前位置不在此参考线上

  LanePosition Position() const; //用于判断当前参考线是在左边还是右边

  std::pair<eActionEnum,eDirectionEnum> LaterAction() const; //表明该参考线用于什么行为的规划. 

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable);
  bool IsDrivable() const;

  bool IsSafeToChangeLane() const { return is_safe_to_change_lane_; }

  // const std::list<hdmap::Id> TargetLaneId() const;
  const std::vector<std::string>& TargetLaneId() const;

  double OffsetToOtherReferenceLine() const {
    return offset_to_other_reference_line_;
  }
  void SetOffsetToOtherReferenceLine(const double offset) {
    offset_to_other_reference_line_ = offset;
  }

  void set_is_on_reference_line() { is_on_reference_line_ = true; }

  void SetDecisionPathBound(const PathBound& bound) {
    decision_bound_ = bound;
  }
  
  void SetBlockDistance(const double distance) {
    nearest_static_obstacle_s_ = distance;
  }
  double SDistanceToNearestStaticObstacle() const {
    return nearest_static_obstacle_s_;
  }

  const PathBound& DecisionPathBound() const {
    return decision_bound_;
  }

  void InitFirstOverlaps();

  uint32_t GetPriority() const { return reference_line_.GetPriority(); }

  void SetPriority(uint32_t priority) { reference_line_.SetPriority(priority); }

  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  ADCTrajectory::TrajectoryType trajectory_type() const {
    return trajectory_type_;
  }

  void SetSpeedFallbackReason(const std::string& reason) {
    speed_fallback_reason_ = reason;
  }

  std::string speed_fallback_reason() const {
    return speed_fallback_reason_;
  }

  void SetPathFallbackReason(const std::string& reason) {
    path_fallback_reason_ = reason;
  }

  std::string path_fallback_reason() const {
    return path_fallback_reason_;
  }

  const std::vector<PathBoundary>& GetCandidatePathBoundaries() const;

  void SetCandidatePathBoundaries(
      std::vector<PathBoundary> candidate_path_boundaries);

  const std::vector<std::pair<double, double>>& GetModifiedPathBoundaries() const{
    return modified_boundary_;
  }

  void SetModifiedPathBoundary(
      std::vector<std::pair<double, double>>& modified_boundary){
    modified_boundary_ = modified_boundary;
  }

  const std::vector<PathInfo>& GetCandidatePathData() const;

  void SetCandidatePathData(std::vector<PathInfo> candidate_path_data);

  // different types of overlaps that can be handleded by different scenarios.
  enum OverlapType {
    CROSSWALK = 1,
    STOP_SIGN = 2,
    SIGNAL = 3,
    CLEAR_AREA = 4,
    PNC_JUNCTION = 5,
    OBSTACLE = 6,
  };

  const std::vector<std::pair<OverlapType, hdmap::PathOverlap>>&
  FirstEncounteredOverlaps() const {
    return first_encounter_overlaps_;
  }

  void SetTargetLaneId(const std::vector<std::string>& lane_ids);

  bool IsReverse() {
    return path_data_.IsReverse();
  }

  const PathInfo GetPathInfo() {
    return path_data_;
  }

  const common::TrajectoryPoint& pathplan_init_point() const {
    return pathplan_init_point_;
  }

  const common::TrajectoryPoint& speed_init_point() const {
    return adc_planning_point_;
  }

  const common::SLPoint& speedplan_init_sl() const {
    return speed_plan_start_sl_point_;
  }

  const LaneBorrowInfo GetLaneBorrowInfo() const {
    return lane_borrow_info_;
  }  
  void SetLaneBorrowInfo(const LaneBorrowInfo& _lane_borrow_info) {
    lane_borrow_info_ = _lane_borrow_info;
  }  
  const common::SLPoint& car_sl() const {
    return car_sl_;
  }
  void AddPassByObstacleSpeedlimit();
  
  bool HasUncertainObstacles() const;

  int AssaignCartesianValues(planning_debug_msgs::DebugSLBoundary& input) const;

 private:
  // 
  void CheckCurrentPathAccess(const vector<const Obstacle*> obstacles_vec);
  
  bool CheckChangeLane() const;

  bool IsUnrelaventObstacle(const Obstacle* obstacle);

  bool AddObstacleHelper(const std::shared_ptr<Obstacle>& obstacle);

  bool GetFirstOverlap(const std::vector<hdmap::PathOverlap>& path_overlaps,
                       hdmap::PathOverlap* path_overlap);

  void SDistanceToFirstStaticObstacle();

  bool ModifyObstacleSLBoundary(Obstacle* const obstacle);

  const VehicleState vehicle_state_;
  const common::TrajectoryPoint adc_planning_point_;//speedplan start point 
  const common::TrajectoryPoint pathplan_init_point_;//pathplan start point
  common::SLPoint start_sl_point_; 
  common::SLPoint speed_plan_start_sl_point_; 

  double start_time_ = 0.0; //speedplan start point relative time

  ReferenceLine reference_line_;

  /**
   * @brief this is the number that measures the goodness of this reference
   * line. The lower the better.
   */
  double cost_ = 0.0;

  std::string id_;

  LanePosition position_ = LanePosition::CURRENT;//0: current lane ,1:left lane , -1:right lane

  const std::pair<eActionEnum,eDirectionEnum> lateral_action_;

  LaneBorrowInfo lane_borrow_info_;

  bool is_inited_ = false; //

  bool is_drivable_ = true;

  std::string speed_fallback_reason_ = "";
  std::string path_fallback_reason_ = "";

  PathDecision path_decision_;

  PathBound decision_bound_;

  std::vector<std::pair<double, double>> modified_boundary_;

  std::vector<PathBoundary> candidate_path_boundaries_;
  std::vector<PathInfo> candidate_path_data_;

  PathInfo path_data_; 
  SpeedInfo speed_data_; 

  DiscretizedTrajectory discretized_trajectory_; 

  RSSInfo rss_info_;

  struct {
    /**
     * @brief SL boundary of stitching point (starting point of plan trajectory)
     * relative to the reference line
     */
    SLBoundary adc_sl_boundary_;
    /**
     * @brief SL boundary of vehicle realtime state relative to the reference
     * line
     */
    SLBoundary vehicle_sl_boundary_;
  } sl_boundary_info_;

  planning_internal::Debug debug_;
  LatencyStats latency_stats_;

  bool is_on_reference_line_ = false;

  bool is_safe_to_change_lane_ = false;

  ADCTrajectory::RightOfWayStatus status_ = ADCTrajectory::UNPROTECTED;

  double offset_to_other_reference_line_ = 0.0;

  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

  ADCTrajectory::TrajectoryType trajectory_type_ = ADCTrajectory::UNKNOWN;

  double nearest_static_obstacle_s_ = std::numeric_limits<double>::max();

  std::vector<std::string> target_lane_ids_;

  /**
   * Overlaps encountered in the first time along the reference line in front of
   * the vehicle
   */
  std::vector<std::pair<OverlapType, hdmap::PathOverlap>>
      first_encounter_overlaps_;


  common::SLPoint car_sl_;    

  DISALLOW_COPY_AND_ASSIGN(ReferenceLineInfo);
};

}  // namespace planning
}  // namespace acu
