/**
 * @file frame.h
 **/

#pragma once

#include <list>
#include <map>
#include <string>
#include <vector>
#include "motionplanning.pb.h"
#include "planning_internal_info.pb.h"
#include "common/math/vec2d.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/macro.h"
#include "src/execution/motionplan/common/indexed_queue.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/trajectory/trajectory_info.h"
#include "src/execution/motionplan/common/trajectory/publishable_trajectory.h"
#include "map/vectormap/src/hdmap/hdmap.h"
#include "map/vectormap/src/hdmap/hdmap_util.h"

namespace acu {
namespace planning {

/**
 * @class Frame
 *
 * @brief Frame holds all data for one planning cycle.
 */

class Frame {
 public:
  explicit Frame(uint32_t sequence_num);
  explicit Frame(uint32_t sequence_num, 
                 const std::vector<common::TrajectoryPoint> &stitching_trajectory,
                 const std::vector<common::TrajectoryPoint> &stitching_trajectory_speedplan,
                 int start_point_type,
                 const double start_time,
                 const VehicleState &vehicle_state);

  const common::TrajectoryPoint &PlanningStartPoint() const;
  size_t PathStitchingStartIndex() const;
  double PlanningStartPointTime() const;

  const int StartPointType() const {
    return planning_start_point_type_;
  }

  bool Init(const vector<ReferenceLineFrame>& ref_lines);
  bool Init(const std::list<ReferenceLine> &reference_lines,
                  const vector<ReferenceLineFrame>& raw_ref_lines);

  uint32_t SequenceNum() const;

  const PublishableTrajectory &ComputedTrajectory() const {
    return current_publishable_trajectory_;
  }

  void set_current_publishable_trajectory(const PublishableTrajectory& trajectory ) {
    current_publishable_trajectory_ = trajectory;
  }

  const ReferenceLineInfo& reference_line_info() const{
    return *(reference_line_info_ptr_);
  }

  std::shared_ptr<ReferenceLineInfo> mutable_reference_line_info(){
    return reference_line_info_ptr_;
  }

  const std::list<TrajectoryInfo> &trajectory_info() const;
  std::list<TrajectoryInfo> *mutable_trajectory_info();

  Obstacle *Find(const std::string &id);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();

  const ReferenceLineInfo *DriveReferenceLineInfo() const;

  const std::vector<const Obstacle *> obstacles() const;

  const Obstacle *CreateStopObstacle(
      ReferenceLineInfo *const reference_line_info,
      const std::string &obstacle_id, const double obstacle_s);

  const Obstacle *CreateStopObstacle(const std::string &obstacle_id,
                                     const std::string &lane_id,
                                     const double lane_s);

  const Obstacle *CreateStaticObstacle(
      const std::string &obstacle_id, const double obstacle_start_s,
      const double obstacle_end_s);
  /**
   * @brief create a static virtual obstacle @pqg add 
   */
  const Obstacle *CreateStaticVirtualObstacle(const std::string &id,
                                              const common::math::Box2d &box);    

  const VehicleState &vehicle_state() const;

  ADCTrajectory *mutable_trajectory() { return &trajectory_; }

  const ADCTrajectory &trajectory() const { return trajectory_; }

  std::vector<common::TrajectoryPoint> *mutable_last_stitching_trajectory() {
    return &stitching_trajectory_;
  }

  const std::vector<common::TrajectoryPoint> &last_stitching_trajectory() {
    return stitching_trajectory_;
  }

  const bool is_near_destination() const { return is_near_destination_; }


  ThreadSafeIndexedObstacles *GetObstacleList() { return &obstacles_; }

  const std::vector<common::TrajectoryPoint>& StitchingTrajectoryForPath() const {
    return stitching_trajectory_for_path_plan_;
  }

 private:

  bool CreateReferenceLineInfo(const vector<ReferenceLineFrame>& ref_lines);
  bool CreateReferenceLineInfo(const std::list<ReferenceLine> &reference_lines,
                               const vector<ReferenceLineFrame>& ref_lines);

  bool CheckCollision() const ;
  bool CheckCollisionOnStitchingTrajectory() const;
  /**
   * Find an obstacle that collides with ADC (Autonomous Driving Car) if
   * such obstacle exists.
   * @return pointer to the obstacle if such obstacle exists, otherwise
   * @return false if no colliding obstacle.
   */
  const Obstacle *FindCollisionObstacle() const;
  const Obstacle *FindCollisionObstacleOnStitchingTrajectory() const;

  void AddObstacle(const Obstacle &obstacle);

  void AddDestinationStopWall();
  void AddVirtualStopWall();

  bool GetNearestPoint(const std::vector<common::TrajectoryPoint>& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral) const;

  bool GetFocusObstacleList(vector<LineObject>& obstacle_list, const std::map<int, LineObject>& obstacle_input,
                            const double cruise_speed) const;
  bool FindStopDecisionHistory(ObjectDecisionType& stop_desicion_last, 
                     const std::string& id) const;
  const SpeedInfo GetHistorySpeed() const;
  double FindNudgeBufferHistory(const std::string& id) const;
  Obstacle::OverlapPoints GetObstacleOverlapPointsOnSTMap(const string& obstacle_id, 
                          const vector<vector<STMapPointStruct>>& st) const;
  bool BuildKeepClearObstacle( const std::string& virtual_obstacle_id, const double keep_clear_start_s,
                            const double keep_clear_end_s);
 private:
  uint32_t sequence_num_ = 0; //记录当前帧序号
  size_t path_stitching_start_index_ = 0;
  const hdmap::HDMap *hdmap_ = nullptr;
  common::TrajectoryPoint planning_start_point_;//speedplan start point
  double start_time_ = 0.0;
  double planning_start_point_time_ = 0.0;
  VehicleState vehicle_state_;
  std::list<ReferenceLineInfo> reference_line_info_vec_;
  std::shared_ptr<ReferenceLineInfo> reference_line_info_ptr_;
  std::list<TrajectoryInfo> trajectory_info_;

  bool is_near_destination_ = false;

  /**
   * the reference line info that the vehicle finally choose to drive on
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  ThreadSafeIndexedObstacles obstacles_;
  ADCTrajectory trajectory_;  // last published trajectory
  std::vector<common::TrajectoryPoint> stitching_trajectory_;//speedplan stitching_trajectory
  std::vector<common::TrajectoryPoint> stitching_trajectory_for_path_plan_;//pathplan stitching_trajectory
  PublishableTrajectory current_publishable_trajectory_;
  bool init_data_ = false;
  int planning_start_point_type_ = 0;
  double dis_2_virtual_wall_ = 9999;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {

  private:
  DECLARE_SINGLETON(FrameHistory)
};

}  // namespace planning
}  // namespace acu
