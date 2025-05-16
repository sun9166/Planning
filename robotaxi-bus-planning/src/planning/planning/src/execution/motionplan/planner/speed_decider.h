/**
 * @file speed_decider.h
 **/

#pragma once
#include <string>

#include "dp_st_speed_config.pb.h"
#include "st_boundary_config.pb.h"

#include "src/execution/motionplan/planner/procedure.h"
namespace acu {
namespace planning {

class SpeedDecider : public Procedure {
 public:
  SpeedDecider();
  ~SpeedDecider() = default;

  bool Init(const PlanningConfig& config) override;

  acu::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 private:
  enum StPosition {
    ABOVE = 1,
    BELOW = 2,
    CROSS = 3,
  };

  StPosition GetStPosition(const PathDecision* const path_decision,
                           const SpeedInfo& speed_profile,
                           const StBoundary& st_boundary) const;

  bool CheckKeepClearCrossable(
      const PathDecision* const path_decision,
      const SpeedInfo& speed_profile,
      const StBoundary& keep_clear_st_boundary) const;

  bool CheckKeepClearBlocked(
      const PathDecision* const path_decision,
      const Obstacle& keep_clear_obstacle) const;

  /**
   * @brief check if the ADC should follow an obstacle by examing the
   *StBoundary of the obstacle.
   * @param boundary The boundary of the obstacle.
   * @return true if the ADC believe it should follow the obstacle, and
   *         false otherwise.
   **/
  bool CheckIsFollowByT(const StBoundary& boundary) const;

  bool CreateStopDecision(const Obstacle& obstacle,
                          ObjectDecisionType* const stop_decision,
                          double stop_distance) const;
  bool CreateStopDecision(const Obstacle& obstacle,
                          ObjectDecisionType* const stop_decision,
                          double stop_distance, double stop_distance_buffer_estimate) const;

  /**
   * @brief create follow decision based on the boundary
   **/
  bool CreateFollowDecision(const Obstacle& obstacle,
                            ObjectDecisionType* const follow_decision) const;

  /**
   * @brief create yield decision based on the boundary
   **/
  bool CreateYieldDecision(const Obstacle& obstacle,
                           ObjectDecisionType* const yield_decision) const;

  /**
   * @brief create overtake decision based on the boundary
   **/
  bool CreateOvertakeDecision(
      const Obstacle& obstacle,
      ObjectDecisionType* const overtake_decision) const;

  common::Status MakeObjectDecision(const SpeedInfo& speed_profile,
                                    PathDecision* const path_decision) const;
  common::Status MakeObjectDecision(PathDecision* const path_decision) const;

  common::Status MakeObjectDecisionBasedOnStMap(PathDecision* const path_decision) const;

  void AppendIgnoreDecision(Obstacle* obstacle) const;

  /**
   * @brief "too close" is determined by whether ego vehicle will hit the front
   * obstacle if the obstacle drive at current speed and ego vehicle use some
   * reasonable deceleration
   **/
  bool IsFollowTooClose(const Obstacle& obstacle) const;
  bool IsYieldObstacleIgnore(const Obstacle& obstacle) const;

 private:
  DpStSpeedConfig dp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
  SLBoundary adc_sl_boundary_;
  common::TrajectoryPoint init_point_;
  const ReferenceLine* reference_line_ = nullptr;
};

}  // namespace planning
}  // namespace acu
