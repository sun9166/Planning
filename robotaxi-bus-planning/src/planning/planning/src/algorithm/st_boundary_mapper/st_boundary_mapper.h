/**
 *   @file st_boundary_mapper.h
 **/

#pragma once
#include <string> 
#include <vector>

#include "decision.pb.h"
#include "st_boundary_config.pb.h"

#include "common/common_header/status.h"
#include "src/execution/motionplan/common/speed/speed_limit.h"
#include "src/execution/motionplan/common/path_decision.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"

namespace acu {
namespace planning {

class StBoundaryMapper {
 public:
  StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                   const StBoundaryConfig& config,
                   const ReferenceLine& reference_line,
                   const PathInfo& path_data, const double planning_distance,
                   const double planning_time, const bool is_change_lane);

  virtual ~StBoundaryMapper() = default;

  acu::common::Status CreateStBoundary(PathDecision* path_decision) const;

 private:
  bool CheckOverlap(const acu::common::PathPoint& path_point,
                    const acu::common::math::Box2d& obs_box,
                    const double buffer) const;
  bool CheckSLboundaryIsOnPath(const SLBoundary& ref_line_sl_boundary) const;

  /**
   * Creates valid st boundary upper_points and lower_points
   * If return true, upper_points.size() > 1 and
   * upper_points.size() = lower_points.size()
   */
  bool GetOverlapBoundaryPoints(
      const std::vector<acu::common::PathPoint>& path_points,
      const Obstacle& obstacle, std::vector<STPoint>* upper_points,
      std::vector<STPoint>* lower_points) const;

  acu::common::Status MapWithoutDecision(Obstacle* obstacle) const;//@pqg

  bool MapStopDecision(Obstacle* stop_obstacle,
                       const ObjectDecisionType& decision) const;

  acu::common::Status MapWithDecision(
      Obstacle* obstacle, const ObjectDecisionType& decision) const;

 private:
  const SLBoundary& adc_sl_boundary_;
  const StBoundaryConfig& st_boundary_config_;
  const ReferenceLine& reference_line_;
  const PathInfo& path_data_;
  CarModel vehicle_param_;
  double back_edge_to_center  ;
  double front_edge_to_center ;
  double left_edge_to_center ;
  double right_edge_to_center;
  const double planning_distance_;
  const double planning_time_;
  bool is_change_lane_ = false;
  bool is_reversing_ = false;
  double blindScope_ = 0;
};

}  // namespace planning
}  // namespace acu
