/**
 * @file discretized_trajectory.h
 **/

#pragma once

#include <vector>
#include "pnc_point.pb.h"
#include "motionplanning.pb.h"
#include "common/math/vec2d.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"

namespace acu {
namespace planning {

class DiscretizedTrajectory : public std::vector<common::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on protobuf message
   */
  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);

  explicit DiscretizedTrajectory(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  void SetTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual common::TrajectoryPoint StartPoint() const;

  virtual size_t QueryLowerBoundPoint(const double relative_time) const;
  virtual size_t QueryLowerBoundPointByS(const double s) const;

  size_t QueryNearestPointWithBuffer(const common::math::Vec2d& position,
                                     const double buffer) const;

  virtual void AppendTrajectoryPoint(
      const common::TrajectoryPoint& trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<common::TrajectoryPoint>& trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      CHECK(trajectory_points.back().relative_time() < front().relative_time())
      <<": "<<trajectory_points.back().relative_time()<<" > "<<front().relative_time();
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  void UpdateTrajectoryPoint(const common::math::Vec2d& position, const double buffer);

  const common::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();

  void SetPropertyLength(double length) {
    length_ = length;
  } 
  double property_length() const {
    return length_;
  } 

  void SetOffsetPropertyLength(double length) {
    offset_property_length_ = length;
  } 

  double offset_property_length() const {
    return offset_property_length_;
  }

  common::PathPoint Evaluate(const double path_s) const;

  std::vector<common::TrajectoryPoint>::const_iterator QueryLowerBound(
      const double path_s) const;

private:
  double length_ = -1;
  double offset_property_length_ = -1;

};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }


}  // namespace planning
}  // namespace acu
