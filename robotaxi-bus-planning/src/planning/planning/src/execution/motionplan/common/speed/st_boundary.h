/**
 *   @file st_boundary.h
 **/

#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "motionplanning.pb.h"

#include "st_point.h"
#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/polygon2d.h"
#include "src/execution/motionplan/common/planning_gflags.h"
namespace acu {
namespace planning {

class StBoundary : public common::math::Polygon2d {
 public:
  StBoundary() = default;

  explicit StBoundary(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs);

  explicit StBoundary(const common::math::Box2d& box) = delete;
  explicit StBoundary(std::vector<common::math::Vec2d> points) = delete;

  ~StBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }
  bool IsPointInBoundary(const STPoint& st_point) const;

  STPoint BottomLeftPoint() const;
  STPoint BottomRightPoint() const;

  StBoundary ExpandByS(const double s) const;
  StBoundary ExpandByT(const double t) const;
  StBoundary ModifyByST(const double t,const double s) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(BoundaryType type);

  BoundaryType boundary_type() const;
  const std::string& id() const;
  const bool IsVirtualObstacleBoundary() const;
  const double& speed() const;
  const double& STspeed() const;
  double characteristic_length() const;

  void SetObstacleId(const std::string& id, const bool is_virtual);
  void SetSpeed(const double& speed);
  void SetBoundaryType(const BoundaryType& boundary_type);
  void SetCharacteristicLength(const double characteristic_length);

  bool GetUnblockSRange(const double curr_time, double* s_upper,
                        double* s_lower) const;

  bool GetBoundarySRange(const double curr_time, double* s_upper,
                         double* s_lower) const;

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  double Area() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  static StBoundary GenerateStBoundary(
      const std::vector<STPoint>& lower_points,
      const std::vector<STPoint>& upper_points);

  StBoundary CutOffByT(const double t) const;

  StBoundary RightBoundaryExpandByT(const double t) const;
  StBoundary LeftBoundaryExpandByT(const double t) const;

 private:
  bool IsValid(
      const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const;

  bool IsPointNear(const common::math::LineSegment2d& seg,
                   const common::math::Vec2d& point, const double max_dist);

  void RemoveRedundantPoints(
      std::vector<std::pair<STPoint, STPoint>>* point_pairs);

  bool GetIndexRange(const std::vector<STPoint>& points, const double t,
                     size_t* left, size_t* right) const;

 private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;

  double area_ = 0.0;

  std::string id_;
  bool is_virtual_obstacle_ = false;
  double speed_;
  double st_speed_ = 0.0;//st图里求得的速度
  double characteristic_length_ = 1.0;
  double s_high_limit_ = 200.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();
};

}  // namespace planning
}  // namespace acu
