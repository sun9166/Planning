/**
 * @file reference_line.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "pnc_point.pb.h"
#include "sl_boundary.pb.h"
#include "common/math/vec2d.h"
#include "src/execution/motionplan/common/path/path.h"
#include "src/execution/motionplan/common/reference_line_info/reference_point.h"
#include "map/map_loader/include/map_loader.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"

namespace acu {
namespace planning {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  explicit ReferenceLine(const ReferenceLine& reference_line) = default;
  template <typename Iterator>
  explicit ReferenceLine(const Iterator begin, const Iterator end)
      : reference_points_(begin, end),
        map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}
  explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
  explicit ReferenceLine(const hdmap::Path& hdmap_path,
         const bool is_change_lane,const vector<pair<double, double> >& speed_limits);

  /** Stitch current reference line with the other reference line
   * The stitching strategy is to use current reference points as much as
   * possible. The following two examples show two successful stitch cases.
   *
   * Example 1
   * this:   |--------A-----x-----B------|
   * other:                 |-----C------x--------D-------|
   * Result: |------A-----x-----B------x--------D-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part B and part C matches, we update current reference
   * line to A-B-D.
   *
   * Example 2
   * this:                  |-----A------x--------B-------|
   * other:  |--------C-----x-----D------|
   * Result: |--------C-----x-----A------x--------B-------|
   * In the above example, A-B is current reference line, and C-D is the other
   * reference line. If part A and part D matches, we update current reference
   * line to C-A-B.
   *
   * @return false if these two reference line cannot be stitched
   */
  bool Stitch(const ReferenceLine& other);

  bool Segment(const common::math::Vec2d& point, double look_backward,
              double look_forward);
  bool Segment(const double s, double look_backward, double look_forward);

  bool Segment(const common::math::Vec2d& point,
                           const planning::ReferenceLineFrame* raw_refrence_line,
                           double look_backward, double look_forward);
  bool Segment(const double s, const planning::ReferenceLineFrame* raw_refrence_line,
               double look_backward, double look_forward);

  const hdmap::Path& map_path() const;
  const std::vector<ReferencePoint>& reference_points() const;
  std::pair<std::array<double, 3>, std::array<double, 3>> ToFrenetFrame(
      const common::TrajectoryPoint& traj_point) const;
  ReferencePoint GetReferencePoint(const double s) const;

  common::FrenetFramePoint GetFrenetPoint(
      const common::PathPoint& path_point) const;

  std::vector<ReferencePoint> GetReferencePoints(double start_s,
                                                 double end_s) const;

  size_t GetNearestReferenceIndex(const double s) const;

  ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;

  ReferencePoint GetNearestReferencePoint(const double s) const;

  ReferencePoint GetReferencePoint(const double x, const double y) const;

  bool GetApproximateSLBoundary(const common::math::Box2d& box,
                                const double start_s, const double end_s,
                                SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const common::math::Box2d& box,
                     SLBoundary* const sl_boundary) const;
  bool GetSLBoundary(const common::math::Polygon2d& polygon,
                     SLBoundary* const sl_boundary) const;

  bool SLToXY(const common::SLPoint& sl_point,
              common::math::Vec2d* const xy_point) const;
  bool XYToSL(const common::math::Vec2d& xy_point,
              common::SLPoint* const sl_point) const;
  template <class XYPoint>
  bool XYToSL(const XYPoint& xy, common::SLPoint* const sl_point) const {
    return XYToSL(common::math::Vec2d(xy.x(), xy.y()), sl_point);
  }

  bool GetLaneWidth(const double s, double* const lane_left_width,
                    double* const lane_right_width) const;
  bool GetOffsetToMap(const double s, double* l_offset) const;

  bool GetRoadWidth(const double s, double* const road_left_width,
                    double* const road_right_width) const;
  bool GetRoadWidth(const Site &point, double* const road_left_width,
                                 double* const road_right_width) const;

  double GetDrivingWidth(const SLBoundary& sl_boundary) const;

  /**
   * @brief: check if a box/point is on lane along reference line
   */
  bool IsOnLane(const common::SLPoint& sl_point) const;
  bool IsOnLane(const common::math::Vec2d& vec2d_point) const;
  template <class XYPoint>
  bool IsOnLane(const XYPoint& xy) const {
    return IsOnLane(common::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnLane(const SLBoundary& sl_boundary) const;
  bool IsChangeLanePath() const {
     return is_change_path_;
  }

  /**
   * @brief: check if a box/point is on road
   *         (not on sideways/medians) along reference line
   */
  bool IsOnRoad(const common::SLPoint& sl_point) const;
  bool IsOnRoad(const common::math::Vec2d& vec2d_point) const;
  bool IsOnRoad(const SLBoundary& sl_boundary) const;

  /**
   * @brief Check if a box is blocking the road surface. The crieria is to check
   * whether the remaining space on the road surface is larger than the provided
   * gap space.
   * @param boxed the provided box
   * @param gap check the gap of the space
   * @return true if the box blocks the road.
   */
  bool IsBlockRoad(const common::math::Box2d& box2d, double gap) const;

  /**
   * @brief check if any part of the box has overlap with the road.
   */
  bool HasOverlap(const common::math::Box2d& box) const;

  double Length() const { return map_path_.length(); }

  std::string DebugString() const;

  double GetSpeedLimitFromS(const double s) const;

  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  uint32_t GetPriority() const { return priority_; }

  void SetPriority(uint32_t priority) { priority_ = priority; }

  const hdmap::Path& GetMapPath() const { return map_path_; }

  const double cruise_speed() const {
    return cruise_speed_;
  }

  void set_cruise_speed(double speed) {
    cruise_speed_ = speed;
  }
  void UpdateSpeedLimit(const vector<pair<double, double> >& speed_limits);
  void CurvatureSpeedLimit();
  

 private:
  /**
   * @brief Linearly interpolate p0 and p1 by s0 and s1.
   * The input has to satisfy condition: s0 <= s <= s1
   * p0 and p1 must have lane_waypoint.
   * Note: it requires p0 and p1 are on the same lane, adjacent lanes, or
   * parallel neighboring lanes. Otherwise the interpolated result may not
   * valid.
   * @param p0 the first anchor point for interpolation.
   * @param s0 the longitutial distance (s) of p0 on current reference line.
   * s0 <= s && s0 <= s1
   * @param p1 the second anchor point for interpolation
   * @param s1 the longitutial distance (s) of p1 on current reference line.
   * s1
   * @param s identifies the the middle point that is going to be
   * interpolated.
   * s >= s0 && s <= s1
   * @return The interpolated ReferencePoint.
   */
  static ReferencePoint Interpolate(const ReferencePoint& p0, const double s0,
                                    const ReferencePoint& p1, const double s1,
                                    const double s);
  ReferencePoint InterpolateWithMatchedIndex(
      const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
      const double s1, const hdmap::InterpolatedIndex& index) const;

  static double FindMinDistancePoint(const ReferencePoint& p0, const double s0,
                                     const ReferencePoint& p1, const double s1,
                                     const double x, const double y);

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  /**
   * This speed limit overrides the lane speed limit
   **/
  std::vector<SpeedLimit> speed_limit_;
  std::vector<ReferencePoint> reference_points_;
  hdmap::Path map_path_;
  bool is_change_path_;
  uint32_t priority_ = 0;
  double cruise_speed_ = 8.333;
  const acu::vectormap::VectorMap* hdmap_ = acu::map::MapLoader::GetVectorMapPtr();
};

}  // namespace planning
}  // namespace acu
