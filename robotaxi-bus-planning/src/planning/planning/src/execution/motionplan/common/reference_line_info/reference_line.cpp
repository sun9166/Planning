/**
 * @file reference_line.cpp
 **/
#include <algorithm>
#include <limits>
#include <unordered_set>
//#include "ros/ros.h"
#include "boost/math/tools/minima.hpp"  

#include "reference_line.h"
#include "common/util/util.h"
#include "common/math/vec2d.h"
#include "common/math/angle.h"
#include "common/util/string_util.h"
#include "common/base/log/include/log.h"
#include "common/math/linear_interpolation.h"
#include "src/algorithm/math_util/planning_util.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/algorithm/math_util//coordinate_conversion/cartesian_frenet_conversion.h"

namespace acu {
namespace planning {

using MapPath = hdmap::Path;
using acu::common::SLPoint;
using acu::planning::math::DistanceXY;
using acu::hdmap::InterpolatedIndex;
const double kToleranceBuffer = 0.1;
const double kCurveKappaThreshold = 0.01;
ReferenceLine::ReferenceLine(
  const std::vector<ReferencePoint>& reference_points)
  : reference_points_(reference_points),
    map_path_(std::move(std::vector<hdmap::MapPathPoint>(
                          reference_points.begin(), reference_points.end()))) {
  CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

ReferenceLine::ReferenceLine(const MapPath& hdmap_path, 
  const bool is_change_lane,const vector<pair<double, double> >& speed_limits)
  : map_path_(hdmap_path), 
    is_change_path_(is_change_lane) {
  for (const auto& point : hdmap_path.path_points()) {
    reference_points_.emplace_back(
      point, point.kappa(), point.dkappa());
  }
  CHECK_EQ(map_path_.num_points(), reference_points_.size());
  double start_s = 0.0;
  speed_limit_.clear();
  for (const auto& limit:speed_limits) {
    speed_limit_.emplace_back(start_s, limit.first, limit.second);
    start_s = limit.first;
  }
}

void ReferenceLine::UpdateSpeedLimit(const vector<pair<double, double> >& speed_limits) {
  double start_s = 0.0;
  speed_limit_.clear();
  for (const auto& limit:speed_limits) {
    speed_limit_.emplace_back(start_s, limit.first, limit.second);
    start_s = limit.first;
  }
}

void ReferenceLine::CurvatureSpeedLimit() {
  //根据实际道路曲率限速，对弯道入口及出口间统一限速为最小的曲率速度
  double curve_start_s = 0.0;
  double curve_end_s = 0.0;
  bool is_in_curve_flag = false;
  int exit_curve_count = 0;
  double avg_curve_kappa = 0.0;
  double sum_curve_kappa = 0.0;
  int curve_point_num = 0;
  double curve_speed_limit = cruise_speed_;
  const double max_centric_acc_limit = FLAGS_max_centric_acc_limit;
  int reference_points_index = 0;
  int reference_points_num = reference_points_.size();
  std::vector<double> accumulated_s = map_path_.accumulated_s();
  for (const auto& point : reference_points_) {
     //判断进弯道
    // std::vector<hdmap::LaneWaypoint> lane_waypoints = point.lane_waypoints();
    if (std::fabs(point.kappa()) >= kCurveKappaThreshold && !is_in_curve_flag) {
      curve_start_s = accumulated_s[reference_points_index];
      if (curve_start_s < 0.1) continue;// 起始点曲率存在异常，过滤掉
      is_in_curve_flag = true;
      avg_curve_kappa = std::fabs(point.kappa());
      sum_curve_kappa += std::fabs(point.kappa());
      curve_point_num = 1;
    }
    if (is_in_curve_flag && (std::fabs(point.kappa())> kCurveKappaThreshold)) {
      if (std::fabs(point.kappa() > 10*avg_curve_kappa)) continue;//曲率突然比平均曲率大很多，过滤掉
      curve_point_num ++;
      sum_curve_kappa += std::fabs(point.kappa());
      avg_curve_kappa = sum_curve_kappa/ curve_point_num;
    }
    curve_speed_limit = std::fmax(1.2,std::sqrt(max_centric_acc_limit / avg_curve_kappa + FLAGS_numerical_epsilon));
    //判断出弯道
    if (std::fabs(point.kappa()) < kCurveKappaThreshold && is_in_curve_flag) {
      exit_curve_count ++;
      if (exit_curve_count > 5 || reference_points_index == reference_points_num) {
          curve_end_s =  accumulated_s[reference_points_index];
          AWARN_IF(FLAGS_enable_debug_motion) << "**@@@ 0)avg_curve_kappa:" << avg_curve_kappa;   
            AWARN_IF(FLAGS_enable_debug_motion) << "**@@@ 1)curve_start_s:" << curve_start_s << 
              ", curve_end_s:" << curve_end_s << ", curve_speed_limit:" << curve_speed_limit; 
          if ((curve_end_s - curve_start_s ) > 10) { 
            AWARN_IF(FLAGS_enable_debug_motion) << "**@@@ 0)avg_curve_kappa:" << avg_curve_kappa;   
            AWARN_IF(FLAGS_enable_debug_motion) << "**@@@ 1)curve_start_s:" << curve_start_s << 
              ", curve_end_s:" << curve_end_s << ", curve_speed_limit:" << curve_speed_limit;  
            AddSpeedLimit(std::max(0.0, curve_start_s - 10.0), curve_end_s, curve_speed_limit);
          }
          is_in_curve_flag = false;
          exit_curve_count = 0;
          curve_point_num = 0;
          sum_curve_kappa = 0.0;
          curve_speed_limit = cruise_speed_;
          continue;
      }
    }
    reference_points_index ++;
  }
}

bool ReferenceLine::Stitch(const ReferenceLine& other) {
  if (other.reference_points().empty()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "The other reference line is empty";
    return true;
  }
  auto first_point = reference_points_.front();
  common::SLPoint first_sl;
  if (!other.XYToSL(first_point, &first_sl)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "failed to project the first point to the other reference line";
    return false;
  }
  constexpr double kStitchingError = 2e-2;
  bool first_join = first_sl.s() > 0 && first_sl.s() < other.Length() &&
                    std::fabs(first_sl.l()) < kStitchingError;
  auto last_point = reference_points_.back();
  common::SLPoint last_sl;
  if (!other.XYToSL(last_point, &last_sl)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "failed to project the last point to the other reference line";
    return false;
  }
  bool last_join = last_sl.s() > 0 && last_sl.s() < other.Length() &&
                   std::fabs(last_sl.l()) < kStitchingError;
  const auto& other_points = other.reference_points();
  if (!first_join && !last_join) {
    common::SLPoint other_first;
    if (!XYToSL(other_points.front(), &other_first)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Could not project point : ";
      return false;
    }
    bool other_on_current = other_first.s() >= 0 &&
                            other_first.s() < Length() &&
                            std::fabs(other_first.l()) < kStitchingError;
    if (other_on_current) {
      return true;
    }
    AWARN_IF(FLAGS_enable_debug_motion)<< "These reference lines are not connected";
    return false;
  }
  const auto& accumulated_s = other.map_path().accumulated_s();
  auto lower = accumulated_s.begin();
  if (first_join) {
    lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                             first_sl.s());
    size_t start_i = std::distance(accumulated_s.begin(), lower);
    reference_points_.insert(reference_points_.begin(), other_points.begin(),
                             other_points.begin() + start_i);
  }
  if (last_join) {
    auto upper = std::upper_bound(lower, accumulated_s.end(), last_sl.s());
    auto end_i = std::distance(accumulated_s.begin(), upper);
    reference_points_.insert(reference_points_.end(),
                             other_points.begin() + end_i, other_points.end());
  }
  map_path_ = MapPath(map_path_.raw_refrence_line(),std::move(std::vector<hdmap::MapPathPoint>(
                                  reference_points_.begin(), reference_points_.end())));
  return true;
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(
  const common::math::Vec2d& xy) const {
  double min_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  size_t reference_points_num = reference_points_.size();
  for (size_t i = 0; i < reference_points_num; ++i) {
    const double distance = DistanceXY(xy, reference_points_[i]);
    if (distance < min_dist) {
      min_dist = distance;
      min_index = i;
    }
  }
  return reference_points_[min_index];
}

bool ReferenceLine::Segment(const common::math::Vec2d& point,
                           double look_backward, double look_forward) {
  common::SLPoint sl;
  if (!XYToSL(point, &sl)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Failed to project point: ";
    return false;
  }
  return Segment(sl.s(), look_backward, look_forward);
}

bool ReferenceLine::Segment(const double s, double look_backward,
                           double look_forward) {
  const auto& accumulated_s = map_path_.accumulated_s();
  // inclusive
  auto start_index =
      std::distance(accumulated_s.begin(),
                    std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward));
  
  // exclusive
  auto end_index =
      std::distance(accumulated_s.begin(),
                    std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s + look_forward));    
  if (end_index - start_index < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }
  reference_points_ =
      std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                  reference_points_.begin() + end_index);
  if (reference_points_.size() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Too few reference points after shrinking.";
    return false;
  }
  map_path_ = MapPath(map_path_.raw_refrence_line(),std::move(std::vector<hdmap::MapPathPoint>(
                                  reference_points_.begin(), reference_points_.end())));
  return true;
}

bool ReferenceLine::Segment(const common::math::Vec2d& point,
                           const planning::ReferenceLineFrame* raw_refrence_line,
                           double look_backward, double look_forward) {
  common::SLPoint sl;
  if (!XYToSL(point, &sl)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Failed to project point: ";
    return false;
  }
  return Segment(sl.s(), raw_refrence_line,look_backward, look_forward);
}

bool ReferenceLine::Segment(const double s, const planning::ReferenceLineFrame* raw_refrence_line,
                           double look_backward,double look_forward) {
  const auto& accumulated_s = map_path_.accumulated_s();
  // inclusive
  auto start_index =
      std::distance(accumulated_s.begin(),
                    std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward));
  // exclusive
  auto end_index =
      std::distance(accumulated_s.begin(),
                    std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s + look_forward));      
  if (end_index - start_index < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }
  reference_points_ =
      std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                  reference_points_.begin() + end_index);
  if (reference_points_.size() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Too few reference points after shrinking.";
    return false;
  }
  map_path_ = MapPath(raw_refrence_line,std::move(std::vector<hdmap::MapPathPoint>(
                                  reference_points_.begin(), reference_points_.end())));
  return true;
}

common::FrenetFramePoint ReferenceLine::GetFrenetPoint(
  const common::PathPoint& path_point) const {
  if (reference_points_.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"reference_points_ is empty";
    return common::FrenetFramePoint();
  }

  common::SLPoint sl_point;
  XYToSL({path_point.x(), path_point.y()}, &sl_point);
  common::FrenetFramePoint frenet_frame_point;
  frenet_frame_point.set_s(sl_point.s());
  frenet_frame_point.set_l(sl_point.l());

  const double theta = path_point.theta();
  const double kappa = path_point.kappa();
  const double l = frenet_frame_point.l();

  ReferencePoint ref_point = GetReferencePoint(frenet_frame_point.s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = math::CalculateFrenetLateralDerivative(
                      theta_ref, theta, l, kappa_ref);
  const double ddl =
    math::CalculateFrenetSecondOrderLateralDerivative(
      theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point.set_dl(dl);
  frenet_frame_point.set_ddl(ddl);
  return frenet_frame_point;
}

std::pair<std::array<double, 3>, std::array<double, 3>>
ReferenceLine::ToFrenetFrame(const common::TrajectoryPoint& traj_point) const {
  CHECK(!reference_points_.empty());

  common::SLPoint sl_point;
  XYToSL(traj_point.path_point(), &sl_point);

  std::array<double, 3> s_condition;
  std::array<double, 3> l_condition;
  ReferencePoint ref_point = GetReferencePoint(sl_point.s());
  math::cartesian_to_frenet(
      sl_point.s(), ref_point.x(), ref_point.y(), ref_point.heading(),
      ref_point.kappa(), ref_point.dkappa(), traj_point.path_point().x(),
      traj_point.path_point().y(), traj_point.v(), traj_point.a(),
      traj_point.path_point().theta(), traj_point.path_point().kappa(),
      &s_condition, &l_condition);

  return std::make_pair(s_condition, l_condition);
}

ReferencePoint ReferenceLine::GetNearestReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    return reference_points_.back();
  }
  auto it_lower =
    std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return reference_points_.front();
  } else {
    auto index = std::distance(accumulated_s.begin(), it_lower);
    if (std::fabs(accumulated_s[index - 1] - s) <
        std::fabs(accumulated_s[index] - s)) {
      return reference_points_[index - 1];
    } else {
      return reference_points_[index];
    }
  }
}

size_t ReferenceLine::GetNearestReferenceIndex(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    return 0;
  }
  if (s > accumulated_s.back() + 1e-2) {
    return reference_points_.size() - 1;
  }
  auto it_lower =
    std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  return std::distance(accumulated_s.begin(), it_lower);
}

std::vector<ReferencePoint> ReferenceLine::GetReferencePoints(
  double start_s, double end_s) const {
  if (start_s < 0.0) {
    start_s = 0.0;
  }
  if (end_s > Length()) {
    end_s = Length();
  }
  std::vector<ReferencePoint> ref_points;
  auto start_index = GetNearestReferenceIndex(start_s);
  auto end_index = GetNearestReferenceIndex(end_s);
  if (start_index < end_index) {
    ref_points.assign(reference_points_.begin() + start_index,
                      reference_points_.begin() + end_index);
  }
  return ref_points;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front() - 1e-2) {
    return reference_points_.front();
  }
  if (s > accumulated_s.back() + 1e-2) {
    return reference_points_.back();
  }

  auto interpolate_index = map_path_.GetIndexFromS(s);

  size_t index = interpolate_index.id;
  size_t next_index = index + 1;
  if (next_index >= reference_points_.size()) {
    next_index = reference_points_.size() - 1;
  }

  const auto& p0 = reference_points_[index];
  const auto& p1 = reference_points_[next_index];

  const double s0 = accumulated_s[index];
  const double s1 = accumulated_s[next_index];
  return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

double ReferenceLine::FindMinDistancePoint(const ReferencePoint& p0,
    const double s0,
    const ReferencePoint& p1,
    const double s1, const double x,
    const double y) {
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = Interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };

  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8)
         .first; 
}

ReferencePoint ReferenceLine::GetReferencePoint(const double x,
    const double y) const {
  CHECK_GE(reference_points_.size(), 0);

  auto func_distance_square = [](const ReferencePoint & point, const double x,
  const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double d_min = func_distance_square(reference_points_.front(), x, y);
  size_t index_min = 0;
  size_t reference_points_num = reference_points_.size();
  for (size_t i = 1; i < reference_points_num; ++i) {
    double d_temp = func_distance_square(reference_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  size_t index_start = (index_min == 0 ? index_min : index_min - 1);
  size_t index_end =
    (index_min + 1 == reference_points_.size() ? index_min : index_min + 1);

  if (index_start == index_end) {
    return reference_points_[index_start];
  }

  double s0 = map_path_.accumulated_s()[index_start];
  double s1 = map_path_.accumulated_s()[index_end];

  double s = ReferenceLine::FindMinDistancePoint(
               reference_points_[index_start], s0, reference_points_[index_end], s1, x,
               y);

  return Interpolate(reference_points_[index_start], s0,
                     reference_points_[index_end], s1, s);
}

bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {
  CHECK_NOTNULL(xy_point);
  if (map_path_.num_points() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "The reference line has too few points.";
    return false;
  }
  const auto matched_point = GetReferencePoint(sl_point.s());
  const auto angle = common::math::Angle16::from_rad(matched_point.heading());
  xy_point->set_x(matched_point.x() - common::math::sin(angle) * sl_point.l());
  xy_point->set_y(matched_point.y() + common::math::cos(angle) * sl_point.l());
  return true;
}

bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           SLPoint* const sl_point) const {
  DCHECK_NOTNULL(sl_point);
  double s = 0.0;
  double l = 0.0;
  if (!map_path_.GetProjection(xy_point, &s, &l)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Can't get nearest point from path.";
    return false;
  }
  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(
  const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
  const double s1, const InterpolatedIndex& index) const {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }

  double s = s0 + index.offset;

  auto map_path_point = map_path_.GetSmoothPoint(index);
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  // AERROR_IF(FLAGS_enable_debug_speedplan && kappa > 0.01)<<index.id<<", kappa = "<<kappa
  //           <<", p0.kappa = "<< p0.kappa()<<", p1.kappa = "<<p1.kappa()<<", s = ("<<s<<", "<<s0<<", "<<s1<<").";
  return ReferencePoint(map_path_point, kappa, dkappa);
}

ReferencePoint ReferenceLine::Interpolate(const ReferencePoint& p0,
    const double s0,
    const ReferencePoint& p1,
    const double s1, const double s) {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }

  const double x = common::math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = common::math::lerp(p0.y(), s0, p1.y(), s1, s);
  const double heading =
    common::math::slerp(p0.heading(), s0, p1.heading(), s1, s);
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  std::vector<hdmap::LaneWaypoint> waypoints;
  if (!p0.lane_waypoints().empty() && !p1.lane_waypoints().empty()) {
    const auto& p0_waypoint = p0.lane_waypoints()[0];
    if ((s - s0) + p0_waypoint.s <= p0_waypoint.lane->total_length()) {
      const double lane_s = p0_waypoint.s + s - s0;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
    const auto& p1_waypoint = p1.lane_waypoints()[0];
    if (p1_waypoint.lane->id().id() != p0_waypoint.lane->id().id() &&
        p1_waypoint.s - (s1 - s) >= 0) {
      const double lane_s = p1_waypoint.s - (s1 - s);
      waypoints.emplace_back(p1_waypoint.lane, lane_s);
    }
    if (waypoints.empty()) {
      const double lane_s = p0_waypoint.s;
      waypoints.emplace_back(p0_waypoint.lane, lane_s);
    }
  }
  return ReferencePoint(hdmap::MapPathPoint({x, y}, heading, waypoints), kappa,
                        dkappa);
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}

const MapPath& ReferenceLine::map_path() const { return map_path_; }

bool ReferenceLine::GetLaneWidth(const double s, double* const lane_left_width,
                                 double* const lane_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }
  return map_path_.GetLaneWidth(std::max(s,0.0), lane_left_width, lane_right_width);
}

bool ReferenceLine::GetOffsetToMap(const double s, double* l_offset) const {
  if (map_path_.path_points().empty()) {
    return false;
  }

  auto ref_point = GetNearestReferencePoint(s);
  Site nearest_point;
  nearest_point.xg = ref_point.x();
  nearest_point.yg = ref_point.y();
  double ref_s = 0;
  double ref_l = 0;
  if (!acu::planning::XYToSL(map_path_.raw_refrence_line()->mapinfo,nearest_point,ref_s,ref_l)) {
    *l_offset = 0;
    return false;
  }

  *l_offset = ref_l;
  return true;
}

bool ReferenceLine::GetRoadWidth(const double s, double* const road_left_width,
                                 double* const road_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }
  return map_path_.GetRoadWidth(s, road_left_width, road_right_width);
}

bool ReferenceLine::GetRoadWidth(const Site &point, double* const road_left_width,
                                 double* const road_right_width) const {
  if (map_path_.path_points().empty()) {
    return false;
  }
  return map_path_.GetRoadWidth(point, road_left_width, road_right_width);
}

double ReferenceLine::GetDrivingWidth(const SLBoundary& sl_boundary) const {
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  GetLaneWidth(sl_boundary.start_s(), &lane_left_width, &lane_right_width);

  double driving_width = std::max(lane_left_width - sl_boundary.end_l(),
                                  lane_right_width + sl_boundary.start_l());
  driving_width = std::min(lane_left_width + lane_right_width, driving_width);
  return driving_width;
}

bool ReferenceLine::IsOnLane(const common::math::Vec2d& vec2d_point) const {
  common::SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnLane(sl_point);
}

bool ReferenceLine::IsOnLane(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  map_path_.GetLaneWidth(middle_s, &lane_left_width, &lane_right_width);
  return !(sl_boundary.start_l() > lane_left_width ||
           sl_boundary.end_l() < -lane_right_width);
}

bool ReferenceLine::IsOnLane(const SLPoint& sl_point) const {
  if (sl_point.s() < -kToleranceBuffer || sl_point.s() > map_path_.length() + kToleranceBuffer) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"sl_point.s = "<<sl_point.s()<<", length = "<<map_path_.length()<<" return false";
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  double on_lane_s = std::min(std::max(sl_point.s(), 0.0), map_path_.length());
  if (!GetLaneWidth(on_lane_s, &left_width, &right_width)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"get lane width failed return false";
    return false;
  }
  AWARN_IF(FLAGS_enable_debug_motion)<<"-right_width "<<-right_width<<" left_width "<<left_width<<" sl_point.s() "<<sl_point.s();
  return !(sl_point.l() < -right_width || sl_point.l() > left_width);
}

bool ReferenceLine::IsBlockRoad(const common::math::Box2d& box2d,
                                double gap) const {
  return map_path_.OverlapWith(box2d, gap);
}

bool ReferenceLine::IsOnRoad(const common::math::Vec2d& vec2d_point) const {
  common::SLPoint sl_point;
  if (!XYToSL(vec2d_point, &sl_point)) {
    return false;
  }
  return IsOnRoad(sl_point);
}

bool ReferenceLine::IsOnRoad(const SLBoundary& sl_boundary) const {
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  double middle_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  double road_left_width = 0.0;
  double road_right_width = 0.0;
  map_path_.GetRoadWidth(middle_s, &road_left_width, &road_right_width);
  return !(sl_boundary.start_l() > road_left_width ||
           sl_boundary.end_l() < -road_right_width);
}

bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > map_path_.length()) {
    return false;
  }
  double road_left_width = 0.0;
  double road_right_width = 0.0;

  if (!GetRoadWidth(sl_point.s(), &road_left_width, &road_right_width)) {
    return false;
  }

  return !(sl_point.l() < -road_right_width || sl_point.l() > road_left_width);
}

// return a rough approximated SLBoundary using box length. It is guaranteed to
// be larger than the accurate SL boundary.
bool ReferenceLine::GetApproximateSLBoundary(
  const common::math::Box2d& box, const double start_s, const double end_s,
  SLBoundary* const sl_boundary) const {
  double s = 0.0;
  double l = 0.0;
  double distance = 0.0;
  if (!map_path_.GetProjectionWithHueristicParams(box.center(), start_s, end_s,
      &s, &l, &distance)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Can't get projection point from path.";
    return false;
  }

  auto projected_point = map_path_.GetSmoothPoint(s);
  auto rotated_box = box;
  rotated_box.RotateFromCenter(-projected_point.heading());

  std::vector<common::math::Vec2d> corners;
  rotated_box.GetAllCorners(&corners);

  double min_s(std::numeric_limits<double>::max());
  double max_s(std::numeric_limits<double>::lowest());
  double min_l(std::numeric_limits<double>::max());
  double max_l(std::numeric_limits<double>::lowest());

  for (const auto& point : corners) {
    // x <--> s, y <--> l
    // because the box is rotated to align the reference line
    min_s = std::fmin(min_s, point.x() - rotated_box.center().x() + s);
    max_s = std::fmax(max_s, point.x() - rotated_box.center().x() + s);
    min_l = std::fmin(min_l, point.y() - rotated_box.center().y() + l);
    max_l = std::fmax(max_l, point.y() - rotated_box.center().y() + l);
  }
  sl_boundary->set_start_s(min_s);
  sl_boundary->set_end_s(max_s);
  sl_boundary->set_start_l(min_l);
  sl_boundary->set_end_l(max_l);
  return true;
}

bool ReferenceLine::GetSLBoundary(const common::math::Box2d& box,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "failed to get projection for point: "
                      << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

bool ReferenceLine::GetSLBoundary(const common::math::Polygon2d& polygon,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  for (const auto& point : polygon.points()) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "failed to get projection for point: "
                      << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

bool ReferenceLine::HasOverlap(const common::math::Box2d& box) const {
  SLBoundary sl_boundary;
  if (!GetSLBoundary(box, &sl_boundary)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "Failed to get sl boundary for box ";
    return false;
  }
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  if (sl_boundary.start_l() * sl_boundary.end_l() < 0) {
    return false;
  }

  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  if (mid_s < 0 || mid_s > Length()) {
    return false;
  }
  if (!map_path_.GetLaneWidth(mid_s, &lane_left_width, &lane_right_width)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "failed to get width at s = " << mid_s;
    return false;
  }
  if (sl_boundary.start_l() > 0) {
    return sl_boundary.start_l() < lane_left_width;
  } else {
    return sl_boundary.end_l() > -lane_right_width;
  }
}

std::string ReferenceLine::DebugString() const {
  const auto limit =
    std::min(reference_points_.size(),
             static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  
  return acu::common::util::StrCat(
           "point num:", reference_points_.size());
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  double cognition_speed_limit = cruise_speed_;
  if (s < 0 && !speed_limit_.empty()) {
    cognition_speed_limit = speed_limit_.front().speed_limit;
  }
  for (const auto& speed_limit : speed_limit_) {
    if (s > speed_limit.start_s && s <= speed_limit.end_s) {
      cognition_speed_limit = speed_limit.speed_limit;
      break;
    }
  }
  double map_speed_limit = cruise_speed_ ;
  if (FLAGS_use_map_speed_limit) {
    const auto& map_path_point = GetReferencePoint(s);
    common::PointENU temp_point;
    double nearest_s, nearest_l;
    int return_value;
    hdmap::LaneInfoConstPtr nearest_lane;
    temp_point.set_x(map_path_point.x());
    temp_point.set_y(map_path_point.y());
    return_value = hdmap_->GetNearestLane(temp_point, nearest_lane, nearest_s, nearest_l);
    if (0 == return_value) {
      map_speed_limit = nearest_lane->lane().speed_limit();
    }
  }

  double speed_limit =
      std::fmin(fabs(map_speed_limit), fabs(cognition_speed_limit));

  return speed_limit;
}

void ReferenceLine::AddSpeedLimit(double start_s, double end_s,
                                  double speed_limit) {
  AINFO_IF(FLAGS_enable_debug_motion)<<"******ADD speedlimit : start_s =  "<< start_s << ", end_s =" << end_s << ", speed_limit" << speed_limit;
  std::vector<SpeedLimit> new_speed_limit;
  for (const auto& limit : speed_limit_) {
    if (start_s >= limit.end_s || end_s <= limit.start_s) {
      new_speed_limit.emplace_back(limit);
    } else {
      // start_s < speed_limit.end_s && end_s > speed_limit.start_s
      double min_speed = std::min(limit.speed_limit, speed_limit);
      if (start_s >= limit.start_s) {
        new_speed_limit.emplace_back(limit.start_s, start_s, limit.speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(start_s, limit.end_s, min_speed);
          new_speed_limit.emplace_back(limit.end_s,end_s, speed_limit);//@pqg
        }
      } else {
        new_speed_limit.emplace_back(start_s, limit.start_s, speed_limit);
        if (end_s <= limit.end_s) {
          new_speed_limit.emplace_back(limit.start_s, end_s, min_speed);
          new_speed_limit.emplace_back(end_s, limit.end_s, limit.speed_limit);
        } else {
          new_speed_limit.emplace_back(limit.start_s, limit.end_s, min_speed);
          new_speed_limit.emplace_back(limit.end_s,end_s, speed_limit);//@pqg
        }
      }
      start_s = limit.end_s;
      end_s = std::max(end_s, limit.end_s);
    }
  }
  speed_limit_.clear();
  if (end_s > start_s) {
    new_speed_limit.emplace_back(start_s, end_s, speed_limit);
  }
  for (const auto& limit : new_speed_limit) {
    if (limit.start_s < limit.end_s) {
      speed_limit_.emplace_back(limit);
    }
  }
  std::sort(speed_limit_.begin(), speed_limit_.end(),
  [](const SpeedLimit & a, const SpeedLimit & b) {
    if (a.start_s != b.start_s) {
      return a.start_s < b.start_s;
    }
    if (a.end_s != b.end_s) {
      return a.end_s < b.end_s;
    }
    return a.speed_limit < b.speed_limit;
  });
}

}  // namespace planning
}  // namespace acu
