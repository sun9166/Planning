/**
 * @file path.cpp
 **/

#include <limits>
#include <algorithm>
#include <unordered_map>

#include "path.h"
#include "common/math/vec2d.h"
#include "common/math/polygon2d.h"
#include "common/math/math_utils.h"
#include "common/util/string_util.h"
#include "common/math/line_segment2d.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/cognition/struct_cognition/struct_cognition.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "common/mapcheck/geotool/include/coordtransform.h"


DEFINE_double(default_lane_width, 3.5, "default lane width is about 10 feet");

namespace acu {
namespace hdmap {

using common::math::Box2d;
using common::math::kMathEpsilon;
using common::math::LineSegment2d;
using common::math::Sqr;
using common::math::Vec2d;
using std::placeholders::_1;
using acu::planning::CoordTransform;
using acu::planning::EgoInfo;
using acu::planning::VehicleState;

namespace {

const double kSampleDistance = 0.25;

bool FindLaneSegment(const MapPathPoint& p1, const MapPathPoint& p2,
                     LaneSegment* const lane_segment) {
  for (const auto& wp1 : p1.lane_waypoints()) {
    for (const auto& wp2 : p2.lane_waypoints()) {
      if (wp1.lane->id().id() == wp2.lane->id().id() && wp1.s < wp2.s) {
        *lane_segment = LaneSegment(wp1.lane, wp1.s, wp2.s);
        return true;
      }
    }
  }
  return false;
}

}  // namespace

void LaneSegment::Join(std::vector<LaneSegment>* segments) {
  constexpr double kSegmentDelta = 0.5;
  std::size_t k = 0;
  std::size_t i = 0;
  while (i < segments->size()) {
    std::size_t j = i;
    while (j + 1 < segments->size() &&
           segments->at(i).lane == segments->at(j + 1).lane) {
      ++j;
    }
    auto& segment_k = segments->at(k);
    segment_k.lane = segments->at(i).lane;
    segment_k.start_s = segments->at(i).start_s;
    segment_k.end_s = segments->at(j).end_s;
    if (segment_k.start_s < kSegmentDelta) {
      segment_k.start_s = 0.0;
    }
    if (segment_k.end_s + kSegmentDelta >= segment_k.lane->total_length()) {
      segment_k.end_s = segment_k.lane->total_length();
    }
    i = j + 1;
    ++k;
  }
  segments->resize(k);
  segments->shrink_to_fit();  // release memory
}

std::vector<MapPathPoint> MapPathPoint::GetPointsFromSegment(
    const LaneSegment& segment) {
  return GetPointsFromLane(segment.lane, segment.start_s, segment.end_s);
}

std::vector<MapPathPoint> MapPathPoint::GetPointsFromLane(LaneInfoConstPtr lane,
                                                          const double start_s,
                                                          const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  size_t lane_points_num = lane->points().size();
  for (size_t i = 0; i < lane_points_num; ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto& segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points.emplace_back(
            segment.start() +
                segment.unit_direction() * (start_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

void MapPathPoint::RemoveDuplicates(std::vector<MapPathPoint>* points) {
  constexpr double kDuplicatedPointsEpsilon = 1e-7;
  constexpr double limit = kDuplicatedPointsEpsilon; 
  CHECK_NOTNULL(points);
  int count = 0;
  size_t points_num = points->size();
  for (size_t i = 0; i < points_num; ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

Path::Path(const std::vector<MapPathPoint>& path_points)
    : path_points_(path_points) {
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points)
    : path_points_(std::move(path_points)) {
  Init();
}

Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
}

Path::Path(const std::vector<MapPathPoint>& path_points,
           const std::vector<LaneSegment>& lane_segments,
           const double max_approximation_error)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

Path::Path(const planning::ReferenceLineFrame* ref_path,const std::vector<MapPathPoint>& path_points) 
  : path_points_(path_points), 
    reference_line_(ref_path) {
  id_ = ref_path->reference_lane_id;
  geometry::SiteVec input_path = ref_path->mapinfo.path_points;
  size_t path_points_num = input_path.size();
  for (size_t i = 0 ; i < path_points_num; ++i){
    if (input_path[i].property > 0) {
      property_length_ = input_path[i].length; 
    }
    if (input_path[i].offset_property > 0 && offset_property_length_ < 0) {
      offset_property_length_ = input_path[i].length;
    }
  }
  CHECK_GE(path_points_.size(), 2);
  Init();
}

Path::Path(const planning::ReferenceLineFrame* ref_path) 
  : reference_line_(ref_path) {// reference_line_provider                            
  double accumulate_s = 0.0;
  std::vector<MapPathPoint> points;
  property_length_ = -1;
  offset_property_length_ = -1;
  is_parking_path_ = false;
  id_ = ref_path->reference_lane_id;

  CoordTransform *coor_transform = CoordTransform::Instance();
  EgoInfo* ego_info = EgoInfo::instance();
  is_parking_path_ = ref_path->mapinfo.IsNeedExtend();

  const VehicleState veh_state = ego_info->vehicle_state();
  geometry::Site ego;
  ego.xg = veh_state.x;
  ego.yg = veh_state.y;
  ego.globalangle = veh_state.heading * 180.0/M_PI;

  if (FLAGS_enable_curvature_calculation) {
    geometry::SiteVec input_path = ref_path->mapinfo.path_points;
    GetCurvature(input_path);
    size_t path_points_num = input_path.size();
    for (size_t i = 0 ; i < path_points_num; ++i) {
      double dkappa = 0.0 ;
      if ( i == 0) dkappa = 0.0;
      else {
        double delta_s = input_path[i].length - input_path[i-1].length;
        if (delta_s == 0) dkappa = 0;
        else dkappa = (input_path[i].curvature - input_path[i-1].curvature) / delta_s;
      }
      if (input_path[i].property > 0) {
        property_length_ = input_path[i].length;
      }
      if (input_path[i].offset_property > 1 && offset_property_length_ < 0) {
        offset_property_length_ = input_path[i].length;
      }
      common::math::Vec2d map_point(input_path[i].xg,input_path[i].yg);
      double heading = input_path[i].globalangle * 0.0174;//全局angle角,后面用的是弧度，需要需要统一做转换
      points.emplace_back(map_point,heading,input_path[i].curvature,
                          dkappa);
    }
  } else {
    geometry::SiteVec input_path = ref_path->mapinfo.path_points;
    size_t path_points_num = input_path.size();

    for (size_t i = 0 ; i < path_points_num; ++i){
      double dkappa = 0.0 ;
      if (i == 0) dkappa = 0.0;
      else {
        double delta_s = input_path[i].length - input_path[i-1].length;
        if (delta_s == 0) dkappa = 0;
        else dkappa = (input_path[i].curvature - input_path[i-1].curvature) / delta_s;
      }
      if (input_path[i].property > 0) {
        property_length_ = input_path[i].length; 
      }
      if (input_path[i].offset_property > 0 && offset_property_length_ < 0) {
        offset_property_length_ = input_path[i].length;
      }
      common::math::Vec2d map_point(input_path[i].xg,input_path[i].yg);

      double heading = input_path[i].globalangle * 0.0174;//角度转弧度,若输入为弧度制,可去掉后面的计算

      if (input_path[i].globalangle < 0 && fabs(fabs(input_path[i].globalangle) -180) < 1e-6) {
        heading = 180 * 0.0174;
      }
      points.emplace_back(map_point,heading,input_path[i].curvature,
                          dkappa);
    }
  }

  AINFO_IF(FLAGS_enable_debug_motion)<<"property_length_:"<<property_length_;

  path_points_.clear();
  path_points_ = points;
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2);
  Init();
}

Path::Path(std::vector<LaneSegment>&& segments)
    : lane_segments_(std::move(segments)) {
  for (const auto& segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2);
  Init();
}

Path::Path(std::vector<MapPathPoint>&& path_points,
           std::vector<LaneSegment>&& lane_segments,
           const double max_approximation_error)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
  if (max_approximation_error > 0.0) {
    use_path_approximation_ = true;
    approximation_ = PathApproximation(*this, max_approximation_error);
  }
}

void Path::Init() {
  InitPoints();
  InitLaneSegments();
  InitPointIndex();
  InitWidth();
}

void Path::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());
  CHECK_GE(num_points_, 2);
  
  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);

  segments_.clear();
  segments_.reserve(num_points_);
  unit_directions_.clear();
  unit_directions_.reserve(num_points_);
  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
    } else {
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      heading = path_points_[i + 1] - path_points_[i];
      s += heading.Length();
    }
    heading.Normalize();
    unit_directions_.push_back(heading);
  }
  length_ = s;
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
  num_segments_ = num_points_ - 1;

  double l_on_raw_line = 0;
  Site first_point;
  first_point.xg = path_points_.front().x();
  first_point.yg = path_points_.front().y();
  XYToSL(reference_line_->mapinfo, first_point, s_offset_on_raw_line_, l_on_raw_line);
  CHECK_EQ(accumulated_s_.size(), num_points_);
  CHECK_EQ(unit_directions_.size(), num_points_);
  CHECK_EQ(segments_.size(), num_segments_);
}

void Path::InitLaneSegments() {
  if (lane_segments_.empty()) {
    for (int i = 0; i + 1 < num_points_; ++i) {
      LaneSegment lane_segment;
      if (FindLaneSegment(path_points_[i], path_points_[i + 1],
                          &lane_segment)) {
        lane_segments_.push_back(lane_segment);
      }
    }
  }
  LaneSegment::Join(&lane_segments_);
  if (lane_segments_.empty()) {
    return;
  }
  lane_accumulated_s_.resize(lane_segments_.size());
  lane_accumulated_s_[0] = lane_segments_[0].Length();
  size_t lane_segments_size = lane_segments_.size();
  for (std::size_t i = 1; i < lane_segments_size; ++i) {
    lane_accumulated_s_[i] =
        lane_accumulated_s_[i - 1] + lane_segments_[i].Length();
  }

  lane_segments_to_next_point_.clear();
  lane_segments_to_next_point_.reserve(num_points_);
  for (int i = 0; i + 1 < num_points_; ++i) {
    LaneSegment lane_segment;
    if (FindLaneSegment(path_points_[i], path_points_[i + 1], &lane_segment)) {
      lane_segments_to_next_point_.push_back(lane_segment);
    } else {
      lane_segments_to_next_point_.push_back(LaneSegment());
    }
  }
  CHECK_EQ(lane_segments_to_next_point_.size(), num_segments_);
}

void Path::InitWidth() {
  lane_left_width_.clear();
  lane_left_width_.reserve(num_sample_points_);
  lane_right_width_.clear();
  lane_right_width_.reserve(num_sample_points_);

  road_left_width_.clear();
  road_left_width_.reserve(num_sample_points_);
  road_right_width_.clear();
  road_right_width_.reserve(num_sample_points_);

  double s = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
      lane_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      lane_right_width_.push_back(FLAGS_default_lane_width / 2.0);

      road_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      road_right_width_.push_back(FLAGS_default_lane_width / 2.0);
  }
  CHECK_EQ(lane_left_width_.size(), num_sample_points_);
  CHECK_EQ(lane_right_width_.size(), num_sample_points_);

  CHECK_EQ(road_left_width_.size(), num_sample_points_);
  CHECK_EQ(road_right_width_.size(), num_sample_points_);
}

void Path::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);//reserve为容器预留空间
  double s = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {//num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;每个点之间间隔kSampleDistance，0.25
    while (last_index + 1 < num_points_ &&
           accumulated_s_[last_index + 1] <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }
  CHECK_EQ(last_point_index_.size(), num_sample_points_);
}

void Path::GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                          std::vector<PathOverlap>* const overlaps) const {
  if (overlaps == nullptr) {
    return;
  }
  overlaps->clear();
  std::unordered_map<std::string, std::vector<std::pair<double, double>>>
      overlaps_by_id;
  double s = 0.0;
  for (const auto& lane_segment : lane_segments_) {
    if (lane_segment.lane == nullptr) {
      continue;
    }
    for (const auto& overlap : GetOverlaps_from_lane(*(lane_segment.lane))) {
      const auto& overlap_info =
          overlap->GetObjectOverlapInfo(lane_segment.lane->id());
      if (overlap_info == nullptr) {
        continue;
      }

      const auto& lane_overlap_info = overlap_info->lane_overlap_info();
      if (lane_overlap_info.start_s() <= lane_segment.end_s &&
          lane_overlap_info.end_s() >= lane_segment.start_s) {
        const double ref_s = s - lane_segment.start_s;
        const double adjusted_start_s =
            std::max(lane_overlap_info.start_s(), lane_segment.start_s) + ref_s;
        const double adjusted_end_s =
            std::min(lane_overlap_info.end_s(), lane_segment.end_s) + ref_s;
        for (const auto& object : overlap->overlap().object()) {
          if (object.id().id() != lane_segment.lane->id().id()) {
            overlaps_by_id[object.id().id()].emplace_back(adjusted_start_s,
                                                          adjusted_end_s);
          }
        }
      }
    }
    s += lane_segment.end_s - lane_segment.start_s;
  }
  for (auto& overlaps_one_object : overlaps_by_id) {
    const std::string& object_id = overlaps_one_object.first;
    auto& segments = overlaps_one_object.second;
    std::sort(segments.begin(), segments.end());

    const double kMinOverlapDistanceGap = 1.5;  // in meters.
    for (const auto& segment : segments) {
      if (!overlaps->empty() && overlaps->back().object_id == object_id &&
          segment.first - overlaps->back().end_s <= kMinOverlapDistanceGap) {
        overlaps->back().end_s =
            std::max(overlaps->back().end_s, segment.second);
      } else {
        overlaps->emplace_back(object_id, segment.first, segment.second);
      }
    }
  }
  std::sort(overlaps->begin(), overlaps->end(),
            [](const PathOverlap& overlap1, const PathOverlap& overlap2) {
              return overlap1.start_s < overlap2.start_s;
            });
}

const PathOverlap* Path::NextLaneOverlap(double s) const {
  auto next = std::upper_bound(
      lane_overlaps_.begin(), lane_overlaps_.end(), s,
      [](double s, const PathOverlap& o) { return s < o.start_s; });
  if (next == lane_overlaps_.end()) {
    return nullptr;
  } else {
    return &(*next);
  }
}

void Path::InitOverlaps() {
  GetAllOverlaps(std::bind(&LaneInfo::cross_lanes, _1), &lane_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::signals, _1), &signal_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::yield_signs, _1), &yield_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::stop_signs, _1), &stop_sign_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::crosswalks, _1), &crosswalk_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::junctions, _1), &junction_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::clear_areas, _1), &clear_area_overlaps_);
  GetAllOverlaps(std::bind(&LaneInfo::speed_bumps, _1), &speed_bump_overlaps_);
}

MapPathPoint Path::GetSmoothPoint(const InterpolatedIndex& index) const {
  CHECK_GE(index.id, 0);
  CHECK_LT(index.id, num_points_);

  const MapPathPoint& ref_point = path_points_[index.id];
  //1.offset不等于0,则需要重新计算，否则，直接等于ref_point
  if (std::abs(index.offset) > kMathEpsilon) {
    const Vec2d delta = unit_directions_[index.id] * index.offset;
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()},
                       ref_point.heading());
    if (index.id < num_segments_ && !ref_point.lane_waypoints().empty()) {
      const LaneSegment& lane_segment = lane_segments_to_next_point_[index.id];
      auto ref_lane_waypoint = ref_point.lane_waypoints()[0];
      if (lane_segment.lane != nullptr) {
        for (const auto& lane_waypoint : ref_point.lane_waypoints()) {
          if (lane_waypoint.lane->id().id() == lane_segment.lane->id().id()) {
            ref_lane_waypoint = lane_waypoint;
            break;
          }
        }
        point.add_lane_waypoint(
            LaneWaypoint(lane_segment.lane, lane_segment.start_s + index.offset,
                         ref_lane_waypoint.l));
      }
    }
    if (point.lane_waypoints().empty() && !ref_point.lane_waypoints().empty()) {
      point.add_lane_waypoint(ref_point.lane_waypoints()[0]);
    }
    return point;
  } else {
    return ref_point;
  }
}

MapPathPoint Path::GetSmoothPoint(double s) const {
  return GetSmoothPoint(GetIndexFromS(s));
}

double Path::GetSFromIndex(const InterpolatedIndex& index) const {
  if (index.id < 0) {
    return 0.0;
  }
  if (index.id >= num_points_) {
    return length_;
  }
  return accumulated_s_[index.id] + index.offset;
}

InterpolatedIndex Path::GetIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(num_points_, 0);
  if (s >= length_) {
    return {num_points_ - 1, 0.0};
  }
  const int sample_id = static_cast<int>(s / kSampleDistance);//kSampleDistance:0.25
  if (sample_id >= num_sample_points_) {//num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
    return {num_points_ - 1, 0.0};
  }
  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return {low, s - accumulated_s_[low]};
}

InterpolatedIndex Path::GetLaneIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(lane_segments_.size(), 0);
  if (s >= length_) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }
  auto iter = std::lower_bound(lane_accumulated_s_.begin(),
                               lane_accumulated_s_.end(), s);
  if (iter == lane_accumulated_s_.end()) {
    return {static_cast<int>(lane_segments_.size() - 1),
            lane_segments_.back().Length()};
  }
  int index =
      static_cast<int>(std::distance(lane_accumulated_s_.begin(), iter));
  if (index == 0) {
    return {index, s};
  } else {
    return {index, s - lane_accumulated_s_[index - 1]};
  }
}

std::vector<hdmap::LaneSegment> Path::GetLaneSegments(
    const double start_s, const double end_s) const {
  std::vector<hdmap::LaneSegment> lanes;
  if (start_s + kMathEpsilon < end_s) {
    return lanes;
  }
  auto start_index = GetLaneIndexFromS(start_s);
  if (start_index.offset + kMathEpsilon >=
      lane_segments_[start_index.id].Length()) {
    start_index.id += 1;
    start_index.offset = 0;
  }
  const int num_lanes = static_cast<int>(lane_segments_.size());
  if (start_index.id >= num_lanes) {
    return lanes;
  }
  lanes.emplace_back(lane_segments_[start_index.id].lane, start_index.offset,
                     lane_segments_[start_index.id].Length());
  auto end_index = GetLaneIndexFromS(end_s);
  for (int i = start_index.id; i < end_index.id && i < num_lanes; ++i) {
    lanes.emplace_back(lane_segments_[i]);
  }
  if (end_index.offset >= kMathEpsilon) {
    lanes.emplace_back(lane_segments_[end_index.id].lane, 0, end_index.offset);
  }
  return lanes;
}

bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral) const {
  double distance = 0.0;
  return GetNearestPoint(point, accumulate_s, lateral, &distance);
}

bool Path::GetNearestPoint(const Vec2d& point, double* accumulate_s,
                           double* lateral, double* min_distance) const {
  if (!GetProjection(point, accumulate_s, lateral, min_distance)) {
    return false;
  }
  if (*accumulate_s < 0.0) {
    *accumulate_s = 0.0;
    *min_distance = point.DistanceTo(path_points_[0]);
  } else if (*accumulate_s > length_) {
    *accumulate_s = length_;
    *min_distance = point.DistanceTo(path_points_.back());
  }
  return true;
}

bool Path::GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                         double* lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool Path::GetProjectionWithHueristicParams(const Vec2d& point,
                                            const double hueristic_start_s,
                                            const double hueristic_end_s,
                                            double* accumulate_s,
                                            double* lateral,
                                            double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();

  int start_interpolation_index = GetIndexFromS(hueristic_start_s).id;
  int end_interpolation_index = static_cast<int>(
      std::fmin(num_segments_, GetIndexFromS(hueristic_end_s).id + 1));
  int min_index = start_interpolation_index;
  for (int i = start_interpolation_index; i < end_interpolation_index; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool Path::GetProjection(const Vec2d& point, double* accumulate_s,
                         double* lateral, double* min_distance) const {
  if (segments_.empty()) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  if (use_path_approximation_) {
    return approximation_.GetProjection(*this, point, accumulate_s, lateral,
                                        min_distance);
  }
  CHECK_GE(num_points_, 2);
  *min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments_; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto& nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else if (min_index == num_segments_ - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
  }
  return true;
}

bool Path::GetHeadingAlongPath(const Vec2d& point, double* heading) const {
  if (heading == nullptr) {
    return false;
  }
  double s = 0;
  double l = 0;
  if (GetProjection(point, &s, &l)) {
    *heading = GetSmoothPoint(s).heading();
    return true;
  }
  return false;
}

double Path::GetLaneLeftWidth(const double s) const {
  return GetSample(lane_left_width_, s);
}

double Path::GetLaneRightWidth(const double s) const {
  return GetSample(lane_right_width_, s);
}

bool Path::GetLaneWidth(const double s, double* lane_left_width,
                        double* lane_right_width) const {
  CHECK_NOTNULL(lane_left_width);
  CHECK_NOTNULL(lane_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }
  *lane_left_width = GetSample(lane_left_width_, s);
  *lane_right_width = GetSample(lane_right_width_, s);

  if (property_length_ < 0 && offset_property_length_ < 0) { //地图上的参考线 @pqg
    if (reference_line_->GetWidthToLaneBoundary(
        *lane_left_width,*lane_right_width, std::fmax(0.0, s + s_offset_on_raw_line_))) {
      if (FLAGS_enable_smooth_reference_line) {
        auto ref_point = GetSmoothPoint(s);
        Site nearest_point;
        nearest_point.xg = ref_point.x();
        nearest_point.yg = ref_point.y();
        double ref_s = 0;
        double ref_l = 0;
        if (XYToSL(reference_line_->mapinfo, nearest_point,ref_s,ref_l)) {
          if (fabs(ref_l) > 0.5) {
            AWARN_IF(FLAGS_enable_debug_motion)<<"s = "<<s<<", lane_left_width = "<<*lane_left_width
               <<", lane_right_width"<<*lane_right_width<<", ref_l = "<<ref_l;
          }
          *lane_left_width = *lane_left_width - ref_l;
          *lane_right_width = *lane_right_width + ref_l;
        } 
      }
      return true;
    } else {
      return false;
    }
  }
  return true;
}

bool Path::GetFreeSpaceboundary(const double s, double* left_width,
                        double* right_width) const {
  if (reference_line_->GetWidthToFreeSpace(
      *left_width,*right_width, std::fmax(0.0, s + s_offset_on_raw_line_))) {
    if (FLAGS_enable_smooth_reference_line) {
      auto ref_point = GetSmoothPoint(s);
      Site nearest_point;
      nearest_point.xg = ref_point.x();
      nearest_point.yg = ref_point.y();
      double ref_s = 0;
      double ref_l = 0;
      if (XYToSL(reference_line_->mapinfo,nearest_point,ref_s,ref_l)) {
        if (fabs(ref_l) > 0.5) {
          AWARN_IF(FLAGS_enable_debug_motion)<<"s = "<<s<<", left_to_fs = "<<*left_width
             <<", right_to_fs"<<*right_width<<", ref_l = "<<ref_l;
        }
        *left_width = *left_width - ref_l;
        *right_width = *right_width + ref_l;
      } 
    }
    return true;
  } else {
    return false;
  }

}

double Path::GetOffsetToMapLine(const double& s) const {
  double offset_to_map = 0.0;
  if (property_length_ < 0 && offset_property_length_ < 0) { //地图上的参考线 @pqg
    if (FLAGS_enable_smooth_reference_line) {
      auto ref_point = GetSmoothPoint(s);
      Site nearest_point;
      nearest_point.xg = ref_point.x();
      nearest_point.yg = ref_point.y();
      double ref_s = 0;
      double ref_l = 0;
      if (XYToSL(reference_line_->mapinfo,nearest_point,ref_s,ref_l)) {
        offset_to_map = ref_l;
      } 
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"offset_to_map = "<<offset_to_map;
  return offset_to_map;
}

double Path::GetRoadLeftWidth(const double s) const {
  return GetSample(road_left_width_, s);
}

double Path::GetRoadRightWidth(const double s) const {
  return GetSample(road_right_width_, s);
}

bool Path::GetRoadWidth(const double s, double* road_left_width,
                        double* road_right_width) const {
  CHECK_NOTNULL(road_left_width);
  CHECK_NOTNULL(road_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }

  if (reference_line_->GetWidthToRoadBoundary(
      *road_left_width,*road_right_width, std::fmax(0.0, s + s_offset_on_raw_line_), false)) {
    if (FLAGS_enable_smooth_reference_line) {
        auto ref_point = GetSmoothPoint(s);
        Site nearest_point;
        nearest_point.xg = ref_point.x();
        nearest_point.yg = ref_point.y();
        double ref_s = 0;
        double ref_l = 0;
        if (XYToSL(reference_line_->mapinfo,nearest_point,ref_s,ref_l)) {
          if (fabs(ref_l) > 0.5) {
            AWARN_IF(FLAGS_enable_debug_motion)<<"ref_l = "<<ref_l<<", smoothed point is more than 0.5m away from reference_point. ";
          }
          *road_left_width = *road_left_width - ref_l;
          *road_right_width = fabs(-(*road_right_width) - ref_l);
        } 
      }
      return true;
  } else {
    return false;
  }
}

bool Path::GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, double s) const {
  if (s < 0.0 || s > length_) {
    return false;
  }
  if (reference_line_->GetLaneBoundaryType(l_bd_type,r_bd_type, std::fmax(0.0, s + s_offset_on_raw_line_))) {
    return true;
  } else {
    return false;
  }
}

bool Path::GetRoadWidth(const Site &point, double* road_left_width,
                    double* road_right_width) const {
  CHECK_NOTNULL(road_left_width);
  CHECK_NOTNULL(road_right_width);

  if (point.xg < 0.0 || point.yg < 0.0) {
    return false;
  }

  if (reference_line_->GetWidthToRoadBoundary(*road_left_width,*road_right_width, point)) {
    if (FLAGS_enable_smooth_reference_line) {
        double ref_s = 0;
        double ref_l = 0;
        if (XYToSL(reference_line_->mapinfo,point,ref_s,ref_l)) {
          if (fabs(ref_l) > 0.5) {
            AWARN_IF(FLAGS_enable_debug_motion)<<"ref_l = "<<ref_l<<", smoothed point is more than 0.5m away from reference_point. ";
          }
          *road_left_width = *road_left_width - ref_l;
          *road_right_width = fabs(-(*road_right_width) - ref_l);
        } 
      }
      return true;
  } else {
    return false;
  }
}

bool Path::GetRoadBoundary(const double s, double* road_left_boundary, double* road_right_boundary) const {
  CHECK_NOTNULL(road_left_boundary);
  CHECK_NOTNULL(road_right_boundary);

  if (s < 0.0 || s > length_) {
    return false;
  }

  if (reference_line_->GetWidthToRoadBoundary(
       *road_left_boundary,*road_right_boundary, std::fmax(0.0, s + s_offset_on_raw_line_), false)) {
    if (FLAGS_enable_smooth_reference_line) {
      auto ref_point = GetSmoothPoint(s);
      Site nearest_point;
      nearest_point.xg = ref_point.x();
      nearest_point.yg = ref_point.y();
      double ref_s = 0;
      double ref_l = 0;
      if (XYToSL(reference_line_->mapinfo,nearest_point,ref_s,ref_l)) {
        if (fabs(ref_l) > 0.5) {
          AWARN_IF(FLAGS_enable_debug_motion)<<"ref_l = "<<ref_l<<", smoothed point is more than 0.5m away from reference_point. ";
        }
        *road_left_boundary = *road_left_boundary - ref_l;
        *road_right_boundary = fabs(-(*road_right_boundary) - ref_l);
      } 
    }
    return true;
  } else {
    return false;
  }
  
}


double Path::GetSample(const std::vector<double>& samples,
                       const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return samples[0];
  }
  const int idx = static_cast<int>(s / kSampleDistance);
  if (idx >= num_sample_points_ - 1) {
    return samples.back();
  }
  const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
  return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
}

bool Path::IsOnPath(const Vec2d& point) const {
  double accumulate_s = 0.0;
  double lateral = 0.0;
  if (!GetProjection(point, &accumulate_s, &lateral)) {
    return false;
  }
  double lane_left_width = 0.0;
  double lane_right_width = 0.0;
  if (!GetLaneWidth(accumulate_s, &lane_left_width, &lane_right_width)) {
    return false;
  }
  if (lateral < lane_left_width && lateral > -lane_right_width) {
    return true;
  }
  return false;
}

bool Path::OverlapWith(const common::math::Box2d& box, double width) const {
  if (use_path_approximation_) {
    return approximation_.OverlapWith(*this, box, width);
  }
  const Vec2d center = box.center();
  const double radius_sqr = Sqr(box.diagonal() / 2.0 + width) + kMathEpsilon;
  for (const auto& segment : segments_) {
    if (segment.DistanceSquareTo(center) > radius_sqr) {
      continue;
    }
    if (box.DistanceTo(segment) <= width + kMathEpsilon) {
      return true;
    }
  }
  return false;
}


void Path::GetCurvature(geometry::SiteVec& trajectory) {//使用全局坐标进行计算曲率
  CalculateLengthOfPoint(trajectory);  
  geometry::SiteVec path_points_smooth = trajectory;
  int path_point_size = trajectory.size();
  int half_length = MIN(5, int(path_point_size / 2)); //窗口大小为 half_length*2+1
  if (!trajectory.empty()) {
    double fit_length = 2.0; //取的 路点 向量的模长为 3.0m
    //1.路径坐标点窗口平滑处理
    for (int i = 0; i < path_point_size; ++i) {
      int j = 0;
      double x_sum = 0;
      double y_sum = 0;
      int smooth_points_cnt = 0;
      if (i < half_length) { //路径前几个路点处理
        for (j = 0; j <= 2 * i; ++j) {
            x_sum += trajectory[j].xg;
            y_sum += trajectory[j].yg;
        }
        smooth_points_cnt = j;
      } else if (i >= path_point_size - half_length) { //路径末尾几个路点处理
        for(j = i - (path_point_size - 1 - i); j < path_point_size; ++j){
            x_sum += trajectory[j].xg;
            y_sum += trajectory[j].yg;
        }
        smooth_points_cnt = path_point_size - (i - (path_point_size - 1 - i));
      } else {//正常平滑处理
        for (j = i - half_length; j <= i + half_length; ++j) {
            x_sum += trajectory[j].xg;
            y_sum += trajectory[j].yg;
        }
        smooth_points_cnt = 2 * half_length + 1;
      }
      smooth_points_cnt = MAX(1,smooth_points_cnt);
      path_points_smooth[i].xg = x_sum / smooth_points_cnt;
      path_points_smooth[i].yg = y_sum / smooth_points_cnt;
    }
    //2.根据截取路径的 两个向量夹角及模长，计算曲率
    double cal_length = 0;
    geometry::Site sample_points[3];
    int i, j;
    int fit_start_index = 0;
    int fit_end_index = 0;
    // 获得向量的第一个起点
    sample_points[0] = path_points_smooth[0];
    for (i = 0; i < path_point_size; ++i) {
      cal_length = GetDist(sample_points[0].xg,
                            sample_points[0].yg,
                            path_points_smooth[i].xg,
                            path_points_smooth[i].yg);
      if (cal_length >= fit_length) {
          sample_points[1] = path_points_smooth[i];
          fit_start_index = i;
          break;
      }
    }
    if (fit_start_index == 0) {
      for (i = 0; i < path_point_size; ++i) {
        trajectory[i].curvature = curve_end_;
      }
      return;
    }
    //开始正常提取向量
    for (i = fit_start_index; i < path_point_size; ++i) {
      int m, n;
      //向量中间点
      sample_points[1] = path_points_smooth[i];
      //计算左向量终点
      for (m = i; m > 0; --m) {
        cal_length = GetDist(sample_points[1].xg,
                              sample_points[1].yg,
                              path_points_smooth[m].xg,
                              path_points_smooth[m].yg);
        if (cal_length >= fit_length) {
            sample_points[0] = path_points_smooth[m];
            break;
        }
      }
      if (m == 0) sample_points[0] = path_points_smooth[0];
      //计算右向量终点
      for (m = i; m < path_point_size; ++m) {
        cal_length = GetDist(sample_points[1].xg,
                              sample_points[1].yg,
                              path_points_smooth[m].xg,
                              path_points_smooth[m].yg);
        if (cal_length >= fit_length) {
          sample_points[2] = path_points_smooth[m];
          break;
        }
      }

      if (m == path_point_size) {
        fit_end_index = i;
        curve_end_ = trajectory[fit_end_index-1].curvature;
        break;
      }

      //计算 路点 曲率
      geometry::Site vecA, vecB,vecAB;
      double AngleAB = 0;
      //根据三点坐标，计算A、B向量
      vecA.xg = sample_points[0].xg - sample_points[1].xg;
      vecA.yg = sample_points[0].yg - sample_points[1].yg;
      vecB.xg = sample_points[2].xg - sample_points[1].xg;
      vecB.yg = sample_points[2].yg - sample_points[1].yg;
      vecAB.xg = vecA.xg - vecB.xg;
      vecAB.yg = vecA.yg - vecB.yg;
      double vector_OA_OB =  vecA.xg * vecB.yg - vecA.yg * vecB.xg;  //OAXOB(叉乘)其值为以OA和OB向量构成的平行四边形的面积。
      double module_vecA = sqrt(vecA.xg * vecA.xg + vecA.yg * vecA.yg);//向量OA的模
      double module_vecB = sqrt(vecB.xg * vecB.xg + vecB.yg * vecB.yg);//向量OB的模
      double module_vecAB = sqrt(vecAB.xg * vecAB.xg + vecAB.yg * vecAB.yg);//向量AB的模
      double Curve = 2 * vector_OA_OB/(module_vecA*module_vecB*module_vecAB);//三角形外切圆半径R=abc/4S,abc为三角形三边长，S为三角形面积。所以kappa=1/R=4S/(abc)
      Curve = -Curve;//@pqg 因后面的规划层算出来的曲率为左正右负，这里不取反，则为左负右正 
      trajectory[i].curvature = Curve;
    }
    if (fit_start_index == fit_end_index) {
      trajectory[fit_start_index].curvature = curve_end_;
    }

    for (i = 0 ; i < fit_start_index; ++i) {
      trajectory[i].curvature = trajectory[fit_start_index].curvature;
    }

    if (trajectory[path_point_size - 1].length < 40 || 1) {
      for (i = fit_end_index ; i < path_point_size; ++i) {
        trajectory[i].curvature = curve_end_;
      }
    }
  }    
}

void Path::CalculateLengthOfPoint(geometry::SiteVec& trajectory) {
    if (trajectory.empty()) return ;
    int points_size = (int)trajectory.size();
    double length_accumulate = 0.0;
    for (int i = 0; i < points_size; ++i){
      if (i == 0){
        trajectory[i].length = 0.0;
        continue;
      } 
      double distance = std::hypot(trajectory[i].xg - trajectory[i-1].xg, 
                              trajectory[i].yg -trajectory[i-1].yg);
      length_accumulate = length_accumulate + distance;
      trajectory[i].length = length_accumulate ;
    }
}

double Path::GetDist(double x0, double y0, double x1, double y1) 
{
  double dist2Pts = sqrt((x0 - x1 ) * (x0 - x1) + (y0 - y1) * (y0 - y1)); 
  return dist2Pts;
}

void Path::ExtendPath( const geometry::SiteVec& ref_path_points, geometry::SiteVec& input_path,
    const geometry::Site& ego, const double& extend_length) {
  const double extend_unit_len = 0.2;
  CoordTransform *coor_transform = CoordTransform::Instance();
  if(!input_path.empty() && !ref_path_points.empty() ) {
    bool reverse = ref_path_points.front().reverse;
    const double cosheading = std::cos((ref_path_points.back().globalangle + (reverse ? 180.0:0.0)) * M_PI/180.0);
    const double sinheading = std::sin((ref_path_points.back().globalangle + (reverse ? 180.0:0.0)) * M_PI/180.0);
    while(input_path.back().length - ref_path_points.back().length < extend_length) {
      geometry::Site input_point;
      input_point.xg = input_path.back().xg + extend_unit_len * cosheading;
      input_point.yg = input_path.back().yg + extend_unit_len * sinheading;
      input_point.length = input_path.back().length + extend_unit_len;
      input_point.globalangle = input_path.back().globalangle;
      input_point.curvature = input_path.back().curvature;
      coor_transform->GCCS2VCS(ego, input_point, input_point);
      input_path.emplace_back(input_point);
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"ref_path back globalangle "<<ref_path_points.back().globalangle;
  }
}

double PathApproximation::compute_max_error(const Path& path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return 0.0;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  double max_distance_sqr = 0.0;
  for (int i = s + 1; i < t; ++i) {
    max_distance_sqr =
        std::max(max_distance_sqr, segment.DistanceSquareTo(points[i]));
  }
  return sqrt(max_distance_sqr);
}

bool PathApproximation::is_within_max_error(const Path& path, const int s,
                                            const int t) {
  if (s + 1 >= t) {
    return true;
  }
  const auto& points = path.path_points();
  const LineSegment2d segment(points[s], points[t]);
  for (int i = s + 1; i < t; ++i) {
    if (segment.DistanceSquareTo(points[i]) > max_sqr_error_) {
      return false;
    }
  }
  return true;
}

void PathApproximation::Init(const Path& path) {
  InitDilute(path);
  InitProjections(path);
}

void PathApproximation::InitDilute(const Path& path) {
  const int num_original_points = path.num_points();
  original_ids_.clear();
  int last_idx = 0;
  while (last_idx < num_original_points - 1) {
    original_ids_.push_back(last_idx);
    int next_idx = last_idx + 1;
    int delta = 2;
    for (; last_idx + delta < num_original_points; delta *= 2) {
      if (!is_within_max_error(path, last_idx, last_idx + delta)) {
        break;
      }
      next_idx = last_idx + delta;
    }
    for (; delta > 0; delta /= 2) {
      if (next_idx + delta < num_original_points &&
          is_within_max_error(path, last_idx, next_idx + delta)) {
        next_idx += delta;
      }
    }
    last_idx = next_idx;
  }
  original_ids_.push_back(last_idx);
  num_points_ = static_cast<int>(original_ids_.size());
  if (num_points_ == 0) {
    return;
  }

  segments_.clear();
  segments_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    segments_.emplace_back(path.path_points()[original_ids_[i]],
                           path.path_points()[original_ids_[i + 1]]);
  }
  max_error_per_segment_.clear();
  max_error_per_segment_.reserve(num_points_ - 1);
  for (int i = 0; i < num_points_ - 1; ++i) {
    max_error_per_segment_.push_back(
        compute_max_error(path, original_ids_[i], original_ids_[i + 1]));
  }
}

void PathApproximation::InitProjections(const Path& path) {
  if (num_points_ == 0) {
    return;
  }
  projections_.clear();
  projections_.reserve(segments_.size() + 1);
  double s = 0.0;
  projections_.push_back(0);
  for (const auto& segment : segments_) {
    s += segment.length();
    projections_.push_back(s);
  }
  const auto& original_points = path.path_points();
  const int num_original_points = static_cast<int>(original_points.size());
  original_projections_.clear();
  original_projections_.reserve(num_original_points);
  for (size_t i = 0; i < projections_.size(); ++i) {
    original_projections_.push_back(projections_[i]);
    if (i + 1 < projections_.size()) {
      const auto& segment = segments_[i];
      for (int idx = original_ids_[i] + 1; idx < original_ids_[i + 1]; ++idx) {
        const double proj = segment.ProjectOntoUnit(original_points[idx]);
        original_projections_.push_back(
            projections_[i] + std::max(0.0, std::min(proj, segment.length())));
      }
    }
  }

  // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
  max_original_projections_to_left_.resize(num_original_points);
  double last_projection = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_original_points; ++i) {
    last_projection = std::max(last_projection, original_projections_[i]);
    max_original_projections_to_left_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(max_original_projections_to_left_[i],
             max_original_projections_to_left_[i + 1] + kMathEpsilon);
  }

  // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
  min_original_projections_to_right_.resize(original_projections_.size());
  last_projection = std::numeric_limits<double>::infinity();
  for (int i = num_original_points - 1; i >= 0; --i) {
    last_projection = std::min(last_projection, original_projections_[i]);
    min_original_projections_to_right_[i] = last_projection;
  }
  for (int i = 0; i + 1 < num_original_points; ++i) {
    CHECK_LE(min_original_projections_to_right_[i],
             min_original_projections_to_right_[i + 1] + kMathEpsilon);
  }

  // Sample max_p_to_left by sample_distance.
  max_projection_ = projections_.back();
  num_projection_samples_ =
      static_cast<int>(max_projection_ / kSampleDistance) + 1;
  sampled_max_original_projections_to_left_.clear();
  sampled_max_original_projections_to_left_.reserve(num_projection_samples_);
  double proj = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_projection_samples_; ++i) {
    while (last_index + 1 < num_original_points &&
           max_original_projections_to_left_[last_index + 1] < proj) {
      ++last_index;
    }
    sampled_max_original_projections_to_left_.push_back(last_index);
    proj += kSampleDistance;
  }
  CHECK_EQ(sampled_max_original_projections_to_left_.size(),
           num_projection_samples_);
}

bool PathApproximation::GetProjection(const Path& path,
                                      const common::math::Vec2d& point,
                                      double* accumulate_s, double* lateral,
                                      double* min_distance) const {
  if (num_points_ == 0) {
    return false;
  }
  if (accumulate_s == nullptr || lateral == nullptr ||
      min_distance == nullptr) {
    return false;
  }
  double min_distance_sqr = std::numeric_limits<double>::infinity();
  int estimate_nearest_segment_idx = -1;
  std::vector<double> distance_sqr_to_segments;
  distance_sqr_to_segments.reserve(segments_.size());
  for (size_t i = 0; i < segments_.size(); ++i) {
    const double distance_sqr = segments_[i].DistanceSquareTo(point);
    distance_sqr_to_segments.push_back(distance_sqr);
    if (distance_sqr < min_distance_sqr) {
      min_distance_sqr = distance_sqr;
      estimate_nearest_segment_idx = static_cast<int>(i);
    }
  }
  if (estimate_nearest_segment_idx < 0) {
    return false;
  }
  const auto& original_segments = path.segments();
  const int num_original_segments = static_cast<int>(original_segments.size());
  const auto& original_accumulated_s = path.accumulated_s();
  double min_distance_sqr_with_error =
      Sqr(sqrt(min_distance_sqr) +
          max_error_per_segment_[estimate_nearest_segment_idx] + max_error_);
  *min_distance = std::numeric_limits<double>::infinity();
  int nearest_segment_idx = -1;
  for (size_t i = 0; i < segments_.size(); ++i) {
    if (distance_sqr_to_segments[i] >= min_distance_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(point);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(point));
      if (prod_sqr >= min_distance_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(min_distance_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    bool min_distance_updated = false;
    bool is_within_end_point = false;
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      const double x0 = point.x() - original_segment.start().x();
      const double y0 = point.y() - original_segment.start().y();
      const double ux = original_segment.unit_direction().x();
      const double uy = original_segment.unit_direction().y();
      double proj = x0 * ux + y0 * uy;
      double distance = 0.0;
      if (proj < 0.0) {
        if (is_within_end_point) {
          continue;
        }
        is_within_end_point = true;
        distance = hypot(x0, y0);
      } else if (proj <= original_segment.length()) {
        is_within_end_point = true;
        distance = std::abs(x0 * uy - y0 * ux);
      } else {
        is_within_end_point = false;
        if (idx != last_segment_idx) {
          continue;
        }
        distance = original_segment.end().DistanceTo(point);
      }
      if (distance < *min_distance) {
        min_distance_updated = true;
        *min_distance = distance;
        nearest_segment_idx = idx;
      }
    }
    if (min_distance_updated) {
      min_distance_sqr_with_error = Sqr(*min_distance + max_error_);
    }
  }
  if (nearest_segment_idx >= 0) {
    const auto& segment = original_segments[nearest_segment_idx];
    double proj = segment.ProjectOntoUnit(point);
    const double prod = segment.ProductOntoUnit(point);
    if (nearest_segment_idx > 0) {
      proj = std::max(0.0, proj);
    }
    if (nearest_segment_idx + 1 < num_original_segments) {
      proj = std::min(segment.length(), proj);
    }
    *accumulate_s = original_accumulated_s[nearest_segment_idx] + proj;
    if ((nearest_segment_idx == 0 && proj < 0.0) ||
        (nearest_segment_idx + 1 == num_original_segments &&
         proj > segment.length())) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0 ? (*min_distance) : -(*min_distance));
    }
    return true;
  }
  return false;
}

bool PathApproximation::OverlapWith(const Path& path, const Box2d& box,
                                    double width) const {
  if (num_points_ == 0) {
    return false;
  }
  const Vec2d center = box.center();
  const double radius = box.diagonal() / 2.0 + width;
  const double radius_sqr = Sqr(radius);
  const auto& original_segments = path.segments();
  for (size_t i = 0; i < segments_.size(); ++i) {
    const LineSegment2d& segment = segments_[i];
    const double max_error = max_error_per_segment_[i];
    const double radius_sqr_with_error = Sqr(radius + max_error);
    if (segment.DistanceSquareTo(center) > radius_sqr_with_error) {
      continue;
    }
    int first_segment_idx = original_ids_[i];
    int last_segment_idx = original_ids_[i + 1] - 1;
    double max_original_projection = std::numeric_limits<double>::infinity();
    if (first_segment_idx < last_segment_idx) {
      const auto& segment = segments_[i];
      const double projection = segment.ProjectOntoUnit(center);
      const double prod_sqr = Sqr(segment.ProductOntoUnit(center));
      if (prod_sqr >= radius_sqr_with_error) {
        continue;
      }
      const double scan_distance = sqrt(radius_sqr_with_error - prod_sqr);
      const double min_projection = projection - scan_distance;
      max_original_projection = projections_[i] + projection + scan_distance;
      if (min_projection > 0.0) {
        const double limit = projections_[i] + min_projection;
        const int sample_index =
            std::max(0, static_cast<int>(limit / kSampleDistance));
        if (sample_index >= num_projection_samples_) {
          first_segment_idx = last_segment_idx;
        } else {
          first_segment_idx =
              std::max(first_segment_idx,
                       sampled_max_original_projections_to_left_[sample_index]);
          if (first_segment_idx >= last_segment_idx) {
            first_segment_idx = last_segment_idx;
          } else {
            while (first_segment_idx < last_segment_idx &&
                   max_original_projections_to_left_[first_segment_idx + 1] <
                       limit) {
              ++first_segment_idx;
            }
          }
        }
      }
    }
    for (int idx = first_segment_idx; idx <= last_segment_idx; ++idx) {
      if (min_original_projections_to_right_[idx] > max_original_projection) {
        break;
      }
      const auto& original_segment = original_segments[idx];
      if (original_segment.DistanceSquareTo(center) > radius_sqr) {
        continue;
      }
      if (box.DistanceTo(original_segment) <= width) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace hdmap
}  // namespace acu
