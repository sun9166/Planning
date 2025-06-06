/**
 * @file path.h
 **/

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "map_lane.pb.h"

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/math/line_segment2d.h"
#include "src/execution/motionplan/common/datatype.h"
#include "datapool/include/cognition_typedef.h"
#include "map/vectormap/src/hdmap/hdmap_common.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"

namespace acu {
namespace hdmap {

struct LaneWaypoint {
  LaneWaypoint() = default;
  LaneWaypoint(LaneInfoConstPtr lane, const double s)
      : lane(lane), s(s) {}

  LaneWaypoint(LaneInfoConstPtr lane, const double s, const double l)
      : lane(lane), s(s), l(l) {}
  LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;
};

struct LaneSegment {
  LaneSegment() = default;
  LaneSegment(LaneInfoConstPtr lane, const double start_s, const double end_s)
      : lane(lane), start_s(start_s), end_s(end_s) {}
  LaneInfoConstPtr lane = nullptr;
  double start_s = 0.0;
  double end_s = 0.0;
  double Length() const { return end_s - start_s; }

  /**
   * Join neighboring lane segments if they have the same lane id
   */
  static void Join(std::vector<LaneSegment>* segments);

};

struct PathOverlap {
  PathOverlap() = default;
  PathOverlap(std::string object_id, const double start_s, const double end_s)
      : object_id(std::move(object_id)), start_s(start_s), end_s(end_s) {}

  std::string object_id;
  double start_s = 0.0;
  double end_s = 0.0;
};

class MapPathPoint : public common::math::Vec2d {
 public:
  MapPathPoint() = default;
  MapPathPoint(const common::math::Vec2d& point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}
  MapPathPoint(const common::math::Vec2d& point, double heading,
               LaneWaypoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }

  MapPathPoint(const common::math::Vec2d& point,double heading,double kappa, 
      double dkappa)
      : Vec2d(point.x(), point.y()),heading_(heading),kappa_(kappa),
        dkappa_(dkappa) {

        }  //@ pqg adapt to acu

  MapPathPoint(const common::math::Vec2d& point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()),
        heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

  double heading() const { return heading_; }
  double kappa() const { return kappa_; }//@pqg 
  double dkappa() const { return dkappa_; } //@pqg
  void set_heading(const double heading) { heading_ = heading; }

  const std::vector<LaneWaypoint>& lane_waypoints() const {
    return lane_waypoints_;
  }

  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  void add_lane_waypoints(const std::vector<LaneWaypoint>& lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  void clear_lane_waypoints() { lane_waypoints_.clear(); }

  static void RemoveDuplicates(std::vector<MapPathPoint>* points);

  static std::vector<MapPathPoint> GetPointsFromSegment(
      const LaneSegment& segment);

  static std::vector<MapPathPoint> GetPointsFromLane(LaneInfoConstPtr lane,
                                                     const double start_s,
                                                     const double end_s);

 protected:
  double heading_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};

class Path;

class PathApproximation {
 public:
  PathApproximation() = default;
  PathApproximation(const Path& path, const double max_error)
      : max_error_(max_error), max_sqr_error_(max_error * max_error) {
    Init(path);
  }
  double max_error() const { return max_error_; }
  const std::vector<int>& original_ids() const { return original_ids_; }
  const std::vector<common::math::LineSegment2d>& segments() const {
    return segments_;
  }

  bool GetProjection(const Path& path, const common::math::Vec2d& point,
                     double* accumulate_s, double* lateral,
                     double* distance) const;

  bool OverlapWith(const Path& path, const common::math::Box2d& box,
                   double width) const;

 protected:
  void Init(const Path& path);
  bool is_within_max_error(const Path& path, const int s, const int t);
  double compute_max_error(const Path& path, const int s, const int t);

  void InitDilute(const Path& path);
  void InitProjections(const Path& path);

 protected:
  double max_error_ = 0;
  double max_sqr_error_ = 0;

  int num_points_ = 0;
  std::vector<int> original_ids_;
  std::vector<common::math::LineSegment2d> segments_;
  std::vector<double> max_error_per_segment_;

  // TODO(@lianglia_apollo): use direction change checks to early stop.

  // Projection of points onto the diluated segments.
  std::vector<double> projections_;
  double max_projection_;
  int num_projection_samples_ = 0;

  // The original_projection is the projection of original points onto the
  // diluated segments.
  std::vector<double> original_projections_;
  // max_p_to_left[i] = max(p[0], p[1], ... p[i]).
  // min_p_to_right[i] = min(p[i], p[i + 1], ... p[size - 1]).
  std::vector<double> max_original_projections_to_left_;
  std::vector<double> min_original_projections_to_right_;
  std::vector<int> sampled_max_original_projections_to_left_;
};

class InterpolatedIndex {
 public:
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
  int id = 0;
  double offset = 0.0;
};

class Path {
 public:
  Path() = default;
  explicit Path(const std::vector<MapPathPoint>& path_points);
  explicit Path(std::vector<MapPathPoint>&& path_points);
  explicit Path(std::vector<LaneSegment>&& path_points);
  explicit Path(const planning::ReferenceLineFrame* ref_path);
  explicit Path(const planning::ReferenceLineFrame* ref_path,const std::vector<MapPathPoint>& path_points);

  Path(const std::vector<MapPathPoint>& path_points,
       const std::vector<LaneSegment>& lane_segments);
  Path(std::vector<MapPathPoint>&& path_points,
       std::vector<LaneSegment>&& lane_segments);

  Path(const std::vector<MapPathPoint>& path_points,
       const std::vector<LaneSegment>& lane_segments,
       const double max_approximation_error);
  Path(std::vector<MapPathPoint>&& path_points,
       std::vector<LaneSegment>&& lane_segments,
       const double max_approximation_error);

  // Return smooth coordinate by interpolated index or accumulate_s.
  MapPathPoint GetSmoothPoint(const InterpolatedIndex& index) const;
  MapPathPoint GetSmoothPoint(double s) const;

  // Compute accumulate s value of the index.
  double GetSFromIndex(const InterpolatedIndex& index) const;
  // Compute interpolated index by accumulate_s.
  InterpolatedIndex GetIndexFromS(double s) const;

  // get the index of the lane from s by accumulate_s
  InterpolatedIndex GetLaneIndexFromS(double s) const;

  std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
                                                  const double end_s) const;

  bool GetNearestPoint(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral) const;
  bool GetNearestPoint(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral, double* distance) const;
  bool GetProjectionWithHueristicParams(const common::math::Vec2d& point,
                                        const double hueristic_start_s,
                                        const double hueristic_end_s,
                                        double* accumulate_s, double* lateral,
                                        double* min_distance) const;
  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral) const;
  bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                     double* lateral, double* distance) const;

  bool GetHeadingAlongPath(const common::math::Vec2d& point,
                           double* heading) const;
  double GetOffsetToMapLine(const double& s) const;

  int num_points() const { return num_points_; }
  int num_segments() const { return num_segments_; }
  const std::vector<MapPathPoint>& path_points() const { return path_points_; }
  const std::vector<LaneSegment>& lane_segments() const {
    return lane_segments_;
  }
  const std::vector<LaneSegment>& lane_segments_to_next_point() const {
    return lane_segments_to_next_point_;
  }
  const std::vector<common::math::Vec2d>& unit_directions() const {
    return unit_directions_;
  }
  const std::vector<double>& accumulated_s() const { return accumulated_s_; }
  const std::vector<common::math::LineSegment2d>& segments() const {
    return segments_;
  }
  const PathApproximation* approximation() const { return &approximation_; }
  double length() const { return length_; }

  double property_length() const {
     return property_length_;
  } 

  double offset_property_length() const {
     return offset_property_length_;
  } 

  const planning::ReferenceLineFrame* raw_refrence_line() const {
    return reference_line_;
  }
  
  bool is_parking_path() const {
    return is_parking_path_;
  } 
  


  const PathOverlap* NextLaneOverlap(double s) const;

  const std::vector<PathOverlap>& lane_overlaps() const {
    return lane_overlaps_;
  }
  const std::vector<PathOverlap>& signal_overlaps() const {
    return signal_overlaps_;
  }
  const std::vector<PathOverlap>& yield_sign_overlaps() const {
    return yield_sign_overlaps_;
  }
  const std::vector<PathOverlap>& stop_sign_overlaps() const {
    return stop_sign_overlaps_;
  }
  const std::vector<PathOverlap>& crosswalk_overlaps() const {
    return crosswalk_overlaps_;
  }
  const std::vector<PathOverlap>& junction_overlaps() const {
    return junction_overlaps_;
  }
  const std::vector<PathOverlap>& pnc_junction_overlaps() const {
    return pnc_junction_overlaps_;
  }
  const std::vector<PathOverlap>& clear_area_overlaps() const {
    return clear_area_overlaps_;
  }
  const std::vector<PathOverlap>& speed_bump_overlaps() const {
    return speed_bump_overlaps_;
  }
  const std::vector<PathOverlap>& parking_space_overlaps() const {
    return parking_space_overlaps_;
  }

  double GetLaneLeftWidth(const double s) const;
  double GetLaneRightWidth(const double s) const;
  bool GetLaneWidth(const double s, double* lane_left_width,
                    double* lane_right_width) const;
  bool GetFreeSpaceboundary(const double s, double* left_width,
                        double* right_width) const; 

  double GetRoadLeftWidth(const double s) const;
  double GetRoadRightWidth(const double s) const;
  bool GetRoadWidth(const double s, double* road_left_width,
                    double* road_ight_width) const;
  bool GetRoadWidth(const Site &point, double* road_left_width,
                    double* road_right_width) const;
  bool GetRoadBoundary(const double s, double* road_left_boundary,
                    double* road_right_boundary) const;//包含非机动车道的road边界
  bool GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, double s) const;

  bool IsOnPath(const common::math::Vec2d& point) const;
  bool OverlapWith(const common::math::Box2d& box, double width) const;

 protected:
  void Init();
  void InitPoints();
  void InitLaneSegments();
  void InitWidth();
  void InitPointIndex();
  void InitOverlaps();

  double GetSample(const std::vector<double>& samples, const double s) const;

  void GetCurvature(geometry::SiteVec& trajectory);
  void CalculateLengthOfPoint(geometry::SiteVec& trajectory);
  double GetDist(double x0, double y0, double x1, double y1);

  using GetOverlapFromLaneFunc =
      std::function<const std::vector<OverlapInfoConstPtr>&(const LaneInfo&)>;
  void GetAllOverlaps(GetOverlapFromLaneFunc GetOverlaps_from_lane,
                      std::vector<PathOverlap>* const overlaps) const;

  void ExtendPath( const geometry::SiteVec& ref_path_points, geometry::SiteVec& input_path,
    const geometry::Site& ego, const double& extend_length);

 protected:
  int num_points_ = 0;
  int num_segments_ = 0;
  std::vector<MapPathPoint> path_points_;
  std::vector<LaneSegment> lane_segments_;
  std::vector<double> lane_accumulated_s_;
  std::vector<LaneSegment> lane_segments_to_next_point_;
  std::vector<common::math::Vec2d> unit_directions_;
  double length_ = 0.0;
  std::vector<double> accumulated_s_;
  std::vector<common::math::LineSegment2d> segments_;
  bool use_path_approximation_ = false;
  PathApproximation approximation_;

  // Sampled every fixed length.
  int num_sample_points_ = 0;
  std::vector<double> lane_left_width_;
  std::vector<double> lane_right_width_;
  std::vector<double> road_left_width_;
  std::vector<double> road_right_width_;
  std::vector<int> last_point_index_;

  std::vector<PathOverlap> lane_overlaps_;
  std::vector<PathOverlap> signal_overlaps_;
  std::vector<PathOverlap> yield_sign_overlaps_;
  std::vector<PathOverlap> stop_sign_overlaps_;
  std::vector<PathOverlap> crosswalk_overlaps_;
  std::vector<PathOverlap> parking_space_overlaps_;
  std::vector<PathOverlap> junction_overlaps_;
  std::vector<PathOverlap> pnc_junction_overlaps_;
  std::vector<PathOverlap> clear_area_overlaps_;
  std::vector<PathOverlap> speed_bump_overlaps_;

  double curve_end_ = 0;
  double property_length_ = -1;
  double offset_property_length_ = -1;

  const planning::ReferenceLineFrame* reference_line_;
  bool is_parking_path_ = false;
  int id_; 
  double s_offset_on_raw_line_ = 0.0 ; 
};

}  // namespace hdmap
}  // namespace acu
