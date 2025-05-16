/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#ifndef MODULES_MAP_HDMAP_HDMAP_COMMON_H_
#define MODULES_MAP_HDMAP_HDMAP_COMMON_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/math/aabox2d.h"
#include "common/math/aaboxkdtree2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "map_clear_area.pb.h"
#include "map_crosswalk.pb.h"
#include "map_id.pb.h"
#include "map_junction.pb.h"
#include "map_lane.pb.h"
#include "map_overlap.pb.h"
#include "map_road.pb.h"
#include "map_signal.pb.h"
#include "map_speed_bump.pb.h"
#include "map_stop_sign.pb.h"
#include "map_yield_sign.pb.h"
#include "map_isolationbelt.pb.h"
#include "map_guardrail.pb.h"
#include "map_inner.pb.h"
#include "map_outer.pb.h"

/**
 * @namespace acu::hdmap
 * @brief acu::hdmap
 */
namespace acu {
namespace hdmap {

template <class Object, class GeoObject>
class ObjectWithAABox {
 public:
  ObjectWithAABox(const acu::common::math::AABox2d &aabox,
                  const Object *object, const GeoObject *geo_object,
                  const int id)
      : aabox_(aabox), object_(object), geo_object_(geo_object), id_(id) {}
  ~ObjectWithAABox() {}
  const acu::common::math::AABox2d &aabox() const { return aabox_; }
  double DistanceTo(const acu::common::math::Vec2d &point) const {
    return geo_object_->DistanceTo(point);
  }
  double DistanceSquareTo(const acu::common::math::Vec2d &point) const {
    return geo_object_->DistanceSquareTo(point);
  }
  const Object *object() const { return object_; }
  const GeoObject *geo_object() const { return geo_object_; }
  int id() const { return id_; }

 private:
  acu::common::math::AABox2d aabox_;
  const Object *object_;
  const GeoObject *geo_object_;
  int id_;
};

class LaneInfo;
class JunctionInfo;
class CrosswalkInfo;
class SignalInfo;
class StopSignInfo;
class YieldSignInfo;
class OverlapInfo;
class ClearAreaInfo;
class SpeedBumpInfo;
class RoadInfo;
class IsolationBeltInfo;
class GuardrailInfo;
class InnerInfo;
class OuterInfo;

class HDMapImpl;

using LaneSegmentBox =
    ObjectWithAABox<LaneInfo, acu::common::math::LineSegment2d>;
using LaneSegmentKDTree = acu::common::math::AABoxKDTree2d<LaneSegmentBox>;

using OverlapInfoConstPtr = std::shared_ptr<const OverlapInfo>;
using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;
using JunctionInfoConstPtr = std::shared_ptr<const JunctionInfo>;
using SignalInfoConstPtr = std::shared_ptr<const SignalInfo>;
using CrosswalkInfoConstPtr = std::shared_ptr<const CrosswalkInfo>;
using StopSignInfoConstPtr = std::shared_ptr<const StopSignInfo>;
using YieldSignInfoConstPtr = std::shared_ptr<const YieldSignInfo>;
using ClearAreaInfoConstPtr = std::shared_ptr<const ClearAreaInfo>;
using SpeedBumpInfoConstPtr = std::shared_ptr<const SpeedBumpInfo>;
using RoadInfoConstPtr = std::shared_ptr<const RoadInfo>;
using RoadROIBoundaryPtr = std::shared_ptr<RoadROIBoundary>;
using IsolationBeltInfoConstPtr = std::shared_ptr<const IsolationBeltInfo>;
using GuardrailInfoConstPtr = std::shared_ptr<const GuardrailInfo>;
using InnerInfoConstPtr = std::shared_ptr<const InnerInfo>;
using OuterInfoConstPtr = std::shared_ptr<const OuterInfo>;

class LaneInfo {
 public:
  explicit LaneInfo(const Lane &lane);

  const Id &id() const { return lane_.id(); }
  const Id &road_id() const { return road_id_; }
  const Id &section_id() const { return section_id_; }
  const Lane &lane() const { return lane_; }
  const std::vector<acu::common::math::Vec2d> &points() const {
    return points_;
  }
  const std::vector<acu::common::math::Vec2d> &unit_directions() const {
    return unit_directions_;
  }
  acu::common::math::Vec2d Point(const double s) const;
  double Heading(const double s) const;
  double Curvature(const double s) const;
  const std::vector<double> &headings() const { return headings_; }
  const std::vector<acu::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<double> &accumulate_s() const { return accumulated_s_; }
  const std::vector<OverlapInfoConstPtr> &overlaps() const { return overlaps_; }
  const std::vector<OverlapInfoConstPtr> &cross_lanes() const {
    return cross_lanes_;
  }
  const std::vector<OverlapInfoConstPtr> &signals() const { return signals_; }
  const std::vector<OverlapInfoConstPtr> &yield_signs() const {
    return yield_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &stop_signs() const {
    return stop_signs_;
  }
  const std::vector<OverlapInfoConstPtr> &crosswalks() const {
    return crosswalks_;
  }
  const std::vector<OverlapInfoConstPtr> &junctions() const {
    return junctions_;
  }
  const std::vector<OverlapInfoConstPtr> &clear_areas() const {
    return clear_areas_;
  }
  const std::vector<OverlapInfoConstPtr> &speed_bumps() const {
    return speed_bumps_;
  }
  double total_length() const { return total_length_; }
  using SampledWidth = std::pair<double, double>;
  const std::vector<SampledWidth> &sampled_left_width() const {
    return sampled_left_width_;
  }
  const std::vector<SampledWidth> &sampled_right_width() const {
    return sampled_right_width_;
  }
  void GetWidth(const double s, double *left_width, double *right_width) const;
  double GetWidth(const double s) const;
  double GetEffectiveWidth(const double s) const;

  const std::vector<SampledWidth> &sampled_left_road_width() const {
    return sampled_left_road_width_;
  }
  const std::vector<SampledWidth> &sampled_right_road_width() const {
    return sampled_right_road_width_;
  }
  void GetRoadWidth(const double s, double *left_width,
                   double *right_width) const;
  double GetRoadWidth(const double s) const;

  bool IsOnLane(const acu::common::math::Vec2d &point) const;
  bool IsOnLane(const acu::common::math::Box2d &box) const;

  acu::common::PointENU GetSmoothPoint(double s) const;
  double DistanceTo(const acu::common::math::Vec2d &point) const;
  double DistanceTo(const acu::common::math::Vec2d &point,
                    acu::common::math::Vec2d *map_point, double *s_offset,
                    int *s_offset_index) const;
  acu::common::PointENU GetNearestPoint(
      const acu::common::math::Vec2d &point, double *distance) const;
  bool GetProjection(const acu::common::math::Vec2d &point,
                     double *accumulate_s, double *lateral) const;

 private:
  friend class HDMapImpl;
  friend class RoadInfo;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);
  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;
  void CreateKDTree();
  void set_road_id(const Id &road_id) { road_id_ = road_id; }
  void set_section_id(const Id &section_id) { section_id_ = section_id; }

  int PathDensify(std::vector<acu::common::math::Vec2d> &points_init, 
        std::vector<acu::common::math::Vec2d> &points_out, 
        const double density);

 private:
  const Lane &lane_;
  std::vector<acu::common::math::Vec2d> points_;
  std::vector<acu::common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  std::vector<acu::common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<std::string> overlap_ids_;
  std::vector<OverlapInfoConstPtr> overlaps_;
  std::vector<OverlapInfoConstPtr> cross_lanes_;
  std::vector<OverlapInfoConstPtr> signals_;
  std::vector<OverlapInfoConstPtr> yield_signs_;
  std::vector<OverlapInfoConstPtr> stop_signs_;
  std::vector<OverlapInfoConstPtr> crosswalks_;
  std::vector<OverlapInfoConstPtr> parking_spaces_;
  std::vector<OverlapInfoConstPtr> junctions_;
  std::vector<OverlapInfoConstPtr> clear_areas_;
  std::vector<OverlapInfoConstPtr> speed_bumps_;
  double total_length_ = 0.0;
  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;

  std::vector<LaneSegmentBox> segment_box_list_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  Id road_id_;
  Id section_id_;
};

class JunctionInfo {
 public:
  explicit JunctionInfo(const Junction &junction);

  const Id &id() const { return junction_.id(); }
  const Junction &junction() const { return junction_; }
  const acu::common::math::Polygon2d &polygon() const { return polygon_; }

  const std::vector<Id>& OverlapStopSignIds() const {
    return overlap_stop_sign_ids_;
  }

 private:
  friend class HDMapImpl;
  void Init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const Junction &junction_;
  acu::common::math::Polygon2d polygon_;

  std::vector<Id> overlap_stop_sign_ids_;
  std::vector<Id> overlap_ids_;
};
using JunctionPolygonBox =
    ObjectWithAABox<JunctionInfo, acu::common::math::Polygon2d>;
using JunctionPolygonKDTree =
    acu::common::math::AABoxKDTree2d<JunctionPolygonBox>;

class SignalInfo {
 public:
  explicit SignalInfo(const Signal &signal);

  const Id &id() const { return signal_.id(); }
  const Signal &signal() const { return signal_; }
  const std::vector<acu::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const Signal &signal_;
  std::vector<acu::common::math::LineSegment2d> segments_;
};
using SignalSegmentBox =
    ObjectWithAABox<SignalInfo, acu::common::math::LineSegment2d>;
using SignalSegmentKDTree =
    acu::common::math::AABoxKDTree2d<SignalSegmentBox>;

class CrosswalkInfo {
 public:
  explicit CrosswalkInfo(const Crosswalk &crosswalk);

  const Id &id() const { return crosswalk_.id(); }
  const Crosswalk &crosswalk() const { return crosswalk_; }
  const acu::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Crosswalk &crosswalk_;
  acu::common::math::Polygon2d polygon_;
};
using CrosswalkPolygonBox =
    ObjectWithAABox<CrosswalkInfo, acu::common::math::Polygon2d>;
using CrosswalkPolygonKDTree =
    acu::common::math::AABoxKDTree2d<CrosswalkPolygonBox>;

class StopSignInfo {
 public:
  explicit StopSignInfo(const StopSign &stop_sign);

  const Id &id() const { return stop_sign_.id(); }
  const StopSign &stop_sign() const { return stop_sign_; }
  const std::vector<acu::common::math::LineSegment2d> &segments() const {
    return segments_;
  }
  const std::vector<Id>& OverlapLaneIds() const { return overlap_lane_ids_; }
  const std::vector<Id>& OverlapJunctionIds() const {
    return overlap_junction_ids_; }

 private:
  friend class HDMapImpl;
  void init();
  void PostProcess(const HDMapImpl &map_instance);
  void UpdateOverlaps(const HDMapImpl &map_instance);

 private:
  const StopSign &stop_sign_;
  std::vector<acu::common::math::LineSegment2d> segments_;

  std::vector<Id> overlap_lane_ids_;
  std::vector<Id> overlap_junction_ids_;
  std::vector<Id> overlap_ids_;
};
using StopSignSegmentBox =
    ObjectWithAABox<StopSignInfo, acu::common::math::LineSegment2d>;
using StopSignSegmentKDTree =
    acu::common::math::AABoxKDTree2d<StopSignSegmentBox>;

class YieldSignInfo {
 public:
  explicit YieldSignInfo(const YieldSign &yield_sign);

  const Id &id() const { return yield_sign_.id(); }
  const YieldSign &yield_sign() const { return yield_sign_; }
  const std::vector<acu::common::math::LineSegment2d> &segments() const {
    return segments_;
  }

 private:
  void Init();

 private:
  const YieldSign &yield_sign_;
  std::vector<acu::common::math::LineSegment2d> segments_;
};
using YieldSignSegmentBox =
    ObjectWithAABox<YieldSignInfo, acu::common::math::LineSegment2d>;
using YieldSignSegmentKDTree =
    acu::common::math::AABoxKDTree2d<YieldSignSegmentBox>;

class ClearAreaInfo {
 public:
  explicit ClearAreaInfo(const ClearArea &clear_area);

  const Id &id() const { return clear_area_.id(); }
  const ClearArea &clear_area() const { return clear_area_; }
  const acu::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const ClearArea &clear_area_;
  acu::common::math::Polygon2d polygon_;
};
using ClearAreaPolygonBox =
    ObjectWithAABox<ClearAreaInfo, acu::common::math::Polygon2d>;
using ClearAreaPolygonKDTree =
    acu::common::math::AABoxKDTree2d<ClearAreaPolygonBox>;

class SpeedBumpInfo {
 public:
  explicit SpeedBumpInfo(const SpeedBump &speed_bump);

  const Id &id() const { return speed_bump_.id(); }
  const SpeedBump &speed_bump() const { return speed_bump_; }
  const acu::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const SpeedBump &speed_bump_;
  acu::common::math::Polygon2d polygon_;
};
using SpeedBumpPolygonBox =
    ObjectWithAABox<SpeedBumpInfo, acu::common::math::Polygon2d>;
using SpeedBumpPolygonKDTree =
    acu::common::math::AABoxKDTree2d<SpeedBumpPolygonBox>;

class OverlapInfo {
 public:
  explicit OverlapInfo(const Overlap &overlap);

  const Id &id() const { return overlap_.id(); }
  const Overlap &overlap() const { return overlap_; }
  const ObjectOverlapInfo *GetObjectOverlapInfo(const Id &id) const;

 private:
  const Overlap &overlap_;
};

class RoadInfo {
 public:
  explicit RoadInfo(const Road &road);
  const Id &id() const { return road_.id(); }
  const Road &road() const { return road_; }
  const std::vector<RoadSection> &sections() const { return sections_; }

  const Id &junction_id() const { return road_.junction_id(); }
  bool has_junction_id() const { return road_.has_junction_id(); }

  const std::vector<RoadBoundary> &GetBoundaries() const;

 private:
  Road road_;
  std::vector<RoadSection> sections_;
  std::vector<RoadBoundary> road_boundaries_;
};

struct JunctionBoundary {
  JunctionInfoConstPtr junction_info;
};

using JunctionBoundaryPtr = std::shared_ptr<JunctionBoundary>;

class IsolationBeltInfo {
 public:
  explicit IsolationBeltInfo(const Isolationbelt &isolationbelt);
  const Id &id() const { return isolationbelt_.id(); }
  const Isolationbelt &isolationbelt() const { return isolationbelt_; }
  const acu::common::math::Polygon2d &polygon() const { return polygon_; }

 private:
  void Init();

 private:
  const Isolationbelt &isolationbelt_;
  acu::common::math::Polygon2d polygon_;
};
using IsolationBeltPolygonBox =
    ObjectWithAABox<IsolationBeltInfo, acu::common::math::Polygon2d>;
using IsolationBeltPolygonKDTree =
    acu::common::math::AABoxKDTree2d<IsolationBeltPolygonBox>;

class GuardrailInfo {
  public:
    explicit GuardrailInfo(const Guardrail &guardrail);
    const Id &id() const { return guardrail_.id(); }
    const Guardrail &guardrail() const { return guardrail_; }
    const acu::common::math::Polygon2d &polygon() const { return polygon_; }
  
  private:
    void Init();

  private:
    const Guardrail &guardrail_;
    acu::common::math::Polygon2d polygon_;
};
using GuardrailPolygonBox = 
    ObjectWithAABox<GuardrailInfo, acu::common::math::Polygon2d>;
using GuardrailPolygonKDTree =
    acu::common::math::AABoxKDTree2d<GuardrailPolygonBox>;

class InnerInfo {
  public:
    explicit InnerInfo(const Inner &inner);
    const Id &id() const { return inner_.id(); }
    const Inner &inner() const { return inner_; }
    const acu::common::math::Polygon2d &polygon() const { return polygon_; }
  
  private:
    void Init();

  private:
    const Inner &inner_;
    acu::common::math::Polygon2d polygon_;
};
using InnerPolygonBox = 
    ObjectWithAABox<InnerInfo, acu::common::math::Polygon2d>;
using InnerPolygonKDTree =
    acu::common::math::AABoxKDTree2d<InnerPolygonBox>;

class OuterInfo {
  public:
    explicit OuterInfo(const Outer &outer);
    const Id &id() const { return outer_.id(); }
    const Outer &outer() const { return outer_; }
    const acu::common::math::Polygon2d &polygon() const { return polygon_; }
  
  private:
    void Init();

  private:
    const Outer &outer_;
    acu::common::math::Polygon2d polygon_;
};
using OuterPolygonBox = 
    ObjectWithAABox<OuterInfo, acu::common::math::Polygon2d>;
using OuterPolygonKDTree =
    acu::common::math::AABoxKDTree2d<OuterPolygonBox>;

}  // namespace hdmap
}  // namespace acu

#endif  // MODULES_MAP_HDMAP_HDMAP_COMMON_H_
