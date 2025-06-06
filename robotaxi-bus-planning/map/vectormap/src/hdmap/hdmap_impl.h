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

#ifndef MODULES_MAP_HDMAP_HDMAP_IMPL_H_
#define MODULES_MAP_HDMAP_HDMAP_IMPL_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/math/aabox2d.h"
#include "common/math/aaboxkdtree2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "map/vectormap/src/hdmap/hdmap_common.h"
#include "map.pb.h"
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

/**
 * @class HDMapImpl
 *
 * @brief High-precision map loader implement.
 */
class HDMapImpl {
 public:
  using LaneTable = std::unordered_map<std::string, std::shared_ptr<LaneInfo>>;
  using JunctionTable =
      std::unordered_map<std::string, std::shared_ptr<JunctionInfo>>;
  using SignalTable =
      std::unordered_map<std::string, std::shared_ptr<SignalInfo>>;
  using CrosswalkTable =
      std::unordered_map<std::string, std::shared_ptr<CrosswalkInfo>>;
  using StopSignTable =
      std::unordered_map<std::string, std::shared_ptr<StopSignInfo>>;
  using YieldSignTable =
      std::unordered_map<std::string, std::shared_ptr<YieldSignInfo>>;
  using ClearAreaTable =
      std::unordered_map<std::string, std::shared_ptr<ClearAreaInfo>>;
  using SpeedBumpTable =
      std::unordered_map<std::string, std::shared_ptr<SpeedBumpInfo>>;
  using OverlapTable =
      std::unordered_map<std::string, std::shared_ptr<OverlapInfo>>;
  using RoadTable = std::unordered_map<std::string, std::shared_ptr<RoadInfo>>;
  using IsolationBeltTable = 
      std::unordered_map<std::string, std::shared_ptr<IsolationBeltInfo>>;
  using GuardrailTable = 
      std::unordered_map<std::string, std::shared_ptr<GuardrailInfo>>;
  using InnerTable = 
      std::unordered_map<std::string, std::shared_ptr<InnerInfo>>;
  using OuterTable = 
      std::unordered_map<std::string, std::shared_ptr<OuterInfo>>;

 public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string& map_filename);

  /**
   * @brief load map from a protobuf message
   * @param map_proto map data in protobuf format
   * @return 0:success, otherwise failed
   */
  int LoadMapFromProto(const Map& map_proto);

  LaneInfoConstPtr GetLaneById(const Id& id) const;
  JunctionInfoConstPtr GetJunctionById(const Id& id) const;
  SignalInfoConstPtr GetSignalById(const Id& id) const;
  CrosswalkInfoConstPtr GetCrosswalkById(const Id& id) const;
  StopSignInfoConstPtr GetStopSignById(const Id& id) const;
  YieldSignInfoConstPtr GetYieldSignById(const Id& id) const;
  ClearAreaInfoConstPtr GetClearAreaById(const Id& id) const;
  SpeedBumpInfoConstPtr GetSpeedBumpById(const Id& id) const;
  OverlapInfoConstPtr GetOverlapById(const Id& id) const;
  RoadInfoConstPtr GetRoadById(const Id& id) const;
  IsolationBeltInfoConstPtr GetIsolationBeltById(const Id& id) const;
  GuardrailInfoConstPtr GetGuardrailById(const Id& id) const;
  InnerInfoConstPtr GetInnerById(const Id& id) const;
  OuterInfoConstPtr GetOuterById(const Id& id) const;

  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const acu::common::PointENU& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  /**
   * @brief get all junctions in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param junctions store all junctions in target range
   * @return 0:success, otherwise failed
   */
  int GetJunctions(const acu::common::PointENU& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  /**
   * @brief get all crosswalks in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param crosswalks store all crosswalks in target range
   * @return 0:success, otherwise failed
   */
  int GetCrosswalks(const acu::common::PointENU& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const acu::common::PointENU& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  /**
   * @brief get all stop signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param stop signs store all stop signs in target range
   * @return 0:success, otherwise failed
   */
  int GetStopSigns(const acu::common::PointENU& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  /**
   * @brief get all yield signs in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param yield signs store all yield signs in target range
   * @return 0:success, otherwise failed
   */
  int GetYieldSigns(const acu::common::PointENU& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  /**
   * @brief get all clear areas in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param clear_areas store all clear areas in target range
   * @return 0:success, otherwise failed
   */
  int GetClearAreas(const acu::common::PointENU& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  /**
   * @brief get all speed bumps in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all speed bumps in target range
   * @return 0:success, otherwise failed
   */
  int GetSpeedBumps(const acu::common::PointENU& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  /**
   * @brief get all isolationbelts in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all isolationbelts in target range
   * @return 0:success, otherwise failed
   */                 
  int GetIsolationBelts(const acu::common::PointENU& point, double distance,
                    std::vector<IsolationBeltInfoConstPtr>* isolationbelts) const;
  /**
   * @brief get all guardrails in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all guardrails in target range
   * @return 0:success, otherwise failed
   */                 
  int GetGuardrails(const acu::common::PointENU& point, double distance,
                    std::vector<GuardrailInfoConstPtr>* guardrails) const;
  /**
   * @brief get all inners in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all inners in target range
   * @return 0:success, otherwise failed
   */                 
  int GetInners(const acu::common::PointENU& point, double distance,
                    std::vector<InnerInfoConstPtr>* inners) const;
  /**
   * @brief get all outers in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param speed_bumps store all outers in target range
   * @return 0:success, otherwise failed
   */                 
  int GetOuters(const acu::common::PointENU& point, double distance,
                    std::vector<OuterInfoConstPtr>* outers) const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const acu::common::PointENU& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const acu::common::PointENU& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  /**
   * @brief get the nearest lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLaneWithHeading(const acu::common::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get the nearest cruise lane within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestCruiseLaneWithHeading(const acu::common::PointENU& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  /**
   * @brief get all lanes within a certain range by pose
   * @param point the target position
   * @param distance the search radius
   * @param central_heading the base heading
   * @param max_heading_difference the heading range
   * @param nearest_lane all lanes that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const acu::common::PointENU& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes,
                          std::vector<double>& distances) const;

  int GetLanesWithHeading(const acu::common::PointENU& point, 
                        const double distance, const double central_heading,
                        const double max_heading_difference,
                        std::vector<LaneInfoConstPtr>* lanes) const;

  /**
   * @brief get all road and junctions boundaries within certain range
   * @param point the target position
   * @param radius the search radius
   * @param road_boundaries the roads' boundaries
   * @param junctions the junctions' boundaries
   * @return 0:success, otherwise failed
   */
  int GetRoadBoundaries(const acu::common::PointENU& point, double radius,
                        std::vector<RoadROIBoundaryPtr>* road_boundaries,
                        std::vector<JunctionBoundaryPtr>* junctions) const;
  /**
   * @brief get forward nearest signals within certain range on the lane
   *        if there are two signals related to one stop line,
   *        return both signals.
   * @param point the target position
   * @param distance the forward search distance
   * @param signals all signals match conditions
   * @return 0:success, otherwise failed
   */
  int GetForwardNearestSignalsOnLane(
      const acu::common::PointENU& point, const double distance,
      std::vector<SignalInfoConstPtr>* signals) const;

  /**
   * @brief get all other stop signs associated with a stop sign
   *        in the same junction
   * @param id id of stop sign
   * @param stop_signs stop signs associated
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedStopSigns(
      const Id& id, std::vector<StopSignInfoConstPtr>* stop_signs) const;

  /**
   * @brief get all lanes associated with a stop sign in the same junction
   * @param id id of stop sign
   * @param lanes all lanes match conditions
   * @return 0:success, otherwise failed
   */
  int GetStopSignAssociatedLanes(const Id& id,
                                 std::vector<LaneInfoConstPtr>* lanes) const;

 private:
  int GetLanes(const acu::common::math::Vec2d& point, double distance,
               std::vector<LaneInfoConstPtr>* lanes) const;
  int GetJunctions(const acu::common::math::Vec2d& point, double distance,
                   std::vector<JunctionInfoConstPtr>* junctions) const;
  int GetCrosswalks(const acu::common::math::Vec2d& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  int GetSignals(const acu::common::math::Vec2d& point, double distance,
                 std::vector<SignalInfoConstPtr>* signals) const;
  int GetStopSigns(const acu::common::math::Vec2d& point, double distance,
                   std::vector<StopSignInfoConstPtr>* stop_signs) const;
  int GetYieldSigns(const acu::common::math::Vec2d& point, double distance,
                    std::vector<YieldSignInfoConstPtr>* yield_signs) const;
  int GetClearAreas(const acu::common::math::Vec2d& point, double distance,
                    std::vector<ClearAreaInfoConstPtr>* clear_areas) const;
  int GetSpeedBumps(const acu::common::math::Vec2d& point, double distance,
                    std::vector<SpeedBumpInfoConstPtr>* speed_bumps) const;
  int GetIsolationBelts(const acu::common::math::Vec2d& point, double distance,
                    std::vector<IsolationBeltInfoConstPtr>* isolationbelts) const;
  int GetGuardrails(const acu::common::math::Vec2d& point, double distance,
                    std::vector<GuardrailInfoConstPtr>* guardrails) const;
  int GetInners(const acu::common::math::Vec2d& point, double distance,
                    std::vector<InnerInfoConstPtr>* inners) const;
  int GetOuters(const acu::common::math::Vec2d& point, double distance,
                    std::vector<OuterInfoConstPtr>* outers) const;
  int GetNearestLane(const acu::common::math::Vec2d& point,
                     LaneInfoConstPtr* nearest_lane, double* nearest_s,
                     double* nearest_l) const;
  int GetNearestLaneWithHeading(const acu::common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  int GetNearestCruiseLaneWithHeading(const acu::common::math::Vec2d& point,
                                const double distance,
                                const double central_heading,
                                const double max_heading_difference,
                                LaneInfoConstPtr* nearest_lane,
                                double* nearest_s, double* nearest_l) const;
  int GetLanesWithHeading(const acu::common::math::Vec2d& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes,
                          std::vector<double>& distances) const;
  int GetLanesWithHeading(const acu::common::math::Vec2d& point,
                          const double distance, const double central_heading,
                          const double max_heading_difference,
                          std::vector<LaneInfoConstPtr>* lanes) const;
  int GetRoads(const acu::common::math::Vec2d& point, double distance,
               std::vector<RoadInfoConstPtr>* roads) const;

  template <class Table, class BoxTable, class KDTree>
  static void BuildSegmentKDTree(
      const Table& table, const acu::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  template <class Table, class BoxTable, class KDTree>
  static void BuildPolygonKDTree(
      const Table& table, const acu::common::math::AABoxKDTreeParams& params,
      BoxTable* const box_table, std::unique_ptr<KDTree>* const kdtree);

  void BuildLaneSegmentKDTree();
  void BuildJunctionPolygonKDTree();
  void BuildCrosswalkPolygonKDTree();
  void BuildSignalSegmentKDTree();
  void BuildStopSignSegmentKDTree();
  void BuildYieldSignSegmentKDTree();
  void BuildClearAreaPolygonKDTree();
  void BuildSpeedBumpPolygonKDTree();
  void BuildIsolationBeltPolygonKDTree();
  void BuildGuardrailPolygonKDTree();
  void BuildInnerPolygonKDTree();
  void BuildOuterPolygonKDTree();

  template <class KDTree>
  static int SearchObjects(const acu::common::math::Vec2d& center,
                           const double radius, const KDTree& kdtree,
                           std::vector<std::string>* const results);

  void Clear();

 private:
  Map map_;
  LaneTable lane_table_;
  JunctionTable junction_table_;
  CrosswalkTable crosswalk_table_;
  SignalTable signal_table_;
  StopSignTable stop_sign_table_;
  YieldSignTable yield_sign_table_;
  ClearAreaTable clear_area_table_;
  SpeedBumpTable speed_bump_table_;
  OverlapTable overlap_table_;
  RoadTable road_table_;
  IsolationBeltTable isolationbelt_table_;
  GuardrailTable guardrail_table_;
  InnerTable inner_table_;
  OuterTable outer_table_;

  std::vector<LaneSegmentBox> lane_segment_boxes_;
  std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

  std::vector<JunctionPolygonBox> junction_polygon_boxes_;
  std::unique_ptr<JunctionPolygonKDTree> junction_polygon_kdtree_;

  std::vector<CrosswalkPolygonBox> crosswalk_polygon_boxes_;
  std::unique_ptr<CrosswalkPolygonKDTree> crosswalk_polygon_kdtree_;

  std::vector<SignalSegmentBox> signal_segment_boxes_;
  std::unique_ptr<SignalSegmentKDTree> signal_segment_kdtree_;

  std::vector<StopSignSegmentBox> stop_sign_segment_boxes_;
  std::unique_ptr<StopSignSegmentKDTree> stop_sign_segment_kdtree_;

  std::vector<YieldSignSegmentBox> yield_sign_segment_boxes_;
  std::unique_ptr<YieldSignSegmentKDTree> yield_sign_segment_kdtree_;

  std::vector<ClearAreaPolygonBox> clear_area_polygon_boxes_;
  std::unique_ptr<ClearAreaPolygonKDTree> clear_area_polygon_kdtree_;

  std::vector<SpeedBumpPolygonBox> speed_bump_polygon_boxes_;
  std::unique_ptr<SpeedBumpPolygonKDTree> speed_bump_polygon_kdtree_;

  std::vector<IsolationBeltPolygonBox> isolationbelt_polygon_boxes_;
  std::unique_ptr<IsolationBeltPolygonKDTree> isolationbelt_polygon_kdtree_;

  std::vector<GuardrailPolygonBox> guardrail_polygon_boxes_;
  std::unique_ptr<GuardrailPolygonKDTree> guardrail_polygon_kdtree_;

  std::vector<InnerPolygonBox> inner_polygon_boxes_;
  std::unique_ptr<InnerPolygonKDTree> inner_polygon_kdtree_;

  std::vector<OuterPolygonBox> outer_polygon_boxes_;
  std::unique_ptr<OuterPolygonKDTree> outer_polygon_kdtree_;
};

}  // namespace hdmap
}  // namespace acu

#endif  // MODULES_MAP_HDMAP_HDMAP_IMPL_H_
