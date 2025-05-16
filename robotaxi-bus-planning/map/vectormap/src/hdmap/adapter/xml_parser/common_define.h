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
#ifndef MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
#define MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_

#include <string>
#include <unordered_set>
#include <vector>
#include "map/vectormap/src/vectormap/alog.h"
#include "map.pb.h"

namespace acu {
namespace hdmap {
namespace adapter {

using PbHeader = acu::hdmap::Header;
using PbRoad = acu::hdmap::Road;
using PbRoadSection = acu::hdmap::RoadSection;
using PbLane = acu::hdmap::Lane;
using PbJunction = acu::hdmap::Junction;
using PbIsolationbelt = acu::hdmap::Isolationbelt;
using PbGuardrail = acu::hdmap::Guardrail;
using PbInner = acu::hdmap::Inner;
using PbOuter = acu::hdmap::Outer;
using PbSignal = acu::hdmap::Signal;
using PbSubSignal = acu::hdmap::Subsignal;
using PbCrosswalk = acu::hdmap::Crosswalk;
using PbSpeedBump = acu::hdmap::SpeedBump;
using PbStopSign = acu::hdmap::StopSign;
using PbYieldSign = acu::hdmap::YieldSign;
using PbObjectOverlapInfo = acu::hdmap::ObjectOverlapInfo;
using PbOverlap = acu::hdmap::Overlap;
using PbClearArea = acu::hdmap::ClearArea;
using PbLineSegment = acu::hdmap::LineSegment;
using PbCurveSegment = acu::hdmap::CurveSegment;
using PbCurve = acu::hdmap::Curve;
using PbPoint3D = acu::common::PointENU;
using PbLaneType = acu::hdmap::Lane_LaneType;
using PbTurnType = acu::hdmap::Lane_LaneTurn;
using PbID = acu::hdmap::Id;
using PbLaneBoundary = acu::hdmap::LaneBoundary;
using PbLaneBoundaryTypeType = acu::hdmap::LaneBoundaryType_Type;
using PbPolygon = acu::hdmap::Polygon;
using PbBoundaryPolygon = acu::hdmap::BoundaryPolygon;
using PbBoundaryEdge = acu::hdmap::BoundaryEdge;

using PbLaneDirection = acu::hdmap::Lane_LaneDirection;
using PbSignalType = acu::hdmap::Signal_Type;
using PbSubSignalType = acu::hdmap::Subsignal_Type;
using PbBoundaryEdgeType = acu::hdmap::BoundaryEdge_Type;

struct StopLineInternal {
  std::string id;
  PbCurve curve;
};

struct StopSignInternal {
  std::string id;
  PbStopSign stop_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct YieldSignInternal {
  std::string id;
  PbYieldSign yield_sign;
  std::unordered_set<std::string> stop_line_ids;
};

struct TrafficLightInternal {
  std::string id;
  PbSignal traffic_light;
  std::unordered_set<std::string> stop_line_ids;
};

struct OverlapWithLane {
  std::string object_id;
  double start_s;
  double end_s;
  bool is_merge;

  OverlapWithLane() : is_merge(false) {}
};

struct OverlapWithJunction {
  std::string object_id;
};

struct LaneInternal {
  PbLane lane;
  std::vector<OverlapWithLane> overlap_signals;
  std::vector<OverlapWithLane> overlap_objects;
  std::vector<OverlapWithLane> overlap_junctions;
  std::vector<OverlapWithLane> overlap_lanes;
};

struct JunctionInternal {
  PbJunction junction;
  std::unordered_set<std::string> road_ids;
  std::vector<OverlapWithJunction> overlap_with_junctions;
};

struct RoadSectionInternal {
  std::string id;
  PbRoadSection section;
  std::vector<LaneInternal> lanes;
};

struct RoadInternal {
  std::string id;
  PbRoad road;

  bool in_junction;
  std::string junction_id;

  std::vector<RoadSectionInternal> sections;

  std::vector<TrafficLightInternal> traffic_lights;
  std::vector<StopSignInternal> stop_signs;
  std::vector<YieldSignInternal> yield_signs;
  std::vector<PbCrosswalk> crosswalks;
  std::vector<PbClearArea> clear_areas;
  std::vector<PbSpeedBump> speed_bumps;
  std::vector<StopLineInternal> stop_lines;

  RoadInternal() : in_junction(false) { junction_id = ""; }
};

struct RoadSurfaceInternal {
  std::string id;
  std::vector<PbIsolationbelt> isolationbelts;  //隔离带
  std::vector<PbGuardrail> guardrails;          //栏杆
  std::vector<PbInner> inners;                  //道路内边界（花坛）
  std::vector<PbOuter> outers;                  //道路外边界（马路牙子）
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu

#endif  // MODULES_MAP_HDMAP_ADAPTER_XML_PARSER_COMMON_DEFINE_H_
