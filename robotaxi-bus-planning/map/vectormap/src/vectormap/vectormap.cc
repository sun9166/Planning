/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: api
*
* Description: fool ACU vectormap API like USB

*
* History:
* lbh         2018/05/15    1.0.0    build this module.
******************************************************************************/
#include "map/vectormap/include/vectormap.h"
#include <iostream>
#include "map/vectormap/src/vectormap/alog.h"
#include "common/util/util.h"

namespace acu {
namespace vectormap {

constexpr double kSampleDistance = 0.25;
constexpr double kRadToDeg = 57.29577950560105;
/*
std::shared_ptr<VectorMap> VectorMap::vectormap_ptr_(new VectorMap());
std::shared_ptr<VectorMap> VectorMap::Initance() { return vectormap_ptr_; }
*/
int VectorMap::LoadMapFromFile(const std::string &file) {
  return impl_.LoadMapFromFile(file);
}

int VectorMap::LoadMapFromProto(const Map &map_proto) {
  std::cout<<"Loading VectorMap with header: " << map_proto.header().ShortDebugString()<<std::endl;
  return impl_.LoadMapFromProto(map_proto);
}

int VectorMap::LoadRoadnetFromFile(const std::string &file) {
  return roadnet_impl_.LoadRoadnetFromFile(file);
}

int VectorMap::LoadRoadnetFromProto(const Roadnet &roadnet_proto) {
  printf("Loading Roadnet proto ... ");
  return roadnet_impl_.LoadRoadnetFromProto(roadnet_proto);
}

const RoadNodeTable *VectorMap::GetRoadNet() const {
  return roadnet_impl_.GetRoadNet();
}

LaneInfoConstPtr VectorMap::GetLaneById(const Id &id) const {
  return impl_.GetLaneById(id);
}

LaneInfoConstPtr VectorMap::GetLaneById(const std::string &id) const {
  Id lane_id;
  lane_id.set_id(id);
  return impl_.GetLaneById(lane_id);
}

RoadInfoConstPtr VectorMap::GetRoadById(const Id &id) const {
  return impl_.GetRoadById(id);
}

RoadNodeInfoConstPtr VectorMap::GetRoadNodeById(const Id &id) const {
  return roadnet_impl_.GetRoadNodeById(id);
}

SignalInfoConstPtr VectorMap::GetSignalById(const Id& id) const {
  return impl_.GetSignalById(id);
}

int VectorMap::GetRoads(const acu::common::PointENU &point, double distance,
                        std::vector<RoadInfoConstPtr> &roads) const {
  return impl_.GetRoads(point, distance, &roads);
}

int VectorMap::GetRoads(const acu::common::PointENU &point, double distance,
                        std::vector<Id> &ids) const {
  std::vector<RoadInfoConstPtr> roads;
  if (GetRoads(point, distance, roads) != 0) return -1;
  for (auto &road : roads) {
    ids.push_back(road->id());
  }
  return 0;
}

int VectorMap::GetNearestRoad(const common::PointENU &point,
                              Id &nearest_road_id) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1) {
    printf("Can't find nearest lane!\n");
    return -1;
  }

  nearest_road_id = nearest_lane->road_id();
  return 0;
}

int VectorMap::GetNearestRoadWidthHeading(const common::PointENU& point, const double distance, 
                                         const double central_heading,
                                         const double max_heading_difference, 
                                         Id &nearest_road_id,
                                         double& nearest_s, double& nearest_l) const {
  LaneInfoConstPtr nearest_lane;
  if (GetNearestLaneWithHeading(point, distance, central_heading, max_heading_difference,
    nearest_lane, nearest_s, nearest_l) == -1) {
    printf("Can't find nearest lane with heading!\n");
    return -1;
  }

  nearest_road_id = nearest_lane->road_id();
  return 0;
}

int VectorMap::GetLanesWithHeading(const common::PointENU& point, const double distance,
                                    const double central_heading,
                                    const double max_heading_difference,
                                    std::vector<std::pair<LaneInfoConstPtr, double>>& output_lanes) const {
  std::vector<LaneInfoConstPtr> lanes;
  std::vector<double> distances;
  if (impl_.GetLanesWithHeading(point, distance, central_heading,
                          max_heading_difference, &lanes, distances) < 0 ||
      lanes.size() != distances.size()) {
    return -1;
  }
  output_lanes.clear();
  for (int i = 0; i < lanes.size(); i++) {
    std::pair<LaneInfoConstPtr, double> temp(lanes.at(i), distances.at(i));
    output_lanes.push_back(temp);
  }
  return 0;
}

int VectorMap::GetNearestRoadWithNoTurn(const common::PointENU& point, Id &road_Id, double &distance_to_end, bool &is_waiting_left) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1) {
    printf("Can't find nearest lane!\n");
    return -1;
  }

  // std::cout << "is_waiting_left: " << is_waiting_left << std::endl;
  if (!is_waiting_left 
        && nearest_lane->lane().has_turn() 
        && nearest_lane->lane().has_type()
        && 
        ((nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN && nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT)
         || (nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN && nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING)
         || (nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_RIGHT_TURN && nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING)
         || (nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN && nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_BUSING))
       ) {
     if(nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN && nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT){
       is_waiting_left = true;
     }else{
       is_waiting_left = false;
     }
     road_Id = nearest_lane->road_id();
     distance_to_end = nearest_lane->total_length() - nearest_s;  
     return 0;
  }

  LaneInfoConstPtr nearest_lane_waiting_left = nullptr;
  if(nearest_lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN && 
     nearest_lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT){
       nearest_lane_waiting_left = nearest_lane;
  }

  std::vector<LaneInfoConstPtr> lanes;
  double distance = 10.0;
  if (impl_.GetLanes(point, distance, &lanes) < 0) {
    return -1;
  }
  double min_dis = 5.0;
  distance_to_end = 100.0;
  for (auto lane : lanes) {
    if (lane->lane().has_turn() 
        && lane->lane().has_type()
        && 
        ((lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN && lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT)
         || (lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN && lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING)
         || (lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_NO_TURN && lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_BUSING))
       ) {
      if(lane->lane().turn() == acu::hdmap::Lane_LaneTurn::Lane_LaneTurn_LEFT_TURN && lane->lane().type() == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT){
        is_waiting_left = true;
      }else{
        is_waiting_left = false;
      }
      acu::common::math::Vec2d point1;
      point1.set_x(point.x());
      point1.set_y(point.y());
      double s = 0.0;
      double l = 0.0;
      if (lane->GetProjection(point1, &s, &l) && ((is_waiting_left && min_dis < 1.0) || fabs(l) < min_dis)) {
        // min_dis = fabs(l);
        auto distance = lane->total_length() - s;
        if(fabs(distance) < distance_to_end){
          road_Id = lane->road_id();
          distance_to_end = distance;
        }
        // std::cout << "road id: " << road_Id.id() << "; lane id: " << lane->lane().id().id() << "; lane length: " << lane->total_length() << "; s: " << s << "; l: " << l << std::endl;
        if(is_waiting_left && nearest_lane_waiting_left!=nullptr && nearest_lane_waiting_left->road_id().id()==lane->road_id().id()) {
          road_Id = lane->road_id();
          distance_to_end = distance;
          return 0;
        }
      }
    }
  }
  return 0;
}

int VectorMap::GetNearestLane(const common::PointENU &point,
                              LaneInfoConstPtr &nearest_lane, double &nearest_s,
                              double &nearest_l) const {
  return impl_.GetNearestLane(point, &nearest_lane, &nearest_s, &nearest_l);
}

int VectorMap::GetNearestLaneWithHeading(const common::PointENU& point, const double distance, 
                                         const double central_heading,
                                         const double max_heading_difference, 
                                         LaneInfoConstPtr& nearest_lane,
                                         double& nearest_s, double& nearest_l) const {
  return impl_.GetNearestLaneWithHeading(point, distance, central_heading,
                                         max_heading_difference, &nearest_lane, 
                                         &nearest_s, &nearest_l);
}

int VectorMap::GetNearestCruiseLaneWithHeading(const acu::common::PointENU& point, const double distance, 
                              const double central_heading,
                              const double max_heading_difference, 
                              LaneInfoConstPtr& nearest_lane,
                              double& nearest_s, double& nearest_l) const {
  return impl_.GetNearestCruiseLaneWithHeading(point, distance, central_heading,
                                         max_heading_difference, &nearest_lane, 
                                         &nearest_s, &nearest_l);                              
}

int VectorMap::GetRoadLanes(const Id &id, roadLanes &lanes) const {
  RoadInfoConstPtr road = GetRoadById(id);
  if (road == nullptr) return -1;
  Id key;
  std::vector<Id> value;
  for (auto &section : road->road().section()) {
    key = section.id();
    for (auto &lane_id : section.lane_id()) {
      if (value.empty()) {
        value.push_back(lane_id);
      }
      else {
        int index = lane_id.id().back() - '1';
        bool has_insert = false;
        for (int i = 0; i < value.size(); i++) {
          if (index < value.at(i).id().back() - '1') {
            value.insert(value.begin() + i, lane_id);
            has_insert = true;
            break;
          }
        }
        if (!has_insert) {
          value.push_back(lane_id);
        }
      }
    }
    lanes.push_back(std::make_pair(key, value));
  }
  return 0;
}

int VectorMap::GetRoadLanes(const Id &id, std::vector<std::string> &lanes, bool check_width) const {
  lanes.clear();
  RoadInfoConstPtr road = GetRoadById(id);
  if (road == nullptr) return -1;
  for (auto &section : road->road().section()) {
    for (auto &lane_id : section.lane_id()) {
      double width;
      if (check_width && GetLaneWidth(lane_id.id(), width) && width < 0.5) {
        continue;
      }
      acu::hdmap::Lane::LaneType lane_type;
      if (GetLaneType(lane_id, lane_type) < 0 || 
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BIKING ||  
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_SIDEWALK ||
        (car_type_ == "taxi" && lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BUSING)) {
        continue;
      }

      if (lanes.empty()) {
        lanes.push_back(lane_id.id());
      }
      else {
        int index = lane_id.id().back() - '1';
        bool has_insert = false;
        for (int i = 0; i < lanes.size(); i++) {
          if (index < lanes.at(i).back() - '1') {
            lanes.insert(lanes.begin() + i, lane_id.id());
            has_insert = true;
            break;
          }
        }
        if (!has_insert) {
          lanes.push_back(lane_id.id());
        }
      }
    }
  }
  if (lanes.empty()) {
    AERROR<<"road "<<id.id()<<" has no lane.";
    return -1;
  }
  return 0;
}

int VectorMap::GetRoadLanes(const std::string &id, std::vector<std::string> &lanes, bool check_width) const {
  Id Id_lane;
  Id_lane.set_id(id);
  return GetRoadLanes(Id_lane, lanes, check_width);
}

int VectorMap::GetAllRoadLanes(const std::string &id, std::vector<std::pair<std::string, bool>> &lanes) const {
  lanes.clear();
  Id Id_lane;
  Id_lane.set_id(id);
  RoadInfoConstPtr road = GetRoadById(Id_lane);
  if (road == nullptr) return -1;
  for (auto &section : road->road().section()) {
    for (auto &lane_id : section.lane_id()) {
      acu::hdmap::Lane::LaneType lane_type;
      if (GetLaneType(lane_id, lane_type) < 0 || 
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BIKING || 
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_SIDEWALK ||
        (car_type_ == "taxi" && lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BUSING)) {
        lanes.emplace_back(lane_id.id(), false);
        continue;
      }
      lanes.emplace_back(lane_id.id(), true);
    }
  }
  if (lanes.empty()) {
    AERROR<<"road "<<id<<" has no lane.";
    return -1;
  }
  return 0;
}

int VectorMap::GetRoadPitch(const Id &id, double &pitch) const {
  RoadInfoConstPtr road_ptr = GetRoadById(id);
  if (road_ptr == nullptr) return -1;
  if (road_ptr->road().has_pitch()) {
    pitch = road_ptr->road().pitch();
    return 0;
  } else {
    return -1;
  }
}

int VectorMap::GetRoadSuccessorIDs(const Id &id,
                                   std::vector<Id> &ids_succ) const {
  RoadNodeInfoConstPtr roadnode = GetRoadNodeById(id);
  if (roadnode == nullptr) return -1;
  ids_succ = roadnode->GetSuccessorIds();
  return 0;
}

int VectorMap::GetRoadPredecessorIDs(const Id &id,
                                     std::vector<Id> &ids_pred) const {
  RoadNodeInfoConstPtr roadnode = GetRoadNodeById(id);
  if (roadnode == nullptr) return -1;
  ids_pred = roadnode->GetPredecessorIds();
  return 0;
}

int VectorMap::GetRoad(const Id &lane_id, Id &road_id) const {
  LaneInfoConstPtr lane = GetLaneById(lane_id);
  if (lane == nullptr) return -1;
  road_id = lane->road_id();
  return 0;
}

int VectorMap::GetRoad(const Id &lane_id, std::string &road_id) const {
  LaneInfoConstPtr lane = GetLaneById(lane_id);
  if (lane == nullptr) return -1;
  road_id = lane->road_id().id();
  return 0;
}

int VectorMap::GetRoad(const std::string &lane_id, std::string &road_id) const {
  Id Id_lane;
  Id_lane.set_id(lane_id);
  LaneInfoConstPtr lane = GetLaneById(lane_id);
  if (lane == nullptr) return -1;
  road_id = lane->road_id().id();
  return 0;
}

int VectorMap::GetLanes(const acu::common::PointENU &point, double distance,
                        std::vector<LaneInfoConstPtr> &lanes) const {
  return impl_.GetLanes(point, distance, &lanes);
}

int VectorMap::GetSignals(const acu::common::PointENU &point, double distance,
                          std::vector<SignalInfoConstPtr> &signals) const {
  return impl_.GetSignals(point, distance, &signals);
}

int VectorMap::GetLaneCenterLine(const Id &id,
                                 std::vector<PointXYA> &points) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  points.clear();
  if (lane->points().size() != lane->headings().size()) return -1;
  int i = 0;
  for (auto &point : lane->points()) {
    points.emplace_back(point.x(), point.y(), lane->headings()[i] * kRadToDeg);
    i++;
  }
  return 0;
}

LaneBoundaryType::Type VectorMap::LeftBoundaryType(
  const LaneWaypoint &waypoint) const {
  if (!waypoint.lane) {
    return LaneBoundaryType::UNKNOWN;
  }
  for (const auto &type :
       waypoint.lane->lane().left_boundary().boundary_type()) {
    if (type.s() <= waypoint.s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type VectorMap::RightBoundaryType(
  const LaneWaypoint &waypoint) const {
  if (!waypoint.lane) {
    return LaneBoundaryType::UNKNOWN;
  }
  for (const auto &type :
       waypoint.lane->lane().right_boundary().boundary_type()) {
    if (type.s() <= waypoint.s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type VectorMap::LeftBoundaryType(
  const common::PointENU &point) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return LaneBoundaryType::UNKNOWN;
  for (const auto &type :
       nearest_lane->lane().left_boundary().boundary_type()) {
    if (type.s() <= nearest_s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type VectorMap::RightBoundaryType(
  const common::PointENU &point) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return LaneBoundaryType::UNKNOWN;
  for (const auto &type :
       nearest_lane->lane().right_boundary().boundary_type()) {
    if (type.s() <= nearest_s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type VectorMap::LeftBoundaryType(
  const LaneInfoConstPtr lane) const {
  double nearest_s;
  double nearest_l;
  for (const auto &type :
       lane->lane().left_boundary().boundary_type()) {
    if (type.s() <= nearest_s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneBoundaryType::Type VectorMap::RightBoundaryType(
  const LaneInfoConstPtr lane) const {
  double nearest_s;
  double nearest_l;
  for (const auto &type :
       lane->lane().right_boundary().boundary_type()) {
    if (type.s() <= nearest_s) {
      if (type.types_size() > 0) {
        return type.types(0);
      } else {
        return LaneBoundaryType::UNKNOWN;
      }
    }
  }
  return LaneBoundaryType::UNKNOWN;
}

LaneWaypoint VectorMap::LeftNeighborWaypoint(
  const LaneWaypoint &waypoint) const {
  LaneWaypoint neighbor;
  if (!waypoint.lane) {
    return neighbor;
  }
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  for (const auto &lane_id :
       waypoint.lane->lane().left_neighbor_forward_lane_id()) {
    auto lane = GetLaneById(lane_id);
    if (!lane) {
      return neighbor;
    }
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }

    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;
    } else {
      return LaneWaypoint(lane, s, l);
    }
  }
  return neighbor;
}

LaneWaypoint VectorMap::RightNeighborWaypoint(
  const LaneWaypoint &waypoint) const {
  LaneWaypoint neighbor;
  if (!waypoint.lane) {
    return neighbor;
  }
  auto point = waypoint.lane->GetSmoothPoint(waypoint.s);
  for (const auto &lane_id :
       waypoint.lane->lane().right_neighbor_forward_lane_id()) {
    auto lane = GetLaneById(lane_id);
    if (!lane) {
      return neighbor;
    }
    double s = 0.0;
    double l = 0.0;
    if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
      continue;
    }
    if (s < -kSampleDistance || s > lane->total_length() + kSampleDistance) {
      continue;
    } else {
      return LaneWaypoint(lane, s, l);
    }
  }
  return neighbor;
}

int VectorMap::DistanceToLaneEnd(const common::PointENU &point,
                                 double &distance) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return -1;
  distance = nearest_lane->total_length() - nearest_s;
  return 0;
}

int VectorMap::GetLeftRightLaneIDs(const Id &id, Id &id_left,
                                   Id &id_right) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  int ret = 0;
  for (const auto &lane_id : lane->lane().left_neighbor_forward_lane_id()) {
    id_left = lane_id;
    ret += 1;
  }
  for (const auto &lane_id : lane->lane().right_neighbor_forward_lane_id()) {
    id_right = lane_id;
    ret += 2;
  }
  return ret;
}

int VectorMap::GetOppositeLaneIDs(const Id &id, std::vector<Id> &ids_opposite) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  int ret = 0;
  for (const auto &lane_id : lane->lane().left_neighbor_reverse_lane_id()) {
    ids_opposite.push_back(lane_id);
    ret += 1;
  }
  return ret;
}

int VectorMap::GetLaneSuccessorIDs(const Id &id,
                                   std::vector<Id> &ids_succ) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  int ret = 0;
  ids_succ.clear();
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (ids_succ.empty()) {
      ids_succ.push_back(lane_id);
    }
    else {
      int index = lane_id.id().back() - '1';
      bool has_insert = false;
      for (int i = 0; i < ids_succ.size(); i++) {
        if (index < ids_succ.at(i).id().back() - '1') {
          ids_succ.insert(ids_succ.begin() + i, lane_id);
          has_insert = true;
          break;
        }
      }
      if (!has_insert) {
        ids_succ.push_back(lane_id);
      }
    }
    ret++;
  }
  return ret;
}

int VectorMap::GetLaneSuccessorIDs(const std::string &id, 
                                   std::vector<Id> &ids_succ) const {
  Id Id_lane;
  Id_lane.set_id(id);
  ids_succ.clear();
  LaneInfoConstPtr lane = GetLaneById(Id_lane);
  if (lane == nullptr) return -1;
  int ret = 0;
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (ids_succ.empty()) {
      ids_succ.push_back(lane_id);
    }
    else {
      int index = lane_id.id().back() - '1';
      bool has_insert = false;
      for (int i = 0; i < ids_succ.size(); i++) {
        if (index < ids_succ.at(i).id().back() - '1') {
          ids_succ.insert(ids_succ.begin() + i, lane_id);
          has_insert = true;
          break;
        }
      }
      if (!has_insert) {
        ids_succ.push_back(lane_id);
      }
    }
    ret++;
  }
  return ret;
}

int VectorMap::GetSuccessorLanesInXRoad(const std::string &id, 
                                        const std::string &x_road, 
                                        std::vector<std::string> &ids_succ,
                                        bool able_driving) const {
  ids_succ.clear();
  std::vector<Id> Ids_next;
  if (GetLaneSuccessorIDs(id, Ids_next) <= 0) {
    return 0;
  }
  for (auto &Id_id : Ids_next) {
    acu::hdmap::Lane::LaneType lane_type;
    if (able_driving && (GetLaneType(Id_id, lane_type) < 0 || 
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BIKING || 
        lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_SIDEWALK ||
        (car_type_ == "taxi" && lane_type == acu::hdmap::Lane_LaneType::Lane_LaneType_BUSING))) {
      continue;
    }
    std::string lane_to_road;
    if (GetRoad(Id_id, lane_to_road) == 0 && (lane_to_road == x_road)) {
      ids_succ.push_back(Id_id.id());
    }
  }
  return (int)ids_succ.size();
}

int VectorMap::GetLaneSpeedLimit(const Id &id, double &speed) const {
  speed = 0.0;
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  if (lane->lane().has_speed_limit() && lane->lane().speed_limit() > 0.1) {
    speed = lane->lane().speed_limit();
    return 0;
  } 
  return -1;
}

int VectorMap::GetLaneTurn(const Id &id,
                           acu::hdmap::Lane::LaneTurn &type) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  if (lane->lane().has_turn()) {
    type = lane->lane().turn();
    return 0;
  } else {
    return -1;
  }
}

int VectorMap::GetRoadTurn(const Id &id, int &min_type, std::vector<std::string> &combine_lanes) const {
  combine_lanes.clear();
  std::vector<std::string> road_lanes;
  if (GetRoadLanes(id, road_lanes) < 0) {
    AERROR<<"GetRoadLanes failed.";
    return -1;
  }
  std::vector<int> turns;
  min_type = 10;
  for (auto &lane : road_lanes) {
    Id Id_lane;
    Id_lane.set_id(lane);
    acu::hdmap::Lane::LaneTurn type;
    if (GetLaneTurn(Id_lane, type) == 0) {
      turns.push_back(type);
      if (min_type > type) {
        min_type = type;
      }
    }
    else {
      AERROR<<"Lane "<<lane<<" has no turn type.";
    }
  }
  if (turns.size() != road_lanes.size()) {
    return -1;
  }
  for (int i = 0; i < turns.size(); i++) {
    if (turns.at(i) > min_type) {
      combine_lanes.push_back(road_lanes.at(i));
    }
  }
  return 0;
}

int VectorMap::GetLaneType(const Id &id,
                           acu::hdmap::Lane::LaneType &type) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  if (lane->lane().has_type()) {
    type = lane->lane().type();
    return 0;
  } else {
    return -1;
  }
}

int VectorMap::GetLaneDirection(const Id &id, 
        acu::hdmap::Lane::LaneDirection &direction) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  if (lane->lane().has_direction()) {
    direction = lane->lane().direction();
    return 0;
  } else {
    return -1;
  }
}

int VectorMap::GetLanePredecessorIDs(const Id &id,
                                     std::vector<Id> &ids_prede) const {
  LaneInfoConstPtr lane = GetLaneById(id);
  if (lane == nullptr) return -1;
  int ret = 0;
  ids_prede.clear();
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    ids_prede.push_back(lane_id);
    ret++;
  }
  return ret;
}

int VectorMap::GetLanePredecessorIDs(const std::string &id, 
                                     std::vector<Id> &ids_prede) const {
  Id Id_lane;
  Id_lane.set_id(id);
  ids_prede.clear();
  LaneInfoConstPtr lane = GetLaneById(Id_lane);
  if (lane == nullptr) return -1;
  int ret = 0;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    ids_prede.push_back(lane_id);
    ret++;
  }
  return ret;
}

int VectorMap::GetPredecessorLanesInXRoad(const std::string &id, std::string &x_road, std::vector<Id> &ids_prede) const {
  ids_prede.clear();
  std::vector<Id> Ids_pre;
  if (GetLanePredecessorIDs(id, Ids_pre) <= 0) {
    return 0;
  }
  for (auto &Id_id : Ids_pre) {
    std::string lane_to_road;
    
    int flag = GetRoad(Id_id, lane_to_road);
    if (flag == 0 && (lane_to_road == x_road)) {
      ids_prede.push_back(Id_id);
    }
  }
  return ids_prede.size();
}

int VectorMap::GetWidthToRoadBoundary(const common::PointENU &point,
                                  double &left_width,
                                  double &right_width) const {
  LaneInfoConstPtr nearest_lane;
  RoadInfoConstPtr road_ptr;
  double nearest_s;
  double nearest_l;

  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return -1;

  road_ptr = GetRoadById(nearest_lane->road_id());
  if (road_ptr->junction_id().id() != "") { // dont get width in junction
    return -2;
	}

  nearest_lane->GetRoadWidth(nearest_s, &left_width, &right_width);
  left_width -= nearest_l;
  right_width += nearest_l;
  return 0;
}

int VectorMap::GetWidthToBoundary(const common::PointENU &point,
                                  double &left_width,
                                  double &right_width) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return -1;
  nearest_lane->GetWidth(nearest_s, &left_width, &right_width);
  left_width -= nearest_l;
  right_width += nearest_l;
  return 0;
}

int VectorMap::GetLaneWidth(const common::PointENU &point, double &width) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return -1;
  width = nearest_lane->GetWidth(nearest_s);
  return 0;
}

int VectorMap::GetLaneWidth(const std::string &id, double &width, double s) const {
  Id Id_lane;
  Id_lane.set_id(id);
  LaneInfoConstPtr lane = GetLaneById(Id_lane);
  if (lane == nullptr) return -1;
  width = lane->GetWidth(s);
}

int VectorMap::GetWidthToNeighbor(const common::PointENU &point,
                                  double &left_width,
                                  double &right_width) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return -1;
  int ret = 0;
  LaneWaypoint waypoint(nearest_lane, nearest_s, nearest_l);
  LaneWaypoint lwp = LeftNeighborWaypoint(waypoint);
  if (lwp.lane != nullptr) {
    ret += 1;
    left_width = fabs(lwp.l);
  }
  LaneWaypoint rwp = RightNeighborWaypoint(waypoint);
  if (rwp.lane != nullptr) {
    ret += 2;
    right_width = fabs(lwp.l);
  }
  return ret;
}

bool VectorMap::IsOnLane(const common::PointENU &point, Id &lane_id) const {
  LaneInfoConstPtr nearest_lane;
  double nearest_s;
  double nearest_l;
  if (GetNearestLane(point, nearest_lane, nearest_s, nearest_l) == -1)
    return false;
  lane_id.set_id(nearest_lane->id().id());
  acu::common::math::Vec2d p(point.x(), point.y());
  return nearest_lane->IsOnLane(p);
}

bool VectorMap::IsOnLane(acu::common::math::Vec2d p, Id &lane_id) const {
  LaneInfoConstPtr lane_ptr;
  lane_ptr = GetLaneById(lane_id);
  if (lane_ptr == nullptr) return false;
  return lane_ptr->IsOnLane(p);
}

bool VectorMap::IsParkRoad(const std::string &road) const {
  std::vector<std::string> lanes_in_road;
  if (GetRoadLanes(road, lanes_in_road) < 0) {
    return false;
  }
  Id lane_Id;
  acu::hdmap::Lane::LaneType lanetype;
  lane_Id.set_id(lanes_in_road.front());
  LaneInfoConstPtr lane_ptr = GetLaneById(lane_Id);
  if (GetLaneType(lane_Id, lanetype) != 0 || 
      lane_ptr == nullptr || !lane_ptr->lane().has_direction()) {
    AERROR<<"lane "<<lanes_in_road.front()<<" lanetype or ptr is wrong.";
    return false;
  }
  if (lanetype == Lane_LaneType::Lane_LaneType_PARKING &&
      lane_ptr->lane().direction() == Lane_LaneDirection::Lane_LaneDirection_BACKWARD) {
    return true;
  }
  return false;
}

int VectorMap::GetLaneOverlap(const std::string &base_lane,
                     std::map<std::string, std::pair<double, double>> &base_projections) const 
{
  base_projections.clear();
  LaneInfoConstPtr base_lane_ptr = GetLaneById(base_lane);
  if (base_lane_ptr == nullptr) {
    return -1;
  }
  const std::vector<OverlapInfoConstPtr> base_crosslanes_ptr = base_lane_ptr->cross_lanes();
  for (auto crosslane_ptr : base_crosslanes_ptr) {
    for (const auto& object : crosslane_ptr->overlap().object()) {
      if (object.id().id() != base_lane &&
          object.lane_overlap_info().end_s() - object.lane_overlap_info().start_s() > 0.1) {
        std::pair<double, double> temp_projection;
        temp_projection.first = object.lane_overlap_info().start_s();
        temp_projection.second = object.lane_overlap_info().end_s();
        base_projections[object.id().id()] = temp_projection;
      }
    } 
  }
  return 0;
}

int VectorMap::GetCrosswalks(const acu::common::PointENU& point, double distance,
                         std::vector<CrosswalkInfoConstPtr>* crosswalks) const {
  return impl_.GetCrosswalks(point, distance, crosswalks);
}

int VectorMap::GetIsolationBelts(const acu::common::PointENU& point, double distance,
                    std::vector<IsolationBeltInfoConstPtr>* isolationbelts) const {
  return impl_.GetIsolationBelts(point, distance, isolationbelts);
}

int VectorMap::GetGuardrails(const acu::common::PointENU& point, double distance,
                    std::vector<GuardrailInfoConstPtr>* guardrails) const {
  return impl_.GetGuardrails(point, distance, guardrails);
}

int VectorMap::GetInners(const acu::common::PointENU& point, double distance,
                    std::vector<InnerInfoConstPtr>* inners) const {
  return impl_.GetInners(point, distance, inners);
}

int VectorMap::GetOuters(const acu::common::PointENU& point, double distance,
                    std::vector<OuterInfoConstPtr>* outers) const {
  return impl_.GetOuters(point, distance, outers);
}

int VectorMap::GetJunctionOppositeLaneIDs(const Id &jun_id, const Id &id, 
                                        std::vector<Id> &ids_opposite) const {
  // if (GetOppositeLaneIDs(id, ids_opposite) > 0) {
  //   return 0;
  // }
  // const auto& junction = impl_.GetJunctionById(jun_id);
  // if (junction == nulptr) {
  //   return -1;
  // }
  // std::vector<Id> ids_extry;
  // std::vector<Id> ids_exit;

  // if (GetExtryExitLane(jun_id, ids_extry, ids_exit) != 0) {
  //   return -1;
  // }


}

int VectorMap::GetExtryExitLane(Id &junction_id, std::vector<Id> &ids_extry,
                      std::vector<Id> &ids_exit) const {
  const auto& junction = impl_.GetJunctionById(junction_id);
  if (junction == nullptr) {
    return -1;
  }  
  std::cout << "junction_id: " << junction->id().id() << std::endl;
  // std::cout << "junction().overlap_id().size(): " << junction->junction().overlap_id().size() << std::endl;
  for (auto overlap_id : junction->junction().overlap_id()) {
    // OverlapIDs
    // std::cout << "overlap_id: " << overlap_id.id() << std::endl;
    OverlapInfoConstPtr overlap_ptr = impl_.GetOverlapById(overlap_id);
    if (overlap_ptr == nullptr){
      // std::cout << "overlap_tr is nullptr for overlap_id: " << overlap_id.id() << std::endl;
      continue;
    }
    // std::cout << overlap_ptr->overlap().object()[0].id().id() << std::endl;
    
    Id lane_id = overlap_ptr->overlap().object()[0].id();
      // std::cout << "over_id: " << over_id.id().id() << std::endl;
    // for (auto over_id : overlap_ptr->overlap().object() ) {
    //   std::cout << "over_id: " << over_id.id().id() << std::endl;
    // }
    
    if (lane_id.id() != junction->id().id()) {
      // std::cout << "lane_id: " << lane_id.id() << std::endl;
      std::vector<Id> tmp_extry;
      GetLanePredecessorIDs(lane_id, tmp_extry);
      ids_extry.insert(ids_extry.end(), tmp_extry.begin(), tmp_extry.end());
      // std::cout << "ids_extry.size(): " << ids_extry.size() << std::endl;
      std::vector<Id> tmp_exit;
      GetLaneSuccessorIDs(lane_id, tmp_exit);
      ids_exit.insert(ids_exit.end(), tmp_exit.begin(), tmp_exit.end());
    }

  }
  return 0;
}           

int VectorMap::GetExitRelation(std::vector<Id> &lane_id1, std::vector<Id> &lane_id2, 
              int &exit_relation) const {
  if (lane_id1.size() == 0 || lane_id2.size() == 0) {
    return 0;
  }  
  
  Id Id_road1;
  Id Id_road2;
  GetRoad(lane_id1.back(), Id_road1);
  GetRoad(lane_id2.back(), Id_road2);
  
  if (Id_road1.id() == Id_road2.id()) {
    exit_relation = 2;//重叠
    return 0;
  }

  LaneInfoConstPtr lane_ptr = nullptr;
  lane_ptr = GetLaneById(lane_id1.back());
  if (lane_ptr == nullptr) {
    // std::cout << "lane_ptr is nullptr." << std::endl;
    exit_relation = 0;//无关
    return 0;
  }
  const std::vector<OverlapInfoConstPtr> cross_lanes_ptr = lane_ptr->cross_lanes();
  for (auto cross_ptr : cross_lanes_ptr) { 
	  // std::cout <<  "overlap().id(): " << cross_ptr->overlap().id().id() << std::endl;
      if (cross_ptr->overlap().object().size() != 2) {
        continue;
      }
      ObjectOverlapInfo object1 = cross_ptr->overlap().object()[0];
      ObjectOverlapInfo object2 = cross_ptr->overlap().object()[1];

      Id overlap_id;
      if (object1.id().id() == lane_id2.back().id() ||
            object2.id().id() == lane_id2.back().id() ) {
        exit_relation = 1; // 交叉
        return 0;
      }
  }
  exit_relation = 0; //无关

  return 0;
}

   /**
   * @brief 
   * @param lane_id1  lane id1
   * @param lane_id2  lane id2
   * @param exit_relation   irrelevant/cross/overlap -> tmp:0,1,2
   * @param relation        merge/cross/reversion/irrelevant    tmp:0,1,2,3
   * @param CollisionIndex1 collision position
   * @param CollisionIndex2 collision position
   * @param lane_pri  lane privilege  id1>id2; id1<id2; id1=id2   tmp: 0,1,2 
   * return -1 : failed : 0 : success. 
   */


int VectorMap::GetLanesRelvant(std::vector<Id> &lane_id1, std::vector<Id> &lane_id2, 
              int &exit_relation,
              int &relation, 
              CollisionIndex &col_index1, CollisionIndex &col_index2,
              int &lanePrivilege) const {
  if (lane_id1.size() == 0 || lane_id2.size() == 0) {
    return 0;
  }
  // exit_relation
  GetExitRelation(lane_id1, lane_id2, exit_relation); //TODO
  

  LaneInfoConstPtr lane_ptr = nullptr;
  acu::hdmap::Lane::LaneTurn lane1_turn =  acu::hdmap::Lane::NO_TURN;
  acu::hdmap::Lane::LaneTurn lane2_turn = acu::hdmap::Lane::NO_TURN;
  for (auto lane1 : lane_id1) {
    if (lane1_turn == acu::hdmap::Lane::NO_TURN) {
      GetLaneTurn(lane1, lane1_turn);
    }
  }
  for (auto lane2 : lane_id2) {
    if (lane2_turn == acu::hdmap::Lane::NO_TURN) {
      GetLaneTurn(lane2, lane2_turn);
    }
  }
  // std::cout << "lane1_turn: " << lane1_turn << 
  //             ", lane2_turn: " << lane2_turn << std::endl;

  int index1 = 0;
  for (auto lane1 : lane_id1) {
    
    // std::cout << "index1: " << index1 << ", lane1: " << lane1.id() << std::endl;
    lane_ptr = GetLaneById(lane1);
    if (lane_ptr == nullptr) {
      // std::cout << "lane_ptr is nullptr." << std::endl;
      continue;
    }
    
    // std::cout << "lane1: " << lane1.id() << std::endl;
    const std::vector<OverlapInfoConstPtr> cross_lanes_ptr = lane_ptr->cross_lanes();
    // std::cout << "cross_lanes_ptr.size(): " << cross_lanes_ptr.size() << std::endl;
    for (auto cross_ptr : cross_lanes_ptr) { 
	  // std::cout <<  "overlap().id(): " << cross_ptr->overlap().id().id() << std::endl;
      if (cross_ptr->overlap().object().size() != 2) {
        continue;
      }
      ObjectOverlapInfo object1 = cross_ptr->overlap().object()[0];
      ObjectOverlapInfo object2 = cross_ptr->overlap().object()[1];
      double tmp_start = 0.0;
      double tmp_end = 0.0;
      bool tmp_merge = false;
      double tmp_start2 = 0.0;
      double tmp_end2 = 0.0;
      // bool tmp_merge2 = false;
      Id overlap_id;
      if (object1.id().id() == lane_ptr->id().id()) {
        overlap_id = object2.id();
        tmp_start = object1.lane_overlap_info().start_s();
        tmp_end = object1.lane_overlap_info().end_s();
        tmp_merge = object1.lane_overlap_info().is_merge();

        tmp_start2 = object2.lane_overlap_info().start_s();
        tmp_end2 = object2.lane_overlap_info().end_s();
        // tmp_merge2 = object2.lane_overlap_info().is_merge();
        // std::cout << "\t == overlap_id: " << overlap_id.id() << 
        //       ", start: " << tmp_start << 
        //       ", end: "<< tmp_end << ", merge: "<< 
        //       tmp_merge << std::endl;
      }
      else {
        overlap_id = object1.id();
        tmp_start = object2.lane_overlap_info().start_s();
        tmp_end = object2.lane_overlap_info().end_s();
        tmp_merge = object2.lane_overlap_info().is_merge();

        tmp_start2 = object1.lane_overlap_info().start_s();
        tmp_end2 = object1.lane_overlap_info().end_s();
        // tmp_merge2 = object1.lane_overlap_info().is_merge();
        // std::cout << "\t != overlap_id: " << overlap_id.id() << 
        //       ", start: " << tmp_start << 
        //       ", end: "<< tmp_end << ", merge: "<< 
        //       tmp_merge << std::endl;
      }
      
      // lane privilege  tmp: 0,1,2 LanePrivilege lane_pri
      int index2 = 0;
      for (auto lane2 : lane_id2) {
        // std::cout << "lane2: " << lane2.id() << ", overlap_id: " <<
            // overlap_id.id() << std::endl; 
        if (lane2.id() == overlap_id.id()) {  //merge || cross
          col_index1.start_s = tmp_start;
          col_index1.end_s = tmp_end;
          col_index1.index = index1;

          col_index2.start_s = tmp_start2;
          col_index2.end_s = tmp_end2;
          col_index2.index = index2;
          // start_s = tmp_start;
          // end_s = tmp_end;
          // merge/cross/irrelevant  tmp:0,1,2
          if (tmp_merge) {
            relation = 0;  // merge
          }
          else {
            relation = 1; // cross
          }

          // 0 : laneList1 > laneList2; 1 : laneList1 < laneList2; 2 : laneList1 = laneList2;  
          //1）转弯让直行
          if (lane1_turn == acu::hdmap::Lane::NO_TURN && 
                lane2_turn != acu::hdmap::Lane::NO_TURN) {
            lanePrivilege = 0;
          }
          else if (lane1_turn != acu::hdmap::Lane::NO_TURN && 
                lane2_turn == acu::hdmap::Lane::NO_TURN) {
            lanePrivilege = 1;
          }
          else {
            lanePrivilege = 2;
          }

          return 0;
        }

        index2++;
      }
    }
    index1++;
  }
  //对向车道 -> 序列都是直行
  if (lane1_turn == acu::hdmap::Lane::NO_TURN && lane2_turn == acu::hdmap::Lane::NO_TURN) { 
     double heading1 = 0.0;
     double heading2 = 0.0;
     LaneInfoConstPtr lane_ptr = nullptr;
     lane_ptr =  GetLaneById(lane_id1.back());
     if (lane_ptr != nullptr ){
        heading1 = lane_ptr->Heading(0.0) *180.0/M_PI;//headings().front();
        // for(auto h : lane_ptr->headings()) {
        //   std::cout << "  " << h;
        // }
        // std::cout << std::endl;
     }
     lane_ptr = GetLaneById(lane_id2.back());
     if (lane_ptr != nullptr) {
        heading2 = lane_ptr->Heading(0.0) *180.0/M_PI;//headings().front();
        if(heading2 < 0) {
          heading2 += 360;
        }
     }
    //  std::cout << "heading1: " << heading1 << ", heading2: " << heading2 << std::endl;

     double reverse_heading = ReverseAngle(heading1);
    //  std::cout << "reverse heading1: " << reverse_heading << std::endl;
     if(fabs(reverse_heading - heading2) < 5){
       relation = 2;  //对向车道
       lanePrivilege = 2;
       return 0;
     }

  }       
  //无关
  relation = 3; // irrelevant
  lanePrivilege = 2; // laneList1 = laneList2
  return 0;
}

int VectorMap::LoadCarSelectConf() {
  std::string src_path;
  //if (!acu::common::util::GetSrcPath(src_path)) {
  //  return -1;
  //}     //huangchuo
  src_path = "/map/work/";
  std::string top_config_file = src_path + "config/car_select.conf";
  ParamConfigManager param_config_manager;
  param_config_manager.Reset();
  if (!param_config_manager.ModuleConfigLoad(top_config_file)) {
    AERROR << "cannot load config file " << top_config_file;
    return -1;
  }
  ModelConfig *pmodel_config = new ModelConfig();
  param_config_manager.GetModelConfig("car_select", &pmodel_config);
  std::string car_type;
  if (!pmodel_config->GetValue("car_type", &car_type)) {
    AERROR << "cannot get param car_type, set default is taxi";
    car_type = "taxi";
  }
  delete pmodel_config;
  AINFO<<"car_type "<<car_type;
  car_type_ = car_type;
  return 0;
}

HDMapImpl* VectorMap::GetHDMapImplPtr() {
  return &impl_;
} 

}  // namespace vectormap
}  // namespace acu
