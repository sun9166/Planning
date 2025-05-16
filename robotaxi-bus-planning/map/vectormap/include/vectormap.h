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
#ifndef ACU_VECTORMAP_H_
#define ACU_VECTORMAP_H_

#include <boost/concept_check.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <tuple>
#include "common/base/macros.h"
#include "geometry.pb.h"
#include "map/vectormap/src/hdmap/hdmap_common.h"
#include "map/vectormap/src/hdmap/hdmap_impl.h"
#include "map/vectormap/src/vectormap/alog.h"
#include "map/vectormap/src/vectormap/roadnet_impl.h"
#include "map_clear_area.pb.h"
#include "map_crosswalk.pb.h"
#include "map_junction.pb.h"
#include "map_lane.pb.h"
#include "map_overlap.pb.h"
#include "map_road.pb.h"
#include "map_signal.pb.h"
#include "map_speed_bump.pb.h"
#include "map_stop_sign.pb.h"
#include "map_yield_sign.pb.h"
#include "common/util/util.h"
#include "common/base/config/include/param_config_manager.h"

/**
 * @namespace acu::vectormap
 * @brief acu::vectormap
 */
namespace acu {
namespace vectormap {

using namespace acu::hdmap;
using roadLanes = std::vector<std::pair<Id, std::vector<Id>>>;

struct CollisionIndex{
  CollisionIndex() = default;
  CollisionIndex(const double start, const double end, const int index)
    : start_s(start), end_s(end), index(index) {}
  double start_s = 0.0;
  double end_s = 0.0;
  int index = 0;
};


/**
 * @brief the point (s, l) in the lane
 */
struct LaneWaypoint {
  LaneWaypoint() = default;
  LaneWaypoint(LaneInfoConstPtr lane, const double s) : lane(lane), s(s) {}
  LaneWaypoint(LaneInfoConstPtr lane, const double s, const double l)
    : lane(lane), s(s), l(l) {}
  LaneInfoConstPtr lane = nullptr;
  double s = 0.0;
  double l = 0.0;
};

/**
 * @brief 2d point: x(m) y(m) angle(-180 ~ +180)
 */
struct PointXYA {
  double x = 0.0;
  double y = 0.0;
  double angle = 0.0;
  PointXYA() = default;
  PointXYA(double px, double py, double pangle) : x(px), y(py), angle(pangle) {}
};

/**
 * @class VectorMap
 */
class VectorMap {
public:
  /**
   * @brief load map from local file
   * @param map_filename path of map data file
   * @return 0:success, otherwise failed
   */
  int LoadMapFromFile(const std::string &map_filename);

  /**
   * @brief load map from a given protobuf message.
   * @param map_proto map data in protobuf format
   * @return 0:success, otherwise failed
   */
  int LoadMapFromProto(const Map &map_proto);

  /**
   * @brief load roadnet from local file
   * @param roadnet_filename path of roadnet data file
   * @return 0:success, otherwise failed
   */
  int LoadRoadnetFromFile(const std::string &roadnet_filename);

  /**
   * @brief load roadnet from a protobuf message
   * @param roadnet_proto map data in protobuf format
   * @return 0:success, otherwise failed
   */
  int LoadRoadnetFromProto(const Roadnet &roadnet_proto);

  /**
   * @brief get lane or road info by id
   */
  LaneInfoConstPtr GetLaneById(const std::string &id) const;
  LaneInfoConstPtr GetLaneById(const Id &id) const;
  RoadInfoConstPtr GetRoadById(const Id &id) const;
  RoadNodeInfoConstPtr GetRoadNodeById(const Id &id) const;
  SignalInfoConstPtr GetSignalById(const Id& id) const;
  /**
   * @brief get the roadnet
   */
  const RoadNodeTable *GetRoadNet() const;
  /**
   * @brief get all roads in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param roads store all roads in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const acu::common::PointENU &point, double distance,
               std::vector<RoadInfoConstPtr> &roads) const;

  /**
   * @brief get all road Ids in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param ids store all road ids in target range
   * @return 0:success, otherwise failed
   */
  int GetRoads(const acu::common::PointENU &point, double distance,
               std::vector<Id> &ids) const;

  /**
   * @brief get nearest road from target point,
   * @param point the target point
   * @param nearest_road_id the nearest road id that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetNearestRoad(const acu::common::PointENU &point,
                     Id &nearest_road_id) const;

  /**
   * @brief get nearest road from target point,
   * @param point the target point
   * @param road_Id the nearest road id that match search conditions
   * @param distance_to_end the distance of current car point to lane end
   * @return 0:success, otherwise, failed.
   */
  int GetNearestRoadWithNoTurn(const common::PointENU& point, Id &road_Id, double &distance_to_end, bool &is_waiting_left) const; 

  /**
   * @brief get nearest road from target point with heading range,
   * @param point the target point, dis range, heading range,
   * @param nearest_road_id the nearest road id that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetNearestRoadWidthHeading(const common::PointENU& point, const double distance, 
                                 const double central_heading,
                                 const double max_heading_difference, 
                                 Id &nearest_road_id,
                                 double& nearest_s, double& nearest_l) const;

  /**
   * @brief get successor ids of the target road ,
   * @param id the target road id
   * @param ids_succ the successor ids
   * @return -1:failed, other: ids_succ.size();
   */
  int GetRoadSuccessorIDs(const Id &id, std::vector<Id> &ids_succ) const;

  /**
   * @brief get predecessor ids of the target road ,
   * @param id the target road id
   * @param ids_succ the predecessor ids
   * @return -1:failed, other: ids_succ.size();
   */
  int GetRoadPredecessorIDs(const Id &id, std::vector<Id> &ids_pred) const;


  /**
   * @brief get lanes in the road,
   * @param id the target road id
   * @param lanes lanes contained in the target road
   * @return 0:success, otherwise, failed.
   */
  int GetRoadLanes(const Id &id, roadLanes &lanes) const;
  int GetRoadLanes(const Id &id, std::vector<std::string> &lanes, bool check_width = false) const;
  int GetRoadLanes(const std::string &id, std::vector<std::string> &lanes, bool check_width = false) const;
  int GetAllRoadLanes(const std::string &id, std::vector<std::pair<std::string, bool>> &lanes) const;
  /**
   * @brief get pitch of the road,
   * @param id the target road id
   * @param pitch pitch of the target road
   * @return 0:success, otherwise, failed.
   */
  int GetRoadPitch(const Id &id, double &pitch) const;

  /**
  * @brief get road id the lane is in,
  * @param lane_id the lane id
  * @param road_id the road id
  * @return 0:success, otherwise, failed.
  */
  int GetRoad(const Id &lane_id, Id &road_id) const;
  int GetRoad(const Id &lane_id, std::string &road_id) const;
  int GetRoad(const std::string &lane_id, std::string &road_id) const;

  /**
   * @brief get nearest lane from target point,
   * @param point the target point
   * @param nearest_lane the nearest lane that match search conditions
   * @param nearest_s the offset from lane start point along lane center line
   * @param nearest_l the lateral offset from lane center line
   * @return 0:success, otherwise, failed.
   */
  int GetNearestLane(const acu::common::PointENU &point,
                     LaneInfoConstPtr &nearest_lane, double &nearest_s,
                     double &nearest_l) const;

   /**
   * @brief get nearest road from target point with heading range,
   * @param point the target point, dis range, heading range,
   * @param nearest_road_id the nearest road id that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetLanesWithHeading(const acu::common::PointENU& point, const double distance,
                          const double central_heading,
                          const double max_heading_difference,
                          std::vector<std::pair<LaneInfoConstPtr, double>>& output_lanes) const;

  int GetNearestLaneWithHeading(const acu::common::PointENU& point, const double distance, 
                              const double central_heading,
                              const double max_heading_difference, 
                              LaneInfoConstPtr& nearest_lane,
                              double& nearest_s, double& nearest_l) const;
   /**
   * @brief get nearest cruise road from target point with heading range,
   * @param point the target point, dis range, heading range,
   * @param nearest_road_id the nearest road id that match search conditions
   * @return 0:success, otherwise, failed.
   */
  int GetNearestCruiseLaneWithHeading(const acu::common::PointENU& point, const double distance, 
                              const double central_heading,
                              const double max_heading_difference, 
                              LaneInfoConstPtr& nearest_lane,
                              double& nearest_s, double& nearest_l) const;
  /**
   * @brief get all lanes in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param lanes store all lanes in target range
   * @return 0:success, otherwise failed
   */
  int GetLanes(const acu::common::PointENU &point, double distance,
               std::vector<LaneInfoConstPtr> &lanes) const;

  /**
   * @brief get all signals in certain range
   * @param point the central point of the range
   * @param distance the search radius
   * @param signals store all signals in target range
   * @return 0:success, otherwise failed
   */
  int GetSignals(const acu::common::PointENU &point, double distance,
                 std::vector<SignalInfoConstPtr> &signals) const;

  /**
   * @brief get centerline of the target lane,
   * @param id the target lane id
   * @param points the centerline points
   * @return 0:success, otherwise, failed.
   */
  int GetLaneCenterLine(const Id &id, std::vector<PointXYA> &points) const;

  /**
  * @brief get left boundary type at a waypoint.
  * @return LaneBoundaryType::Type
  * enum Type {
  *   UNKNOWN = 0;
  *   DOTTED_YELLOW = 1;
  *   DOTTED_WHITE = 2;
  *   SOLID_YELLOW = 3;
  *   SOLID_WHITE = 4;
  *   DOUBLE_YELLOW = 5;
  *   CURB = 6;
  * };
  */
  LaneBoundaryType::Type LeftBoundaryType(const LaneWaypoint &waypoint) const;

  /**
  * @brief get left boundary type at a waypoint.
  */
  LaneBoundaryType::Type RightBoundaryType(const LaneWaypoint &waypoint) const;

  /**
  * @brief get left boundary type at a point.
  */
  LaneBoundaryType::Type LeftBoundaryType(const common::PointENU &point) const;
  LaneBoundaryType::Type LeftBoundaryType(const LaneInfoConstPtr lane) const;

  /**
  * @brief get left boundary type at a point.
  */
  LaneBoundaryType::Type RightBoundaryType(const common::PointENU &point) const;
  LaneBoundaryType::Type RightBoundaryType(const LaneInfoConstPtr lane) const;

  /**
  * @brief get left neighbor lane waypoint. If not exist, the Waypoint.lane will
  * be null.
  */
  LaneWaypoint LeftNeighborWaypoint(const LaneWaypoint &waypoint) const;

  /**
  * @brief get left neighbor lane waypoint. If not exist, the Waypoint.lane will
  * be null.
  */
  LaneWaypoint RightNeighborWaypoint(const LaneWaypoint &waypoint) const;

  /**
   * @brief get distace to lane end,
   * @param point the target point
   * @param distance the offset to lane end point along lane center line
   * @return 0:success, otherwise, failed.
   */
  int DistanceToLaneEnd(const common::PointENU &point, double &distance) const;

  /**
   * @brief get left and right lane ids of the target lane,
   * @param id the target lane id
   * @param id_left the left lane id
   * @param id_right the right lane id
   * @return -1 : false, 0 : no left and right lanes,
   *     1 : only get left lane id, 2 : only get right lane id,
   *    3 : get left and right lane ids.
   */
  int GetLeftRightLaneIDs(const Id &id, Id &id_left, Id &id_right) const;

  /**
   * @brief get opposite lane ids of the target lane,
   * @param id the target lane id
   * @param ids_opposite the opposite lane ids
   * @return -1 : false, 0 : no opposite lanes,
   *     >= 1 : get opposite lane id nums.
   */
  int GetOppositeLaneIDs(const Id &id, std::vector<Id> &ids_opposite) const;

    /**
   * @brief get opposite lane ids of the target lane in junction
   * @param junc_id the target junction
   * @param id the target lane id
   * @param ids_opposite the opposite lane ids
   * @return -1 : false, 0 : no opposite lanes,
   *     >= 1 : get opposite lane id nums.
   */
  int GetJunctionOppositeLaneIDs(const Id &jun_id, const Id &id, 
                                        std::vector<Id> &ids_opposite) const;


  /**
   * @brief get successor ids of the target lane ,
   * @param id the target lane id
   * @param ids_succ the successor ids
   * @return -1:failed, other: ids_succ.size();
   */
  int GetLaneSuccessorIDs(const Id &id, std::vector<Id> &ids_succ) const;
  int GetLaneSuccessorIDs(const std::string &id, std::vector<Id> &ids_succ) const;

  int GetSuccessorLanesInXRoad(const std::string &id, const std::string &x_road, 
          std::vector<std::string> &ids_succ, bool able_driving = true) const;

  /**
   * @brief get speed limit of the target land,
   * @param id the target lane id
   * @param speed the limit speed (m/s), [ km/h in .xml ]
   * @return -1:failed, 0: success, 1: no speed limit value
   */
  int GetLaneSpeedLimit(const Id &id, double &speed) const;

  /**
   * @brief get LaneTurn of the target land,
   * @param id the target lane id
   * @param type the LaneTurn
   *  enum LaneTurn {
   *  NO_TURN = 1;
   *  LEFT_TURN = 2;
   *  RIGHT_TURN = 3;
   *  U_TURN = 4;
   *  };
   * @return -1:failed, 0: success
   */
  int GetLaneTurn(const Id &id, acu::hdmap::Lane::LaneTurn &type) const;

  int GetRoadTurn(const Id &id, int &min_type, std::vector<std::string> &combine_lanes) const;

  int GetLaneType(const Id &id, acu::hdmap::Lane::LaneType &type) const;
  int GetLaneDirection(const Id &id, acu::hdmap::Lane::LaneDirection &direction) const;

  /**9
   * @brief get predecessor ids of the target lane ,
   * @param id the target lane id
   * @param ids_succ the predecessor ids
   * @return -1:failed, other: ids_prede.size();
   */
  int GetLanePredecessorIDs(const Id &id, std::vector<Id> &ids_prede) const;
  int GetLanePredecessorIDs(const std::string &id, std::vector<Id> &ids_prede) const;

  int GetPredecessorLanesInXRoad(const std::string &id, std::string &x_road, std::vector<Id> &ids_prede) const;

  /**
   * @brief get width from point to road boundary. DO NOT get width in junction, if in junction, return -2
   * @param point the target point
   * @param left_width width from point to road left boundary
   * @param right_width width from point to road right boundary
   * @return 0:success, otherwise, failed.
   */
  int GetWidthToRoadBoundary(const common::PointENU &point, double &left_width,
                         double &right_width) const;

  /**
   * @brief get width from point to lane boundary,
   * @param point the target point
   * @param left_width width from point to lane left boundary
   * @param right_width width from point to lane right boundary
   * @return 0:success, otherwise, failed.
   */
  int GetWidthToBoundary(const common::PointENU &point, double &left_width,
                         double &right_width) const;

  /**
   * @brief get lane width contain point,
   * @param point the target point
   * @param width the lane width
   * @return 0:success, otherwise, failed.
   */
  int GetLaneWidth(const common::PointENU &point, double &width) const;
  int GetLaneWidth(const std::string &id, double &width, double s = 0.0) const;

  /**
  * @brief get width to Neighbor lanes,
  * @param point the target point
  * @param left_width width from point to left neighbor lane
  * @param right_width width from point to right neighbor lane
  * @return -1: failed, 0: no left and right neighbors, 1: only have left
  * neighbor,2: only have neighbor, 3 have left and right neighbors.
  */
  int GetWidthToNeighbor(const common::PointENU &point, double &left_width,
                         double &right_width) const;

  /**
   * @brief whether or not the point is on one lane
   * @param point the target point
   * @param lane_id the lane that the point is on
   * return true : point is on lane. false: point is not on lane
   */
  bool IsOnLane(const common::PointENU &point, Id &lane_id) const;

  bool IsOnLane(acu::common::math::Vec2d p, Id &lane_id) const;

  bool IsParkRoad(const std::string &road) const;

  int GetCrosswalks(const acu::common::PointENU& point, double distance,
                    std::vector<CrosswalkInfoConstPtr>* crosswalks) const;
  
  int GetIsolationBelts(const acu::common::PointENU& point, double distance,
                    std::vector<IsolationBeltInfoConstPtr>* isolationbelts) const;

  int GetGuardrails(const acu::common::PointENU& point, double distance,
                    std::vector<GuardrailInfoConstPtr>* guardrails) const;

  int GetInners(const acu::common::PointENU& point, double distance,
                    std::vector<InnerInfoConstPtr>* inners) const;

  int GetOuters(const acu::common::PointENU& point, double distance,
                    std::vector<OuterInfoConstPtr>* outers) const; 

  int GetLaneOverlap(const std::string &base_lane,
                     std::map<std::string, std::pair<double, double>> &base_projections) const;
    /**
   * @brief get extry lane and exit lane in the junction_id
   * @param junction_id   the target junction_id
   * @param ids_extry   extry lane ids
   * @param ids_exit  exit lane ids
   * return -1 : failed; 0: success. 
   */
  int GetExtryExitLane(Id &junction_id, std::vector<Id> &ids_extry,
                      std::vector<Id> &ids_exit) const;                  

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
  int GetLanesRelvant(std::vector<Id> &lane_id1, std::vector<Id> &lane_id2, 
              int &exit_relation,
              int &relation, 
              CollisionIndex &col_index1, CollisionIndex &col_index2,
              int &lanePrivilege) const;

  /**
   * @brief get exit lane relation
   * @param lane_id1   lane id1
   * @param lane_id2   lane id2 
   * @param exit_relation  exit lane relation: 0:无关 ; 1:交叉 ; 2: 重叠
   * return -1 : failed; 0: success. 
   */
  int GetExitRelation(std::vector<Id> &lane_id1, std::vector<Id> &lane_id2, 
              int &exit_relation) const;    

  /**
    get car_type from car_select.conf
  */
  int LoadCarSelectConf();  
  std::string GetCarType() const {return car_type_;};

  HDMapImpl* GetHDMapImplPtr();
  /*
    static std::shared_ptr<VectorMap> Initance();
    static std::shared_ptr<VectorMap> vectormap_ptr_;
  */
private:
  double ReverseAngle(double &angle_input) const {
      double anglerr = 0.0;
      angle_input = (angle_input < 0.0)?  (angle_input + 360.0) : angle_input;
      angle_input = (angle_input > 360.0)?  (angle_input - 360.0) : angle_input;
      anglerr = (angle_input < 180.0)?  (angle_input + 180.0) : (angle_input - 180.0);
      return anglerr;
  }
private:
  HDMapImpl impl_;
  RoadnetImpl roadnet_impl_;
  std::string car_type_ = "taxi";
  /*
   protected:
    VectorMap(){}
    VectorMap(const VectorMap &){}
    VectorMap &operator = (const VectorMap &){}
    */
};

}  // namespace vectormap
}  // namespace acu

#endif  // ACU_VECTORMAP_H_
