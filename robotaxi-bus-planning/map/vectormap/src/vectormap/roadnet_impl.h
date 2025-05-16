/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: roadnet_impl.h
*
* Description: prepare for roadnet API

*
* History:
* lbh         2018/05/18    1.0.0    build this module.
******************************************************************************/

#ifndef ROADNET_IMPL_H_
#define ROADNET_IMPL_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "map/vectormap/src/hdmap/hdmap_common.h"
#include "map_roadnet.pb.h"

/**
 * @namespace acu::vectormap
 * @brief acu::vector
 */
namespace acu {
namespace vectormap {

class RoadNodeInfo;
using namespace acu::hdmap;
using RoadNodeInfoConstPtr = std::shared_ptr<const RoadNodeInfo>;
using RoadNodeTable =
    std::unordered_map<std::string, std::shared_ptr<RoadNodeInfo>>;

class RoadnetImpl {
 public:
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

  RoadNodeInfoConstPtr GetRoadNodeById(const Id &id) const;
  const RoadNodeTable *GetRoadNet() const { return &road_node_table_; }

 private:
  void Clear();
  Roadnet roadnet_;
  RoadNodeTable road_node_table_;
};

class RoadNodeInfo {
 public:
  explicit RoadNodeInfo(const RoadNode &road_node);
  const Id id() const { return road_node_.id(); }
  const double length() const { return road_node_.length(); }
  const double speed_limit() const { return road_node_.speed_limit(); }
  const RoadNode::RoadTurn turn() const { return road_node_.turn(); }

  const std::vector<Id> &GetSuccessorIds() const;
  const std::vector<Id> &GetPredecessorIds() const;
  const RoadNode road_node() const { return road_node_; }

 private:
  RoadNode road_node_;
  std::vector<Id> successor_ids_;
  std::vector<Id> predecessor_ids_;
};

}  // namespace vectormap
}  // namespace acu

#endif
