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

#include "map/vectormap/src/vectormap/roadnet_impl.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include "common/util/file.h"
#include "common/util/string_util.h"
#include <unordered_set>
#include "map/vectormap/src/hdmap/adapter/opendrive_adapter.h"

namespace acu {
namespace vectormap {

int RoadnetImpl::LoadRoadnetFromFile(const std::string& roadnet_filename) {
  Clear();
  if (acu::common::util::EndWith(roadnet_filename, ".xml")) {
    if (!adapter::OpendriveAdapter::LoadData(roadnet_filename, &roadnet_)) {
      return -1;
    }
  } else if (!acu::common::util::GetProtoFromFile(roadnet_filename, &roadnet_)) {
    return -1;
  }
  return LoadRoadnetFromProto(roadnet_);
}

int RoadnetImpl::LoadRoadnetFromProto(const Roadnet& roadnet_proto) {
  if (&roadnet_proto != &roadnet_) {  // avoid an unnecessary copy
    Clear();
    roadnet_ = roadnet_proto;
  }
  for (const auto& road_node : roadnet_.road_node()) {
    road_node_table_[road_node.id().id()].reset(new RoadNodeInfo(road_node));
  }
  std::cout<<"load roadnodes in roadnet: "<<road_node_table_.size()<<std::endl;
  return 0;
}

RoadNodeInfoConstPtr RoadnetImpl::GetRoadNodeById(const Id& id) const {
  RoadNodeTable::const_iterator it = road_node_table_.find(id.id());
  return it != road_node_table_.end() ? it->second : nullptr;
}
void RoadnetImpl::Clear() {
  road_node_table_.clear();
}

RoadNodeInfo::RoadNodeInfo(const RoadNode &road_node) : road_node_(road_node) {
    successor_ids_.clear();
    predecessor_ids_.clear();
    for(auto& id:road_node_.predecessor_id())
	predecessor_ids_.push_back(id);
    for(auto& id:road_node_.successor_id())
	successor_ids_.push_back(id);
}

const std::vector<Id> &RoadNodeInfo::GetSuccessorIds() const {
  return successor_ids_;
}
const std::vector<Id> &RoadNodeInfo::GetPredecessorIds() const {
  return predecessor_ids_;
}

}  // namespace hdmap
}  // namespace acu
