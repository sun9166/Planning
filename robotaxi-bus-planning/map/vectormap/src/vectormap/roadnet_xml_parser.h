/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: roadnet_xml_parser.h
*
* Description: get roadnet from xml

*
* History:
* lbh         2018/05/18    1.0.0    build this module.
******************************************************************************/
#ifndef ROADNET_XML_PARSER_H_
#define ROADNET_XML_PARSER_H_

#include <string>
#include <vector>

#include "tinyxml2.h"

#include "map/vectormap/src/hdmap/adapter/xml_parser/common_define.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/status.h"
#include "map_roadnet.pb.h"

namespace acu {
namespace hdmap {
namespace adapter {

using PbRoadNode = acu::hdmap::RoadNode;
using PbRoadnet = acu::hdmap::Roadnet;
using vectorStr = std::vector<std::string>;

struct RoadNode {
  std::string id;
  double length;
  double speed_limit;
  std::string turn;
  vectorStr successor_ids;
  vectorStr predecessor_ids;
  
  RoadNode() = default;
  RoadNode(std::string id, double length, double speed_limit, std::string turn,
           std::vector<std::string> successor_ids,
           std::vector<std::string> predecessor_ids)
      : id(id),
        length(length),
        speed_limit(speed_limit),
        turn(turn),
        successor_ids(successor_ids),
        predecessor_ids(predecessor_ids) {}
};

struct Roadnet {
  std::vector<RoadNode> pb_roadnode;
};

class RoadnetXmlParser {
 public:
  int GetRoadnetProto(const tinyxml2::XMLElement &xml_node, PbRoadnet *roadnet);

 private:
  Status ParseRoadNode(const tinyxml2::XMLElement &xml_node);
  void ParseRoadLink(const tinyxml2::XMLElement &xml_node,
                     vectorStr &successor_ids, vectorStr &predecessor_ids);
  int ToProto(PbRoadnet *roadnet);
  void GetRoadNode(const RoadNode &roadnode, PbRoadNode *pbroadnode);

 private:
  Roadnet roadnet_;
};

}  // namespace adapter
}  // namespace vectormap
}  // namespace acu

#endif
