/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: roadnet_xml_parser.cc
*
* Description: get roadnet from xml

*
* History:
* lbh         2018/05/18    1.0.0    build this module.
******************************************************************************/
#include <string>
#include <vector>

#include "map/vectormap/src/hdmap/adapter/xml_parser/util_xml_parser.h"
#include "map/vectormap/src/vectormap/roadnet_xml_parser.h"

namespace acu {
namespace hdmap {
namespace adapter {

int RoadnetXmlParser::GetRoadnetProto(const tinyxml2::XMLElement &xml_node,
                                      PbRoadnet *roadnet) {
  ACU_RETURN_IF_NULL(roadnet);
  Status status = ParseRoadNode(xml_node);
  if (!status.ok()) {
    std::cout<<"fail to parse opendrive roadnet, "
                     << status.error_message()<<std::endl;
    return -1;
  }
  return ToProto(roadnet);
}

Status RoadnetXmlParser::ParseRoadNode(const tinyxml2::XMLElement &xml_node) {
  auto road_node = xml_node.FirstChildElement("road");
  while (road_node) {
    // road attributes
    std::string id;
    double length;
    double speed_limit;
    std::string turn;
    int checker = UtilXmlParser::QueryStringAttribute(*road_node, "id", &id);
    checker += road_node->QueryDoubleAttribute("length", &length);
    checker += road_node->QueryDoubleAttribute("max", &speed_limit);
    checker +=
        UtilXmlParser::QueryStringAttribute(*road_node, "turnType", &turn);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parsing road attributes";
      //       return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR,
      //       err_msg);
    }
    vectorStr successor_ids;
    vectorStr predecessor_ids;
    auto sub_node = road_node->FirstChildElement("link");
    if (!sub_node) {
      std::string err_msg = "Error parsing road : no link tag";
      //       return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR,
      //       err_msg);
    } else {
      ParseRoadLink(*sub_node, successor_ids, predecessor_ids);

      roadnet_.pb_roadnode.emplace_back(id, length, speed_limit, turn,
                                        successor_ids, predecessor_ids);
    }
    road_node = road_node->NextSiblingElement("road");
  }
  return Status::OK();
}

int RoadnetXmlParser::ToProto(PbRoadnet *roadnet) {
  for (auto &roadnode : roadnet_.pb_roadnode) {
    auto pbroadnode = roadnet->add_road_node();
    GetRoadNode(roadnode, pbroadnode);
  }
  return 0;
}

void RoadnetXmlParser::GetRoadNode(const RoadNode &roadnode,
                                   PbRoadNode *pbroadnode) {
  (pbroadnode->mutable_id())->set_id(roadnode.id);
  pbroadnode->set_length(roadnode.length);
  pbroadnode->set_speed_limit(roadnode.speed_limit);
  std::string upper_str = UtilXmlParser::ToUpper(roadnode.turn);
  if (upper_str == "NOTURN") {
    pbroadnode->set_turn(hdmap::RoadNode::NO_TURN);
  } else if (upper_str == "LEFTTURN") {
    pbroadnode->set_turn(hdmap::RoadNode::LEFT_TURN);
  } else if (upper_str == "RIGHTTURN") {
    pbroadnode->set_turn(hdmap::RoadNode::RIGHT_TURN);
  } else if (upper_str == "UTURN") {
    pbroadnode->set_turn(hdmap::RoadNode::U_TURN);
  }
  for (auto &id : roadnode.successor_ids)
    pbroadnode->add_successor_id()->set_id(id);
  for (auto &id : roadnode.predecessor_ids)
    pbroadnode->add_predecessor_id()->set_id(id);
  return;
}

void RoadnetXmlParser::ParseRoadLink(const tinyxml2::XMLElement &xml_node,
                                     vectorStr &successor_ids,
                                     vectorStr &predecessor_ids) {
  const tinyxml2::XMLElement *sub_node =
      xml_node.FirstChildElement("predecessor");
  while (sub_node) {
    std::string road_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "elementId", &road_id);
    if (checker == tinyxml2::XML_SUCCESS) {
      predecessor_ids.push_back(road_id);
    }
    sub_node = sub_node->NextSiblingElement("predecessor");
  }

  sub_node = xml_node.FirstChildElement("successor");
  while (sub_node) {
    std::string road_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "elementId", &road_id);
    if (checker == tinyxml2::XML_SUCCESS) {
      successor_ids.push_back(road_id);
    }
    sub_node = sub_node->NextSiblingElement("successor");
  }
}
}  // namespace adapter
}  // namespace vectormap
}  // namespace acu
