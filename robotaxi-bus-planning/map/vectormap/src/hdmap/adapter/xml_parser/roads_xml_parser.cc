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
#include <string>
#include <vector>

#include "map/vectormap/src/hdmap/adapter/xml_parser/lanes_xml_parser.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/objects_xml_parser.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/roads_xml_parser.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/signals_xml_parser.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace {
bool IsRoadBelongToJunction(const std::string& road_id) {
  ACU_CHECK(!road_id.empty());
  return road_id != "-1";
}
}  // namespace

namespace acu {
namespace hdmap {
namespace adapter {

void RoadsXmlParser::ParseRoadLinkForOppositeLane(const tinyxml2::XMLElement &xml_node,
                                     std::vector<std::string> &opposite_ids) {
  const tinyxml2::XMLElement *sub_node =
      xml_node.FirstChildElement("neighbor");
  while (sub_node) {
    std::string road_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "elementId", &road_id);
    // checker += UtilXmlParser::QueryStringAttribute(*road_node, "pitch", &pitch);
    // std::cout << "opposite road_id: " << road_id << std::endl;
    if (checker == tinyxml2::XML_SUCCESS) {
      opposite_ids.push_back(road_id + "_1_-1");
    }
    sub_node = sub_node->NextSiblingElement("neighbor");
  }

}  

Status RoadsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                             std::vector<RoadInternal>* roads) {
  ACU_RETURN_IF_NULL(roads);

  auto road_node = xml_node.FirstChildElement("road");
  while (road_node) {
    // road attributes
    std::string id;
    std::string junction_id;
    std::string pitch;
    int checker = UtilXmlParser::QueryStringAttribute(*road_node, "id", &id);
    checker = UtilXmlParser::QueryStringAttribute(*road_node, "pitch", &pitch);
    checker += UtilXmlParser::QueryStringAttribute(*road_node, "junction",
                                                   &junction_id);

    // pitch might be missed in road, so don't check return value
    UtilXmlParser::QueryStringAttribute(*road_node, "pitch", &pitch);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::cout<<"error road is "<<id<<" junction "<<junction_id<<" pitch "<<pitch<<std::endl;
      std::string err_msg = "Error parsing road attributes";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    RoadInternal road_internal;
    road_internal.id = id;
    road_internal.road.mutable_id()->set_id(id);
    road_internal.road.set_pitch(strtod(pitch.c_str(), nullptr));
    if (IsRoadBelongToJunction(junction_id)) {
      road_internal.road.mutable_junction_id()->set_id(junction_id);
    }

    // links for opposite lane 
    // std::cout << "current road_id: " << id << std::endl;
    std::vector<std::string> opposite_ids;
    auto link_node = road_node->FirstChildElement("link");
    if (!link_node) {
      std::string err_msg = "Error parsing road : no link tag";
    } else {
      ParseRoadLinkForOppositeLane(*link_node, opposite_ids);
    }

    // lanes
    RETURN_IF_ERROR(LanesXmlParser::Parse(*road_node, road_internal.id,
                                    &road_internal.sections, opposite_ids));

    // objects
    auto sub_node = road_node->FirstChildElement("objects");
    if (sub_node != nullptr) {
      // stop line
      ObjectsXmlParser::ParseStopLines(*sub_node, &road_internal.stop_lines);
      // crosswalks
      ObjectsXmlParser::ParseCrosswalks(*sub_node, &road_internal.crosswalks);
      // clearareas
      ObjectsXmlParser::ParseClearAreas(*sub_node, &road_internal.clear_areas);
      // speed_bumps
      ObjectsXmlParser::ParseSpeedBumps(*sub_node, &road_internal.speed_bumps);
    }

    // signals
    sub_node = road_node->FirstChildElement("signals");
    if (sub_node != nullptr) {
      // traffic lights
      SignalsXmlParser::ParseTrafficLights(*sub_node,
                                           &road_internal.traffic_lights);
      // stop signs
      SignalsXmlParser::ParseStopSigns(*sub_node, &road_internal.stop_signs);
      // yield signs
      SignalsXmlParser::ParseYieldSigns(*sub_node, &road_internal.yield_signs);
    }

    roads->push_back(road_internal);
    road_node = road_node->NextSiblingElement("road");
  }

  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu
