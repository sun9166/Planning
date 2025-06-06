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

#include "map/vectormap/src/hdmap/adapter/xml_parser/objects_xml_parser.h"

#include <string>
#include <vector>

#include "map/vectormap/src/hdmap/adapter/xml_parser/util_xml_parser.h"

namespace acu {
namespace hdmap {
namespace adapter {

Status ObjectsXmlParser::ParseCrosswalks(const tinyxml2::XMLElement& xml_node,
                                         std::vector<PbCrosswalk>* crosswalks) {
  ACU_RETURN_IF_NULL(crosswalks);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "crosswalk") {
      PbCrosswalk crosswalk;
      crosswalk.mutable_id()->set_id(object_id);
      PbPolygon* polygon = crosswalk.mutable_polygon();
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse crosswalk outline";
        return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      crosswalks->emplace_back(crosswalk);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParseClearAreas(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbClearArea>* clear_areas) {
  ACU_RETURN_IF_NULL(clear_areas);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("object");
  while (sub_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*sub_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*sub_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "cleararea") {
      PbClearArea clear_area;
      clear_area.mutable_id()->set_id(object_id);
      PbPolygon* polygon = clear_area.mutable_polygon();
      ACU_CHECK(polygon != nullptr);
      const tinyxml2::XMLElement* outline_node =
          sub_node->FirstChildElement("outline");
      if (outline_node == nullptr) {
        std::string err_msg = "Error parse cleararea outline";
        return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*outline_node, polygon));
      clear_areas->emplace_back(clear_area);
    }
    sub_node = sub_node->NextSiblingElement("object");
  }

  return Status::OK();
}

Status ObjectsXmlParser::ParseSpeedBumps(
    const tinyxml2::XMLElement& xml_node,
    std::vector<PbSpeedBump>* speed_bumps) {
  ACU_RETURN_IF_NULL(speed_bumps);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }
    
    if (object_type == "speedbump") {
      PbSpeedBump speed_bump;
      const tinyxml2::XMLElement* sub_node =
          object_node->FirstChildElement("outline");
      speed_bump.mutable_id()->set_id(object_id);
      PbPolygon* polygon = speed_bump.mutable_polygon();
      if (sub_node == nullptr) {
        std::string err_msg = "Error parse speedbump outline";
        return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
      speed_bumps->emplace_back(speed_bump);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Status::OK();
}

Status ObjectsXmlParser::ParseStopLines(
    const tinyxml2::XMLElement& xml_node,
    std::vector<StopLineInternal>* stop_lines) {
  ACU_RETURN_IF_NULL(stop_lines);
  const tinyxml2::XMLElement* object_node =
      xml_node.FirstChildElement("object");
  while (object_node) {
    std::string object_type;
    std::string object_id;
    int checker =
        UtilXmlParser::QueryStringAttribute(*object_node, "id", &object_id);
    checker +=
        UtilXmlParser::QueryStringAttribute(*object_node, "type", &object_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse object type.";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    if (object_type == "stopline") {
      StopLineInternal stop_line;
      stop_line.id = object_id;
      PbCurveSegment* curve_segment = stop_line.curve.add_segment();
      ACU_CHECK(curve_segment != nullptr);
      const auto sub_node = object_node->FirstChildElement("geometry");
      if (sub_node == nullptr) {
        std::string err_msg = "Error parse stopline geometry";
        return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
      }
      RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node, curve_segment));
      stop_lines->emplace_back(stop_line);
    }
    object_node = object_node->NextSiblingElement("object");
  }
  return Status::OK();
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu
