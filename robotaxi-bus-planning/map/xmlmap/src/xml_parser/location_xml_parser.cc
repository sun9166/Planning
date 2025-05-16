/*
idriverplus
create 20181202 xf
=========================================================================*/
#include "location_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {
int LocationXmlParser::Parse(const tinyxml2::XMLElement& xml_node, PathPoint* pathpoint) {
  auto location_node = xml_node.FirstChildElement("pathPoint");
  // ACU_RETURN_IF_ERROR(!location_node, "xml data missing pathPoint.");
  if (location_node == nullptr) {
    return 0;
  }

  const tinyxml2::XMLElement* sub_node = location_node->FirstChildElement("voronoi");
  while (sub_node) {
    std::string id;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing voronoi attributes.");

    // auto sub_node2 = sub_node->FirstChildElement("centerPoint");
    // ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing centerPoint.");

    Point center_point;
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParsePoint(*sub_node, &center_point),
                         "Error parsing center_point");
    pathpoint->push_back({id, center_point});
    sub_node = sub_node->NextSiblingElement("voronoi");
  }
  (*pathpoint).shrink_to_fit();
  return 0;
}
}
}
}
