/*
idriverplus
create 20181122 lbh
=========================================================================*/
#include "points_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int PointsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                           Points* points) {
  auto points_node = xml_node.FirstChildElement("points");
  // ACU_RETURN_IF_ERROR(!points_node, "xml data missing points.");
  if (points_node == nullptr) {
    return 0;
  }

  const tinyxml2::XMLElement* sub_node =
      points_node->FirstChildElement("function_point");
  while (sub_node) {
    std::string id, name, type;
    //int type;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "name", &name);

    //checker += sub_node->QueryIntAttribute("type", &type);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "type", &type);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing function_point attributes.");
    const tinyxml2::XMLElement* sub_node2 =
        sub_node->FirstChildElement("position");
    ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing position.");
    Position position;
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParsePosition(*sub_node2, &position),
                         "Error parsing ParsePosition");
    points->push_back({id, name, type, position});
    sub_node = sub_node->NextSiblingElement("function_point");
  }
  (*points).shrink_to_fit();
  return 0;
}
}
}
}
