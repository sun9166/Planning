/*
idriverplus
create 20190108 xf
=========================================================================*/
#include "surfaces_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int SurfacesXmlParser::Parse(const tinyxml2::XMLElement& xml_node, Surfaces* surfaces) {
  auto surface_node = xml_node.FirstChildElement("surfaces");
  if(!surface_node)
      return 0;
  //ACU_RETURN_IF_ERROR(!surface_node, "xml data missing surfaces.");

  const tinyxml2::XMLElement* sub_node =
      surface_node->FirstChildElement("road_region");
  while (sub_node) {
    std::string id;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing segment attributes.");
    const tinyxml2::XMLElement* sub_node2 =
        sub_node->FirstChildElement("outline");
    ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing geometry.");
    Outline outline;
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node2, &outline),
                         "Error parsing ParseOutline");
    surfaces->push_back({id, outline});
    sub_node = sub_node->NextSiblingElement("road_region");
  }
  (*surfaces).shrink_to_fit();
  return 0;
}
}
}
}
