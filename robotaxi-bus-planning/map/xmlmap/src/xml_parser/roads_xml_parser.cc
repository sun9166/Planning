/*
idriverplus
create 20181122 lbh
midify 20190712 pgq
=========================================================================*/
#include "roads_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int RoadsXmlParser::Parse(const tinyxml2::XMLElement& xml_node, Roads* roads) {
  auto road_node = xml_node.FirstChildElement("roads");
  if(!road_node)
      return 0;
  //ACU_RETURN_IF_ERROR(!road_node, "xml data missing roads.");

  const tinyxml2::XMLElement* sub_node =
      road_node->FirstChildElement("segment");
  while (sub_node) {
    std::string id, type;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing segment id attributes.");
    int checker_type = UtilXmlParser::QueryStringAttribute(*sub_node, "type", &type);
    if (checker_type != tinyxml2::XML_SUCCESS) {
      type = "W";
     }
    const tinyxml2::XMLElement* sub_node1 =
        sub_node->FirstChildElement("contact_boundary");
    std::string contact_boundary_id = "-1";
    if (sub_node1) {
       ACU_RETURN_IF_ERROR(!sub_node1, "xml data missing contact boundary.");
       int checker1 = UtilXmlParser::QueryStringAttribute(*sub_node1, "id", &contact_boundary_id);
       ACU_RETURN_IF_ERROR(checker1 != tinyxml2::XML_SUCCESS,
                           "Error parsing contact boundary id attributes.");
     }
    const tinyxml2::XMLElement* sub_node2 =
        sub_node->FirstChildElement("geometry");
    ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing geometry.");
    Geometry geo;
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParseGeometry(*sub_node2, &geo),
                         "Error parsing ParseGeometry");
    roads->push_back({id, type, contact_boundary_id, geo});
    sub_node = sub_node->NextSiblingElement("segment");
  }
  (*roads).shrink_to_fit();
  return 0;
}
}
}
}
