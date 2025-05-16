/*
idriverplus
create 20181122 lbh
=========================================================================*/
#include "header_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {
int HeaderXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                           Header* header) {
  auto sub_node = xml_node.FirstChildElement("header");
  ACU_RETURN_IF_ERROR(!sub_node, "xml data missing header.");

  std::string name, version, date;
  int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "name", &name);
  checker +=
      UtilXmlParser::QueryStringAttribute(*sub_node, "version", &version);
  checker += UtilXmlParser::QueryStringAttribute(*sub_node, "date", &date);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing header attributes.");
  header->name = name;
  header->version = version;
  header->date = date;
  auto sub_node2 = sub_node->FirstChildElement("move_origin");
  ACU_RETURN_IF_ERROR(!sub_node2, "Error finding header move_origin tag");
  ACU_RETURN_IF_ERROR(UtilXmlParser::ParseOrigin(*sub_node2, &header->origin),
                       "Error parsing ParseOrigin");
  auto* boundary = &header->boundary;
  ACU_RETURN_IF_ERROR(UtilXmlParser::ParseBoundary(*sub_node, boundary),
                       "Error parsing boundary point");
  return 0;
}
}
}
}
