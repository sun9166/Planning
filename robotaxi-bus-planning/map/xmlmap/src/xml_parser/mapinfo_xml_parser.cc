/*
idriverplus
create 20181122 xf
=========================================================================*/
#include "mapinfo_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {
int MapinfoXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                            miHeader* header) {
  auto sub_node = xml_node.FirstChildElement("header");
  ACU_RETURN_IF_ERROR(!sub_node, "mapinfi xml data missing header.");

  std::string name, version, describe, date;
  int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "name", &name);
  checker += UtilXmlParser::QueryStringAttribute(*sub_node, "date", &date);
  checker +=
      UtilXmlParser::QueryStringAttribute(*sub_node, "describe", &describe);
  checker +=
      UtilXmlParser::QueryStringAttribute(*sub_node, "version", &version);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing miheader attributes.");
  header->name = name;
  header->version = version;
  header->describe = describe;
  header->date = date;
  return 0;
}
}
}
}
