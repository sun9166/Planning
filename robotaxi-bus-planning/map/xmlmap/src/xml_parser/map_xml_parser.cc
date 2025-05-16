/*
idriverplus
create 20181122 xf
=========================================================================*/
#include "map_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int MapXmlParser::Parse(const tinyxml2::XMLElement& xml_node, mHeader* header) {
  auto sub_node = xml_node.FirstChildElement("header");
  ACU_RETURN_IF_ERROR(!sub_node, "xml data missing mheader.");

  std::string default_name, date;
  int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "default_name",
                                                    &default_name);
  checker += UtilXmlParser::QueryStringAttribute(*sub_node, "date", &date);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing mheader attributes.");

  std::vector<std::string> nameset;
  const tinyxml2::XMLElement* sub_node2 =
      sub_node->FirstChildElement("nameset");
  if (sub_node2)
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParseNameSet(*sub_node2, &nameset),
                         "Error parsing ParseNameSet");
  // header->push_back({default_name, date, nameset});
  header->name = default_name;
  header->date = date;
  header->nameset = nameset;
  return 0;
}
}
}
}
