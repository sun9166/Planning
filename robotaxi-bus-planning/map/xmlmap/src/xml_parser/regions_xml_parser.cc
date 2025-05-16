/*
idriverplus
create 20181122 lbh
modify 20190613 pgq
=========================================================================*/
#include "regions_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int RegionsXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                            Regions* regions) {
  auto regions_node = xml_node.FirstChildElement("regions");
  if(!regions_node)
      return 0;
  //ACU_RETURN_IF_ERROR(!regions_node, "xml data missing regions.");

  const tinyxml2::XMLElement* sub_node =
      regions_node->FirstChildElement("function_region");
  while (sub_node) {
    std::string id, name, type;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "name", &name);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "type", &type);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing function_region attributes.");
    const tinyxml2::XMLElement* sub_node2 =
        sub_node->FirstChildElement("outline");
    ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing outline.");
    Outlines outlines;
    while(sub_node2) {
       Outline outline;
       ACU_RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node2, &outline),
                            "Error parsing ParseOutline");
       outlines.push_back(outline);
       sub_node2 = sub_node2->NextSiblingElement("outline");
     }
    outlines.shrink_to_fit();
    const tinyxml2::XMLElement* sub_node3 =
         sub_node->FirstChildElement("curbs");
    Curbs curbs;
    if(sub_node3) {
        ACU_RETURN_IF_ERROR(!sub_node3, "xml data missing curbs.");
        ACU_RETURN_IF_ERROR(UtilXmlParser::ParseCurbs(*sub_node3, &curbs),
                         "Error parsing ParseCurbs");
    }
    regions->push_back({id, name, type, outlines, curbs});
    sub_node = sub_node->NextSiblingElement("function_region");
  }
  (*regions).shrink_to_fit();
  return 0;
}
}
}
}
