/*
idriverplus
create 20181122 lbh
midify 20190712 pgq
=========================================================================*/
#include "jobs_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {
int JobsXmlParser::Parse(const tinyxml2::XMLElement& xml_node, Jobs* jobs) {
  auto jobs_node = xml_node.FirstChildElement("jobs");
  if(!jobs_node)
      return 0;
  //ACU_RETURN_IF_ERROR(!jobs_node, "xml data missing jobs.");

  const tinyxml2::XMLElement* sub_node = jobs_node->FirstChildElement("job");
  while (sub_node) {
    std::string id, from, to;
    int checker = UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "from", &from);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "to", &to);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing job attributes.");
    auto sub_node2 = sub_node->FirstChildElement("connect_segment");
    ACU_RETURN_IF_ERROR(!sub_node2, "xml data missing connect_segment.");
    ConnectSegments connect_segments;
    while(sub_node2) {
      ConnectSegment connect_segment;
      std::string id;
      int order;
      int checker = tinyxml2::XML_SUCCESS;
      checker += UtilXmlParser::QueryStringAttribute(*sub_node2, "id", &id);
      ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                           "Error parsing connect segment id attributes");
      int checker_order = sub_node2->QueryIntAttribute("order", &order);
      if (checker_order != tinyxml2::XML_SUCCESS) {
          order = 1;
       }
      connect_segment.id = id;
      connect_segment.order = order;
      connect_segments.push_back(connect_segment);
      sub_node2 = sub_node2->NextSiblingElement("connect_segment");
    }
    connect_segments.shrink_to_fit();
    jobs->push_back({id, from, to, connect_segments});
    sub_node = sub_node->NextSiblingElement("job");
  }
  (*jobs).shrink_to_fit();
  return 0;
}
}
}
}
