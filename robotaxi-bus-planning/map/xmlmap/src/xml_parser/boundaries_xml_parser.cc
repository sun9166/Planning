/*
idriverplus
create 20190712 pgq
=========================================================================*/
#include "boundaries_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {
int BoundariesXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                            Boundaries* boundaries) {
  auto boundaries_node = xml_node.FirstChildElement("boundaries");
  if (boundaries_node) {
    ACU_RETURN_IF_ERROR(!boundaries_node, "xml data missing boundaries.");
    const tinyxml2::XMLElement* sub_node = boundaries_node->FirstChildElement("boundary");
    if (sub_node) {
      ACU_RETURN_IF_ERROR(!sub_node, "xml data missing boundary.");
      while (sub_node) {
        RegionBoundary region_boundary;
        ACU_RETURN_IF_ERROR(UtilXmlParser::ParseRegionBoundary(*sub_node, &region_boundary),
            "Error parsing ParseRegionBoundary");
        boundaries->push_back(region_boundary);
        sub_node = sub_node->NextSiblingElement("boundary");
      }
    }
  }
  (*boundaries).shrink_to_fit();
  return 0;
}
}
}
}
