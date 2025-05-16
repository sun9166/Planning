/*
idriverplus
create 20181202 xf
=========================================================================*/
#include "topo_xml_parser.h"
namespace acu {
namespace xmlmap {
namespace adapter {
int TopoXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                            Topology* topology) {
  auto topo_node = xml_node.FirstChildElement("topology");
  // ACU_RETURN_IF_ERROR(!topo_node, "xml data missing topology.");
  if (topo_node == nullptr) {
    return 0;
  }

  const tinyxml2::XMLElement* sub_node = topo_node->FirstChildElement("edges");

  while (sub_node) {
    Edges edges;
    ACU_RETURN_IF_ERROR(UtilXmlParser::ParseEdges(*sub_node, &edges),
                         "Error parsing ParseEdges");
    topology->push_back(edges);
    sub_node = sub_node->NextSiblingElement("edges");
  }
  (*topology).shrink_to_fit();
  return 0;
}
}
}
}
