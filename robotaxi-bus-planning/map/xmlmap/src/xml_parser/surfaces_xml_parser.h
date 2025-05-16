/*
idriverplus
create 20190108 xf
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_SURFACES_H_
#define ACU1_XMLMAP_XML_PARSER_SURFACES_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class SurfacesXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, Surfaces* surfaces);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
