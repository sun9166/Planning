/*
idriverplus
create 20190712 pgq
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_BOUNDARIES_H_
#define ACU1_XMLMAP_XML_PARSER_BOUNDARIES_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class BoundariesXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, Boundaries* boundaries);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
