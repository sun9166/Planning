/*
idriverplus
create 20181122 xf
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_MAP_H_
#define ACU1_XMLMAP_XML_PARSER_MAP_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class MapXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, mHeader* header);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
