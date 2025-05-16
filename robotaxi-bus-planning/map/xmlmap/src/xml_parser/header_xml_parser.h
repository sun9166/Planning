/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_HEADER_H_
#define ACU1_XMLMAP_XML_PARSER_HEADER_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class HeaderXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, Header* header);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
