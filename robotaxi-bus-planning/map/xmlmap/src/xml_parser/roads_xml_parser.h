/*
idriverplus
create 20181122 lbh
midify 20190712 pgq
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_ROADS_H_
#define ACU1_XMLMAP_XML_PARSER_ROADS_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class RoadsXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, Roads* roads);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
