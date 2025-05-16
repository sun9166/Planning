/*
idriverplus
create 20181122 lbh
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_POINTS_H_
#define ACU1_XMLMAP_XML_PARSER_POINTS_H_

#include "tinyxml2.h"
#include "util_xml_parser.h"
#include <sstream>

namespace acu {
namespace xmlmap {
namespace adapter {

class PointsXmlParser {
 public:
  static int Parse(const tinyxml2::XMLElement& xml_node, Points* points);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
