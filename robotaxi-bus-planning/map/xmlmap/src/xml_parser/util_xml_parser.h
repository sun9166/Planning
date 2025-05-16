/*
idriverplus
create 20181122 lbh
midify 20190712 pgq
=========================================================================*/

#ifndef ACU1_XMLMAP_XML_PARSER_UTIL_H_
#define ACU1_XMLMAP_XML_PARSER_UTIL_H_

#include <string>
#include <vector>

#include "tinyxml2.h"
#include "alog.h"
#include "xmlstruct_common.h"
#include "xmlstruct_mapinfo.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class UtilXmlParser {
 public:
  static int ParseRegionBoundary(const tinyxml2::XMLElement& xml_node,
                                 RegionBoundary* region_boundary);
  static int ParseGeometry(const tinyxml2::XMLElement& xml_node,
                           Geometry* geometry);
  static int ParseOutline(const tinyxml2::XMLElement& xml_node,
                          Outline* outline);
  static int ParseCurbs(const tinyxml2::XMLElement& xml_node,
                        Curbs* curbs);
  static int ParsePointSet(const tinyxml2::XMLElement& xml_node,
                           PointSet* pointset);
  static int ParsePosition(const tinyxml2::XMLElement& xml_node,
                           Position* position);
  static int ParseOrigin(const tinyxml2::XMLElement& xml_node, Origin* origin);

  static int ParseBoundary(const tinyxml2::XMLElement& xml_node,
                           acu::xmlmap::Boundary* boundary);
  static int ParseNameSet(const tinyxml2::XMLElement& xml_node,
                          Nameset* nameset);
  static int ParsePoint(const tinyxml2::XMLElement& xml_node, Point* pt);

  static int ParseEdges(const tinyxml2::XMLElement& xml_node, Edges* edges);

  static tinyxml2::XMLError QueryStringAttribute(
      const tinyxml2::XMLElement& xml_node, const std::string& name,
      std::string* value);

  // static tinyxml2::XMLError QueryIntAttribute(
  //     const tinyxml2::XMLElement& xml_node, const std::string& name,
  //     int& value);
};
}
}
}

#endif
