/*
idriverplus
create 20181122 lbh
midify 20180108 xf
midify 20190712 pgq
=========================================================================*/

#ifndef ACU1_XMLMAP_ADAPTER_H_
#define ACU1_XMLMAP_ADAPTER_H_
#include <fstream>
#include <iostream>
#include <string>

#include "tinyxml2.h"
#include "alog.h"
#include "header_xml_parser.h"
#include "jobs_xml_parser.h"
#include "map_xml_parser.h"
#include "mapinfo_xml_parser.h"
#include "boundaries_xml_parser.h"
#include "points_xml_parser.h"
#include "regions_xml_parser.h"
#include "roads_xml_parser.h"
#include "surfaces_xml_parser.h"
#include "location_xml_parser.h"
#include "topo_xml_parser.h"
#include "xmlstruct_common.h"
#include "xmlstruct_mapinfo.h"

namespace acu {
namespace xmlmap {
namespace adapter {

class XmlMapAdapter {
 public:
  static int LoadData(const std::string &filename, Vectormap *vectormap);
  static int LoadData(const std::string &filename, MapinfoXml *mapinfoxml);
  static int LoadData(const std::string &filename, MapXml *mapxml);
  static int SetMapHeader(const std::string &filename,
                          const std::string &default_name);
};

}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu

#endif
