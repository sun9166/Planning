/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/
#include "map/vectormap/src/hdmap/adapter/opendrive_adapter.h"

#include <vector>

#include "map/vectormap/src/hdmap/adapter/proto_organizer.h"
#include "map/vectormap/src/hdmap/adapter/xml_parser/status.h"
#include "map/vectormap/src/vectormap/alog.h"

namespace acu {
namespace hdmap {
namespace adapter {

bool OpendriveAdapter::LoadData(const std::string &filename,
                                acu::hdmap::Map *pb_map,
                                acu::hdmap::Roadnet *pb_roadnet) {
  printf("BEGIN TO LOAD MAP DATA ... ...");
  if (!LoadData(filename, pb_map)) return false;
  printf("BEGIN TO LOAD ROADNET DATA ... ...");
  if (!LoadData(filename, pb_roadnet)) return false;
  return true;
}
bool OpendriveAdapter::LoadData(const std::string &filename,
                                acu::hdmap::Map *pb_map) {
  ACU_RETURN_IF_NULL(pb_map);

  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return false;
  }

  // root node
  const tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);
  // header
  PbHeader *map_header = pb_map->mutable_header();
  Status status = HeaderXmlParser::Parse(*root_node, map_header);
  if (!status.ok()) {
    std::cout<<"fail to parse opendrive header, "
                     << status.error_message()<<std::endl;
    return false;
  }

  // roads
  std::vector<RoadInternal> roads;
  status = RoadsXmlParser::Parse(*root_node, &roads);
  if (!status.ok()) {
    std::cout<<"fail to parse opendrive road, "
                     << status.error_message()<<std::endl;
    return false;
  }

  // junction
  std::vector<JunctionInternal> junctions;
  status = JunctionsXmlParser::Parse(*root_node, &junctions);
  if (!status.ok()) {
    std::cout<<"fail to parse opendrive junction, "
                     << status.error_message()<<std::endl;
    return false;
  }

  //road_surface
  std::vector<RoadSurfaceInternal> roadsurfaces;
  status = RoadSurfaceXmlParser::Parse(*root_node, &roadsurfaces);
  if (!status.ok()) {
    std::cout<<"fail to parse opendrive road_surface, "
                     << status.error_message()<<std::endl;
    return false;
  }

  ProtoOrganizer proto_organizer;
  proto_organizer.GetRoadElements(&roads);
  proto_organizer.GetJunctionElements(junctions);
  proto_organizer.GetRoadsurfaces(roadsurfaces);
  proto_organizer.GetOverlapElements(roads, junctions);
  proto_organizer.OutputData(pb_map);
  return true;
  /*
  if (pb_roadnet == nullptr) return false;
  printf("BEGIN TO LOAD ROADNET DATA ... ...");
  RoadnetXmlParser roadnet_par;
  int ret = roadnet_par.GetRoadnetProto(*root_node, pb_roadnet);
  if (ret == -1) {
    printf("failed to get roadnet proto");
    return false;
  }
  return true;
  */
}

bool OpendriveAdapter::LoadData(const std::string &filename,
                                acu::hdmap::Roadnet *pb_roadnet) {
  ACU_RETURN_IF_NULL(pb_roadnet);

  std::cout<<" OpendriveAdapter::LoadData filename " << filename<<std::endl;
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return false;
  }

  // root node
  const tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);

  if (pb_roadnet == nullptr) return false;

  RoadnetXmlParser roadnet_par;
  int ret = roadnet_par.GetRoadnetProto(*root_node, pb_roadnet);
  if (ret == -1) {
    printf("failed to get roadnet proto\n");
    return false;
  }
  return true;
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu
