/*
idriverplus
create 20181122 lbh
midify 20190712 pgq
=========================================================================*/
#include "xmlmap_adapter.h"
namespace acu {
namespace xmlmap {
namespace adapter {

int XmlMapAdapter::LoadData(const std::string &filename, Vectormap *vectormap) {
  ACU_RETURN_IF_NULL(vectormap);
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return -1;
  }
  if (vectormap == nullptr) return -1;
  // root node
  const tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);

  // header
  auto *vectormap_header = &vectormap->header;
  int ret = HeaderXmlParser::Parse(*root_node, vectormap_header);
  ACU_RETURN_IF_ERROR((ret == -1), "fail to parse vectormap header.");

  // roads
  auto *vectormap_roads = &vectormap->roads;
  ret = RoadsXmlParser::Parse(*root_node, vectormap_roads);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap roads.");

  //surfaces
  auto *vectormap_surfaces = &vectormap->surfaces;
  ret = SurfacesXmlParser::Parse(*root_node, vectormap_surfaces);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap surfaces.");

  // regions
  auto *vectormap_regions = &vectormap->regions;
  ret = RegionsXmlParser::Parse(*root_node, vectormap_regions);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap regions.");

  // boundaries
  auto *vectormap_boundaries = &vectormap->boundaries;
  ret = BoundariesXmlParser::Parse(*root_node, vectormap_boundaries);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap boundaries.");

  // points
  auto *vectormap_points = &vectormap->points;
  ret = PointsXmlParser::Parse(*root_node, vectormap_points);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap points.");

  // jobs
  auto *vectormap_jobs = &vectormap->jobs;
  ret = JobsXmlParser::Parse(*root_node, vectormap_jobs);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap jobs.");

  // pathPoint
  auto *vectormap_pathPoint = &vectormap->pathpoint;
  ret = LocationXmlParser::Parse(*root_node, vectormap_pathPoint);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap pathpoint.");

  // topo
  auto *vectormap_topo = &vectormap->topology;
  ret = TopoXmlParser::Parse(*root_node, vectormap_topo);
  ACU_RETURN_IF_ERROR((ret == -1), "failed to parse vectormap topo.");

  return 0;
}

int XmlMapAdapter::LoadData(const std::string &filename,
                            MapinfoXml *mapinfoxml) {
  ACU_RETURN_IF_NULL(mapinfoxml);
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return -1;
  }
  if (mapinfoxml == nullptr) return -1;
  // root node
  const tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);

  // header
  auto *mapinfoxml_header = &mapinfoxml->header;
  int ret = MapinfoXmlParser::Parse(*root_node, mapinfoxml_header);
  ACU_RETURN_IF_ERROR((ret == -1), "fail to parse mapinfoxml header.");

  return 0;
}

int XmlMapAdapter::LoadData(const std::string &filename, MapXml *mapxml) {
  ACU_RETURN_IF_NULL(mapxml);
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return -1;
  }
  if (mapxml == nullptr) return -1;
  // root node
  const tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);

  // header
  auto *mapxml_header = &mapxml->header;
  int ret = MapXmlParser::Parse(*root_node, mapxml_header);
  ACU_RETURN_IF_ERROR((ret == -1), "fail to parse mapxml header.");

  return 0;
}
int XmlMapAdapter::SetMapHeader(const std::string &filename,
                                const std::string &default_name) {
  tinyxml2::XMLDocument document;
  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to load file " << filename<<std::endl;
    return -1;
  }
  tinyxml2::XMLElement *root_node = document.RootElement();
  ACU_CHECK(root_node != nullptr);
  auto sub_node = root_node->FirstChildElement("header");
  ACU_RETURN_IF_ERROR(!sub_node, "xml data missing mheader.");
  sub_node->SetAttribute("default_name", default_name.c_str());
  auto re = document.SaveFile(filename.c_str());
  if (re != tinyxml2::XML_SUCCESS) {
    std::cout<<"fail to SaveFile " << filename<<std::endl;
    return -1;
  }
  return 0;
}
}  // namespace adapter
}  // namespace xmlmap
}  // namespace acu
