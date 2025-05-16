/*
idriverplus
create 20181122 lbh
modify 20190612 pgq
midify 20190712 pgq
=========================================================================*/

#include "util_xml_parser.h"
#include <cmath>

namespace acu {
namespace xmlmap {
namespace adapter {
int Point2Yaw(const PointSet* ps, AngleSet* as) {
  size_t size = ps->size();
  if (size == 0) return 0;
  as->reserve(size);
  double yaw;
  for (int i = 0; i < size - 1; i++) {
    yaw = atan2((*ps)[i + 1].y - (*ps)[i].y, (*ps)[i + 1].x - (*ps)[i].x)*180.0/3.14159;
    as->push_back(yaw);
  }
  if (ps->size() < 2) {
    as->push_back(0);
  } else {
    as->push_back(yaw);
  }
  
}

int UtilXmlParser::ParseRegionBoundary(const tinyxml2::XMLElement& xml_node,
                                       RegionBoundary* region_boundary) {
  ACU_RETURN_IF_NULL(region_boundary);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("exterior_outline");
  ACU_RETURN_IF_ERROR(!sub_node, "xml data missing exterior outline.");
  const tinyxml2::XMLElement* sub_node1 = xml_node.FirstChildElement("interior_outline");
  const tinyxml2::XMLElement* boundary_lines_node = sub_node->FirstChildElement("boundary_line");
  ACU_RETURN_IF_ERROR(!boundary_lines_node, "xml data missing boundary line.");
  BoundaryLines boundary_lines;
  while (boundary_lines_node) {
    BoundaryLine boundary_line;
    std::string id;
    std::string type;
    int order;
    int checker = UtilXmlParser::QueryStringAttribute(*boundary_lines_node, "id", &id);
    checker += UtilXmlParser::QueryStringAttribute(*boundary_lines_node, "type", &type);
    checker += boundary_lines_node->QueryIntAttribute("order", &order);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing boundary line id and order attributes");
    boundary_line.boundary_line_id = id;
    boundary_line.boundary_line_type = type;
    boundary_line.boundary_line_order = order;
    const auto geo_node = boundary_lines_node->FirstChildElement("geometry");
    auto* geometry = &(boundary_line.geometry);
    ACU_RETURN_IF_ERROR(ParseGeometry(*geo_node, geometry),
                        "Error parsing ParseGeometry Geometry");
    boundary_lines.push_back(boundary_line);
    boundary_lines_node = boundary_lines_node->NextSiblingElement("boundary_line");
  }
  region_boundary->exterior_outline = boundary_lines;
  if (sub_node1) {
      InteriorOutlines interior_outlines;
      while (sub_node1) {
        const tinyxml2::XMLElement* boundary_lines_node1 = sub_node1->FirstChildElement("boundary_line");
        ACU_RETURN_IF_ERROR(!boundary_lines_node1, "xml data missing boundary line.");
        BoundaryLines boundary_lines1;
        while (boundary_lines_node1) {
          BoundaryLine boundary_line1;
          std::string id1;
          std::string type1;
          int order1;
          int checker1 = UtilXmlParser::QueryStringAttribute(*boundary_lines_node1, "id", &id1);
          checker1 += UtilXmlParser::QueryStringAttribute(*boundary_lines_node1, "type", &type1);
          checker1 += boundary_lines_node1->QueryIntAttribute("order", &order1);
          ACU_RETURN_IF_ERROR(checker1 != tinyxml2::XML_SUCCESS,
                           "Error parsing boundary line id and order attributes");
          boundary_line1.boundary_line_id = id1;
          boundary_line1.boundary_line_type = type1;
          boundary_line1.boundary_line_order = order1;
          const auto geo_node1 = boundary_lines_node1->FirstChildElement("geometry");
          auto* geometry1 = &(boundary_line1.geometry);
          ACU_RETURN_IF_ERROR(ParseGeometry(*geo_node1, geometry1),
                               "Error parsing ParseGeometry Geometry");
          boundary_lines1.push_back(boundary_line1);
          boundary_lines_node1 = boundary_lines_node1->NextSiblingElement("boundary_line");
        }
        interior_outlines.push_back(boundary_lines1);
        sub_node1 = sub_node1->NextSiblingElement("interior_outline");
      }
      region_boundary->interior_outlines = interior_outlines;
    }
  return 0;
}

int UtilXmlParser::ParseGeometry(const tinyxml2::XMLElement& xml_node,
                                 Geometry* geometry) {
  ACU_RETURN_IF_NULL(geometry);
  double length = 0.0;
  int checker = tinyxml2::XML_SUCCESS;
  checker += xml_node.QueryDoubleAttribute("length", &length);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing geometry length attributes");
  const auto sub_node = xml_node.FirstChildElement("pointSet");
  geometry->length = length;
  if (sub_node) {
    auto* pointset = &geometry->pointset;
    ACU_RETURN_IF_ERROR(ParsePointSet(*sub_node, pointset),
                         "Error parsing ParseGeometry PointSet");
    Point2Yaw(pointset, &geometry->angleset);
    return 0;
  }
  ACU_RETURN_IF_ERROR(1, "Error geometry object");
}
int UtilXmlParser::ParseOutline(const tinyxml2::XMLElement& xml_node,
                                Outline* outline) {
  ACU_RETURN_IF_NULL(outline); 
  double area = 0.0;
  int checker = tinyxml2::XML_SUCCESS;
  checker += xml_node.QueryDoubleAttribute("area", &area);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing outline area attributes");
  outline->area = area;
  const auto sub_node = xml_node.FirstChildElement("pointSet");
  if (sub_node) {
    auto* pointset = &outline->pointset;
    ACU_RETURN_IF_ERROR(ParsePointSet(*sub_node, pointset),
                         "Error parsing ParseOutline PointSet");
    return 0;
  }
  ACU_RETURN_IF_ERROR(1, "Error outline object");
}
int UtilXmlParser::ParseCurbs(const tinyxml2::XMLElement& xml_node,
                              Curbs* curbs) {
  ACU_RETURN_IF_NULL(curbs);
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("trajectory");
  while(sub_node) {
    Curb trajectory;
    int checker = tinyxml2::XML_SUCCESS, sequence = 0;
    std::string id;
    checker += sub_node->QueryIntAttribute("sequence", &sequence);
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);
    trajectory.id = id;
    trajectory.sequence = sequence;
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing trajectory id and sequence");
    const auto geo_node = sub_node->FirstChildElement("geometry");
    auto* geometry = &(trajectory.geometry);
    ACU_RETURN_IF_ERROR(ParseGeometry(*geo_node, geometry),
                          "Error parsing ParseGeometry Geometry");
    curbs->push_back(trajectory);
    sub_node = sub_node->NextSiblingElement("trajectory");
  }
  (*curbs).shrink_to_fit();
  return 0;
}
int UtilXmlParser::ParsePointSet(const tinyxml2::XMLElement& xml_node,
                                 PointSet* pointset) {
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("point");
  while (sub_node) {
    double ptx = 0.0;
    double pty = 0.0;
    double ptz = 0.0;
    int checker = tinyxml2::XML_SUCCESS;
    checker += sub_node->QueryDoubleAttribute("x", &ptx);
    checker += sub_node->QueryDoubleAttribute("y", &pty);
    checker += sub_node->QueryDoubleAttribute("z", &ptz);

    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing geometry point attributes");
    pointset->push_back({ptx, pty, ptz});
    sub_node = sub_node->NextSiblingElement("point");
  }
  (*pointset).shrink_to_fit();

  return 0;
}

int UtilXmlParser::ParsePosition(const tinyxml2::XMLElement& xml_node,
                                 Position* position) {
  ACU_RETURN_IF_NULL(position);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  int checker = tinyxml2::XML_SUCCESS;
  checker += xml_node.QueryDoubleAttribute("roll", &roll);
  checker += xml_node.QueryDoubleAttribute("pitch", &pitch);
  checker += xml_node.QueryDoubleAttribute("yaw", &yaw);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing position attributes");
  position->roll = roll;
  position->pitch = pitch;
  position->yaw = yaw;
  auto* point = &position->center_point;
  ACU_RETURN_IF_ERROR(ParsePoint(xml_node, point),
                       "Error parsing position point");
  return 0;
}

int UtilXmlParser::ParseOrigin(const tinyxml2::XMLElement& xml_node,
                               Origin* origin) {
  ACU_RETURN_IF_NULL(origin);
  int zone;
  int multpcd_enable;
  int checker = tinyxml2::XML_SUCCESS;
  checker += xml_node.QueryIntAttribute("zone", &zone);
  checker += xml_node.QueryIntAttribute("multpcd_enable", &multpcd_enable);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing origin attributes");
  origin->zone = zone;
  origin->multpcd_enable = multpcd_enable;
  auto* point = &origin->center_point;
  ACU_RETURN_IF_ERROR(ParsePoint(xml_node, point),
                       "Error parsing origin point");
  return 0;
}

int UtilXmlParser::ParseEdges(const tinyxml2::XMLElement& xml_node,
                                Edges* edges) {
  // const auto edge_node = xml_node.FirstChildElement("edges");
  // ACU_RETURN_IF_NULL(edges);
  std::string origin_voronoi;
  int checker = tinyxml2::XML_SUCCESS;
  checker += UtilXmlParser::QueryStringAttribute(xml_node, "origin_voronoi", &origin_voronoi);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parsing edge attributes");
  edges->id = origin_voronoi;

  auto* connect_ids = &edges->connect_ids; 
  
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("connect_voronoi");
  while (sub_node) {
    std::string id;
    int checker = tinyxml2::XML_SUCCESS;
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "id", &id);

    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing connect_voronoi attributes");
    connect_ids->push_back(id);
    sub_node = sub_node->NextSiblingElement("connect_voronoi");
  }
  (*connect_ids).shrink_to_fit();

  return 0;
}

int UtilXmlParser::ParseBoundary(const tinyxml2::XMLElement& xml_node,
        acu::xmlmap::Boundary* boundary)
{
  ACU_RETURN_IF_NULL(boundary);
  const auto sub_node = xml_node.FirstChildElement("boundary");
  ACU_CHECK(sub_node != nullptr);
  int checker = tinyxml2::XML_SUCCESS;
  double xg = 0.0;
  double yg = 0.0;
  double length = 0.0;
  double width = 0.0;
  checker += sub_node->QueryDoubleAttribute("x", &xg);
  checker += sub_node->QueryDoubleAttribute("y", &yg);
  checker += sub_node->QueryDoubleAttribute("length", &length);
  checker += sub_node->QueryDoubleAttribute("width", &width);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parse boundary attributes");
  boundary->xg = xg;
  boundary->yg = yg;
  boundary->length = length;
  boundary->width = width;

  // std::cout << "xg=" << xg << ", yg=" << yg << std::endl;
  // std::cout << "length=" << length << ", width=" << width << std::endl;
  return 0;
}

int UtilXmlParser::ParseNameSet(const tinyxml2::XMLElement& xml_node,
                                Nameset* nameset) {
  const tinyxml2::XMLElement* sub_node = xml_node.FirstChildElement("name");
  while (sub_node) {
    std::string x;
    int checker = tinyxml2::XML_SUCCESS;
    checker += UtilXmlParser::QueryStringAttribute(*sub_node, "x", &x);
    ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                         "Error parsing name attributes");
    nameset->push_back(x);
    sub_node = sub_node->NextSiblingElement("name");
  }
  (*nameset).shrink_to_fit();

  return 0;
}

int UtilXmlParser::ParsePoint(const tinyxml2::XMLElement& xml_node, Point* pt) {
  ACU_RETURN_IF_NULL(pt);
  const auto sub_node = xml_node.FirstChildElement("centerPoint");
  ACU_CHECK(sub_node != nullptr);
  int checker = tinyxml2::XML_SUCCESS;
  double ptx = 0.0;
  double pty = 0.0;
  double ptz = 0.0;
  checker += sub_node->QueryDoubleAttribute("x", &ptx);
  checker += sub_node->QueryDoubleAttribute("y", &pty);
  checker += sub_node->QueryDoubleAttribute("z", &ptz);
  ACU_RETURN_IF_ERROR(checker != tinyxml2::XML_SUCCESS,
                       "Error parse point attributes");
  pt->x = ptx;
  pt->y = pty;
  pt->z = ptz;
  return 0;
}


tinyxml2::XMLError UtilXmlParser::QueryStringAttribute(
    const tinyxml2::XMLElement& xml_node, const std::string& name,
    std::string* value) {
  if (value == nullptr) return tinyxml2::XML_ERROR_PARSING;
  const char* val = xml_node.Attribute(name.c_str());
  if (val == nullptr) {
    return tinyxml2::XML_NO_ATTRIBUTE;
  }
  *value = val;
  return tinyxml2::XML_SUCCESS;
}

}
}
}
