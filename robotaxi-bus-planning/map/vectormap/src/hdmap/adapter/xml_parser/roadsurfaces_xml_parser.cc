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
#include <iomanip>
#include "map/vectormap/src/hdmap/adapter/xml_parser/roadsurfaces_xml_parser.h"

namespace acu {
namespace hdmap {
namespace adapter {

using acu::common::math::Vec2d;

Status RoadSurfaceXmlParser::Parse(const tinyxml2::XMLElement& xml_node,
                                 std::vector<RoadSurfaceInternal>* roadsurfaces) {
  const tinyxml2::XMLElement* roadsurface_node =
      xml_node.FirstChildElement("road_surface");
  while (roadsurface_node) {
    // id, type
    std::string roadsurface_id;
    std::string roadsurface_type;
    int checker = UtilXmlParser::QueryStringAttribute(*roadsurface_node, "id", &roadsurface_id);
    checker += UtilXmlParser::QueryStringAttribute(*roadsurface_node, "type", &roadsurface_type);
    if (checker != tinyxml2::XML_SUCCESS) {
      std::string err_msg = "Error parse roadsurface id and type";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    // outline
    const tinyxml2::XMLElement* sub_node =
        roadsurface_node->FirstChildElement("outline");
    if (!sub_node) {
      std::string err_msg = "Error parse roadsurface outline";
      return Status(acu::common::ErrorCode::HDMAP_DATA_ERROR, err_msg);
    }

    RoadSurfaceInternal roadsurface;
    roadsurface.id = roadsurface_id;

    if (roadsurface_type == "isolation") {
      PbIsolationbelt isolationbelt;
      isolationbelt.mutable_id()->set_id(roadsurface_id);
      PbPolygon* polygon = isolationbelt.mutable_polygon();
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
      CheckPolygonValid(polygon);
      roadsurface.isolationbelts.push_back(isolationbelt);
    } else if (roadsurface_type == "guardrail") {
      PbGuardrail guardrail;
      guardrail.mutable_id()->set_id(roadsurface_id);
      PbPolygon* polygon = guardrail.mutable_polygon();
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
      CheckPolygonValid(polygon);
      roadsurface.guardrails.push_back(guardrail);
    } else if (roadsurface_type == "inner") {
      PbInner inner;
      inner.mutable_id()->set_id(roadsurface_id);
      PbPolygon* polygon = inner.mutable_polygon();
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
      CheckPolygonValid(polygon);
      roadsurface.inners.push_back(inner);
    } else if (roadsurface_type == "outer") {
      PbOuter outer;
      outer.mutable_id()->set_id(roadsurface_id);
      PbPolygon* polygon = outer.mutable_polygon();
      RETURN_IF_ERROR(UtilXmlParser::ParseOutline(*sub_node, polygon));
      CheckPolygonValid(polygon);
      roadsurface.outers.push_back(outer);
    }

    roadsurfaces->push_back(roadsurface);
    roadsurface_node = roadsurface_node->NextSiblingElement("road_surface");
  }
  return Status::OK();
}

//两个点的polygon,插个中点; 校验面积
void RoadSurfaceXmlParser::CheckPolygonValid(PbPolygon* polygon) {
  //两个点的polygon,插个中点
  if (polygon->point_size() == 2) {
    double mid_xg = 0.0;
    double mid_yg = 0.0;
    double mid_zg = 0.0;
    for (const auto &point : polygon->point()) {
      mid_xg += point.x();
      mid_yg += point.y();
      mid_zg += point.z();
    }
    mid_xg = mid_xg / 2 - 0.1;
    mid_yg = mid_yg / 2 - 0.1;
    mid_zg = mid_zg / 2;

    PbPoint3D* pt = polygon->add_point();
    pt->set_x(mid_xg);
    pt->set_y(mid_yg);
    pt->set_z(mid_zg);
  }

  //校验面积
  std::vector<Vec2d> points;
  for (const auto &point : polygon->point()) {
    points.emplace_back(point.x(), point.y());
  }

  double area = 0.0;
  double points_num = points.size();
  for (int i = 1; i < points_num; ++i) {
    area += CrossProd(points[0], points[i - 1], points[i]);
  }
  if (fabs(area < 0.01)) {
    double first_xg = 0.0;
    double first_yg = 0.0;
    first_xg = points.at(0).x() + 0.1;
    first_yg = points.at(0).y();
    PbPoint3D* pt_first = polygon->add_point();
    pt_first->set_x(first_xg);
    pt_first->set_y(first_yg);
    pt_first->set_z(0.0);

    double second_xg = 0.0;
    double second_yg = 0.0;
    second_xg = points.at(points_num - 1).x();
    second_yg = points.at(points_num - 1).y() - 0.1;
    PbPoint3D* pt_second = polygon->add_point();
    pt_second->set_x(second_xg);
    pt_second->set_y(second_yg);
    pt_second->set_z(0.0);
  }
}

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu


