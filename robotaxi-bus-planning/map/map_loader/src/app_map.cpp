
#include "map/map_loader/include/app_map.h"
#include "tinyxml2.h"
#include "common/base/log/include/log.h"

namespace acu {
namespace map {

std::vector<FunctionPoint> AppMap::points_;
std::vector<FunctionArea> AppMap::areas_;

bool AppMap::isFileExist(const std::string file_name) {
  if (file_name == "")
    return false;
  if (access(file_name.c_str(), F_OK) == 0)
    return true;
  return false;
}

int AppMap::LoadMapFromFile(const std::string &map_filename)
{
    if (!isFileExist(map_filename)) {
        AERROR << map_filename << " isn't exist.";
        return false;
    }

    tinyxml2::XMLDocument document;
    if (document.LoadFile(map_filename.c_str()) != tinyxml2::XML_SUCCESS) {
        AERROR << "Failed to load " << map_filename;
        return false;
    }

    this->points_.clear();
    this->areas_.clear();

    tinyxml2::XMLElement *root_node = document.RootElement();
    if (root_node == nullptr) {
        return true;
    }

    tinyxml2::XMLElement *node;

    node = root_node->FirstChildElement();
    std::string type;
    while (node != nullptr) {
        type = node->Attribute("type");
        if (type == "area") {
            tinyxml2::XMLElement *poly_node = node->FirstChildElement();
            while (poly_node != nullptr) {
                FunctionArea func_area;
                func_area.polygon.points.clear();
                func_area.name = poly_node->Attribute("name");
                func_area.type = poly_node->Attribute("type");
                func_area.signal = poly_node->Attribute("signal");
                func_area.canid = poly_node->Attribute("canid");
                func_area.bit = poly_node->Attribute("bit");

                tinyxml2::XMLElement *point_node = poly_node->FirstChildElement();
                while (point_node != nullptr) {
                    geometry_msgs::Point32 point;
                    point.x = strtod(point_node->Attribute("x"), nullptr);
                    point.y = strtod(point_node->Attribute("y"), nullptr);
                    func_area.polygon.points.push_back(point);

                    point_node = point_node->NextSiblingElement();
                }

                this->areas_.push_back(func_area);

                poly_node = poly_node->NextSiblingElement();
            }
        } else if (type == "point") {
            FunctionPoint func_point;
            tinyxml2::XMLElement *point_node = node->FirstChildElement();

            while (point_node != nullptr) {
                func_point.name = point_node->Attribute("name");
                type = point_node->Attribute("type");
                func_point.type = type;

                func_point.point.x = strtod(point_node->Attribute("x"), nullptr);
                func_point.point.y = strtod(point_node->Attribute("y"), nullptr);
                func_point.point.z = 0.0;
                func_point.heading = strtod(point_node->Attribute("heading"), nullptr);
                this->points_.push_back(func_point);

                point_node = point_node->NextSiblingElement();
            }
        }

        node = node->NextSiblingElement();
    }

    return true;
}

std::vector<FunctionArea> AppMap::GetFunctionAreaByPoint(geometry_msgs::Point32 point)
{
    std::vector<FunctionArea> area_vec;
    area_vec.clear();

    for (auto area_item : areas_) {
        if (IsInPolygon(area_item, point)) {
            area_vec.push_back(area_item);
        }
    }

    return area_vec;
}

std::vector<FunctionPoint>& AppMap::GetFunctionPoints()
{
    return points_;
}

std::vector<FunctionArea>& AppMap::GetFunctionAreas()
{
    return areas_;
}

bool AppMap::IsInPolygon(FunctionArea area, geometry_msgs::Point32 point)
{
    int count = area.polygon.points.size();

    if(count < 3) {
        return false;
    }

    bool result = false;

    for(int i = 0, j = count - 1; i < count; i++) {
        auto p1 = area.polygon.points[i];
        auto p2 = area.polygon.points[j];

        if(p1.x < point.x && p2.x >= point.x || p2.x < point.x && p1.x >= point.x) {
            if(p1.y + (point.x - p1.x) / (p2.x - p1.x) * (p2.y - p1.y) < point.y) {
                result = !result;
            }
        }
        j = i;
    }
    return result;
}

}  // namespace map
}  // namespace acu
