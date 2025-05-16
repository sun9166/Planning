#ifndef __SHOW_VECTORMAP_H__
#define __SHOW_VECTORMAP_H__

#include <iostream>
#include <string>
#include "tinyxml2.h"
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include "common/base/log/include/log.h"
#include "map/vectormap/include/geotool.h"

using namespace std;
using namespace tinyxml2;
using namespace acu::vectormap;

typedef struct PointGps
{
  double lon;
  double lat;
  double x;
  double y;

  void convert();
  void pointOff(double xg, double yg);
  // void GetXgYg(double _x, double _y);
} PointGps;

typedef struct MissionLaneInfo
{
  std::vector<PointGCCS> gccs_point;
  double length;
  std::string x;
  std::string y;
  std::string lane_id;
  std::string road_id;
} MissionLaneInfo;


typedef struct Mission_RoadInfo
{
  std::vector<std::string> lanes;

} Mission_RoadInfo;

extern vector<std::map<std::string , std::vector<std::string>>*> groad_info_map_vec;
extern vector<std::map<std::string, MissionLaneInfo>*> glane_info_map_vec;

int LoadVectorFile(PointGCCS &car_gccs);


#endif // __SHOW_VECTORMAP_H__
