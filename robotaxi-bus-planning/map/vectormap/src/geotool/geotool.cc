/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: geotool.cc
*
* Description: fool ACU geotool API like USB

*
* History:
* lbh         2018/05/21    1.0.0    build this module.
******************************************************************************/
#include "map/vectormap/include/geotool.h"
#include "proj_api.h"

namespace acu {
namespace vectormap {

int GetLongZone(double longitude) {
  double longZone = 0.0;
  if (longitude < 0.0) {
    longZone = ((180.0 + longitude) / 6.0) + 1;
  } else {
    longZone = (longitude / 6.0) + 31;
  }
  return static_cast<int>(longZone);
}
double Angle2Heading (double input) {
  double output = 90 - input;
  if (output < 0)
    output += 360;
  return output;
}
double Heading2Angle (double input) {
  double output = 90 - input;
  if (output < -180)
    output += 360;
  return output;
}
int GeoTransform(const PointGCCS &pgccs, const int utm_zone, PointGPS &pgps) {
  RETURN_EQ(const_cast<PointGCCS &>(pgccs).Check(), 0);
  double x = pgccs.xg, y = pgccs.yg;
  double z = 0.;
  std::string utmInit = "+proj=utm +zone=" + std::to_string(utm_zone) +
                        "+ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  projPJ lonlat = pj_init_plus(" +proj=longlat +datum=WGS84 +no_defs");
  projPJ utm = pj_init_plus(utmInit.c_str());
  RETURN_NE(pj_transform(utm, lonlat, 1, 1, &x, &y, &z), 0);
  x *= RAD_TO_DEG;
  y *= RAD_TO_DEG;
  pgps.lon = x;
  pgps.lat = y;
  pgps.heading = Angle2Heading(pgccs.angle);
  if (lonlat) {
    pj_free(lonlat);
  }
  if (utm) {
    pj_free(utm);
  }
  return 0;
}
int GeoTransform(const PointGPS &pgps, PointGCCS &pgccs) {
  RETURN_EQ(const_cast<PointGPS &>(pgps).Check(), 0);
  double lon = pgps.lon;
  double lat = pgps.lat;
  double height = 0.;
  std::string utmInit = "+proj=utm +zone=" + std::to_string(GetLongZone(lon)) +
                        "+ellps=WGS84 +datum=WGS84 +units=m +no_defs";
  projPJ lonlat = pj_init_plus(" +proj=longlat +datum=WGS84 +no_defs");
  projPJ utm = pj_init_plus(utmInit.c_str());
  lon *= DEG_TO_RAD;
  lat *= DEG_TO_RAD;
  RETURN_NE(pj_transform(lonlat, utm, 1, 1, &lon, &lat, &height), 0);
  pgccs.xg = lon;
  pgccs.yg = lat;
  pgccs.angle = Heading2Angle(pgps.heading);
  if (lonlat) {
    pj_free(lonlat);
  }
  if (utm) {
    pj_free(utm);
  }
  return 0;
}

int GeoTransform(const PointGCCS &pgccs, const double cell_size,
                 PointGICS &pgics) {
  RETURN_EQ(const_cast<PointGCCS &>(pgccs).Check(), 0);
  pgics.ug = static_cast<long>(pgccs.xg / cell_size);
  pgics.vg = static_cast<long>(pgccs.yg / cell_size);
  return 0;
}
int GeoTransform(const PointGICS &pgics, const double cell_size,
                 PointGCCS &pgccs) {
  RETURN_EQ(const_cast<PointGICS &>(pgics).Check(), 0);
  pgccs.xg = static_cast<double>(pgics.ug * cell_size);
  pgccs.yg = static_cast<double>(pgics.vg * cell_size);
  return 0;
}
int GeoTransform(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
                 PointVCS &output_pvcs) {
  // RETURN_EQ(const_cast<PointGCCS &>(car_pgccs).Check(), 0);
  // RETURN_EQ(const_cast<PointGCCS &>(target_pgccs).Check(), 0);
  double rad = car_pgccs.angle * DEG_TO_RAD;
  double dx = target_pgccs.xg - car_pgccs.xg;
  double dy = target_pgccs.yg - car_pgccs.yg;
  output_pvcs.x = +dx * cos(rad) + dy * sin(rad);
  output_pvcs.y = -dx * sin(rad) + dy * cos(rad);
  output_pvcs.angle = target_pgccs.angle - car_pgccs.angle;
  if (output_pvcs.angle >= 360) output_pvcs.angle -= 360;
  return 0;
}

int GeoTransform(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                 PointGCCS &output_pgccs) {
  // RETURN_EQ(const_cast<PointGCCS &>(car_pgccs).Check(), 0);
  double rad = car_pgccs.angle * DEG_TO_RAD;
  output_pgccs.xg = target_pvcs.x * cos(rad) - target_pvcs.y * sin(rad);
  output_pgccs.yg = target_pvcs.x * sin(rad) + target_pvcs.y * cos(rad);
  output_pgccs.xg += car_pgccs.xg;
  output_pgccs.yg += car_pgccs.yg;
  output_pgccs.angle = car_pgccs.angle + target_pvcs.angle;
  if (output_pgccs.angle >= 360) output_pgccs.angle -= 360;
  return 0;
}

bool GeoTool::SetUtmZone(int zone) {
  if (zone < 0 || zone > 60)
    return 0;
  else {
    utm_zone_ = zone;
    return 1;
  }
}
bool GeoTool::SetCellSize(double size) {
  if (size <= 0)
    return 0;
  else {
    cell_size_ = size;
    return 1;
  }
}

int GeoTool::GCCS2GPS(const PointGCCS &pgccs, PointGPS &pgps) const {
  return GeoTransform(pgccs, utm_zone_, pgps);
}
int GeoTool::GPS2GCCS(const PointGPS &pgps, PointGCCS &pgccs) const {
  return GeoTransform(pgps, pgccs);
}

int GeoTool::GCCS2GICS(const PointGCCS &pgccs, PointGICS &pgics) const {
  return GeoTransform(pgccs, cell_size_, pgics);
}
int GeoTool::GICS2GCCS(const PointGICS &pgics, PointGCCS &pgccs) const {
  return GeoTransform(pgics, cell_size_, pgccs);
}

int GeoTool::GCCS2VCS(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
                      PointVCS &output_pvcs) const {
  return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
}
int GeoTool::VCS2GCCS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                      PointGCCS &output_pgccs) const {
  return GeoTransform(car_pgccs, target_pvcs, output_pgccs);
}

int GeoTool::GICS2VCS(const PointGCCS &car_pgccs, const PointGICS &target_pgics,
                      PointVCS &output_pvcs) const {
  PointGCCS target_pgccs = {0.0, 0.0, 0.0, 0.0};
  RETURN_EQ(GeoTransform(target_pgics, cell_size_, target_pgccs), -1);
  return GeoTransform(car_pgccs, target_pgccs, output_pvcs);
}
int GeoTool::VCS2GICS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                      PointGICS &output_pgics) const {
  PointGCCS output_pgccs = {0.0, 0.0, 0.0, 0.0};
  RETURN_EQ(GeoTransform(car_pgccs, target_pvcs, output_pgccs), -1);
  return GeoTransform(output_pgccs, cell_size_, output_pgics);
}
}  // namespace map
}  // namespace acu