/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: geotool.h
*
* Description: fool ACU geotool API like USB

*
* History:
* lbh         2018/05/21    1.0.0    build this module.
******************************************************************************/
#ifndef MAP_GEOTOOL_
#define MAP_GEOTOOL_

#include <iostream>

/**
*@brief macros using by check functions in follow structs about point
*/
#define RETURN_EQ(val1, val2) \
  if ((val1) == (val2)) return -1;
#define RETURN_NE(val1, val2) \
  if ((val1) != (val2)) return -1;

#define BOOL_GE(val1, val2) ((val1) >= (val2)) ? 1 : 0

#define BOOL_IN(val1, val2, val3) \
  (((val1) >= (val2)) && ((val1) <= (val3))) ? 1 : 0

namespace acu {
namespace vectormap {
/**
*@brief the min and max values of xg yg in PointGCCS
*/
constexpr double kXGMIN = 166021.443081, kXGMAX = 833978.556908, kYGMIN = -8881585.81599, kYGMAX = 9328093.83056;
/**
*@brief the min and max values of lon lat in PointGPS
*/
constexpr double kLONMIN = -180, kLONMAX = 180, kLATMIN = -80, kLATMAX = 84;
/**
*@brief define the input and output points using in the API
*@func Check() check the members in point structs
*/
struct PointGCCS {
  double xg;
  double yg;
  double angle;
  double zg;
  void Set(double _xg, double _yg, double _angle) {
    xg = _xg;
    yg = _yg;
    angle = _angle;
  }
  
  bool Check() {
    bool b1 = BOOL_IN(xg, kXGMIN, kXGMAX);
    bool b2 = BOOL_IN(yg, kYGMIN, kYGMAX);
    return b1 && b2;
  }
};
struct PointVCS {
  double x;
  double y;
  double angle;
};
struct PointGICS {
  explicit PointGICS(long ug_, long vg_)
    :ug(ug_), vg(vg_) {} 

  explicit PointGICS() {
    ug = 0.0;
    vg = 0.0;
  }
  long ug;
  long vg;
  bool Check() {
    bool b1 = BOOL_GE(ug, 0);
    bool b2 = BOOL_GE(vg, 0);
    return b1 && b2;
  }
};
struct PointGPS {
  explicit PointGPS(double longitude_, double latitude_, double heading_)
    :lon(longitude_), lat(latitude_), heading(heading_) {} 

  explicit PointGPS() {
    lon = 0.0;
    lat = 0.0;
    heading = 0.0;
  }
  double lon;
  double lat;
  double heading;
  bool Check() {
    bool b1 = BOOL_IN(lon, kLONMIN, kLONMAX);
    bool b2 = BOOL_IN(lat, kLATMIN, kLATMAX);
    return b1 && b2;
  }
};

/**
* @functions GeoTransform
* @brief transform between gccs and gps/gics/vcs,
*/

/**
*@brief transform between gccs and gps,
*@param pgccs input gccs point
*@param utm_zone the zone of input point
*@param pgps output gps point
*@return -1: failed, 0: success
*/
int GeoTransform(const PointGCCS &pgccs, const int utm_zone, PointGPS &pgps);
int GeoTransform(const PointGPS &pgps, PointGCCS &pgccs);

/**
*@brief transform between gccs and gics,
*@param pgccs input gccs point
*@param cell_size the size of cell
*@param pgics output gics point
*@return -1: failed, 0: success
*/
int GeoTransform(const PointGCCS &pgccs, const double cell_size,
                 PointGICS &pgics);
int GeoTransform(const PointGICS &pgics, const double cell_size,
                 PointGCCS &pgccs);

/**
*@brief transform between gccs and vcs,
*@param car_pgccs input gccs point where the car is
*@param target_pgccs input gccs point where the target is
*@param output_pvcs output vcs point
*@return -1: failed, 0: success
*/
int GeoTransform(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
                 PointVCS &output_pvcs);
int GeoTransform(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
                 PointGCCS &output_pgccs);

int GetLongZone(double longitude);
double Angle2Heading (double input);
double Heading2Angle (double input);

class GeoTool {
 public:
  GeoTool() : cell_size_(0.05), utm_zone_(50) {}
  GeoTool(int zone, double size) : cell_size_(size), utm_zone_(zone) {}
  /**
  *@brief set members cell_size_ utm_zone_.
  *@return -1: failed, 0: success
  */
  bool SetCellSize(double size);
  bool SetUtmZone(int zone);

 public:
  /**
  *@brief transform between gccs and gps,
  *@param pgccs input gccs point
  *@param utm_zone the zone of input point
  *@param pgps output gps point
  *@return -1: failed, 0: success
  */
  int GCCS2GPS(const PointGCCS &pgccs, PointGPS &pgps) const;
  int GPS2GCCS(const PointGPS &pgps, PointGCCS &pgccs) const;
  /**
  *@brief transform between gccs and gics,
  *@param pgccs input gccs point
  *@param cell_size the size of cell
  *@param pgics output gics point
  *@return -1: failed, 0: success
  */
  int GCCS2GICS(const PointGCCS &pgccs, PointGICS &pgics) const;
  int GICS2GCCS(const PointGICS &pgics, PointGCCS &pgccs) const;
  /**
  *@brief transform between gccs and vcs,
  *@param car_pgccs input gccs point where the car is
  *@param target_pgccs input gccs point where the target is
  *@param output_pvcs output vcs point
  *@return -1: failed, 0: success
  */
  int GCCS2VCS(const PointGCCS &car_pgccs, const PointGCCS &target_pgccs,
               PointVCS &output_pvcs) const;
  int VCS2GCCS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
               PointGCCS &output_pgccs) const;
  /**
  *@brief transform between gics and vcs,
  *@param car_pgccs input gccs point where the car is
  *@param target_pgics input gics point where the target is
  *@param output_pvcs output vcs point
  *@return -1: failed, 0: success
  */
  int GICS2VCS(const PointGCCS &car_pgccs, const PointGICS &target_pgics,
               PointVCS &output_pvcs) const;
  int VCS2GICS(const PointGCCS &car_pgccs, const PointVCS &target_pvcs,
               PointGICS &output_pgics) const;

 private:
  double cell_size_;
  int utm_zone_;
};
}  // namespace map
}  // namespace avos
#endif