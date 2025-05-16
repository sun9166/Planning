#ifndef COMMON_TOOLBOX_MATH_INCLUDE_DISTANCE_TRANSFORM_H_
#define COMMON_TOOLBOX_MATH_INCLUDE_DISTANCE_TRANSFORM_H_

#include <bitset>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <array>

#include "common/toolbox/geometry/include/geoheader.h"
#include <time.h>
#include "common/toolbox/geometry/include/dwa_header.h"
// #include "src/application/architecture/datapool/include/common_typedef.h" 
#include "common/base/log/include/log.h"

// using namespace acu::vectormap;
using namespace acu::dwa;
// using namespace acu::planning;
using geometry::Site;
using geometry::SiteVec;
#define CHECK_FREE_SPACE false

struct GridID {
  GridID()
    : gx(0), gy(0) {}
  GridID(int gx_, int gy_)
    : gx(gx_), gy(gy_) {}

  int gx;
  int gy;

  bool operator== (const GridID& grid_id) {
    bool flag = false;
    flag = (gx == grid_id.gx) && (gy == grid_id.gy);
    return flag;
  }

  GridID& operator= (const GridID& grid_id) {
    this->gx = grid_id.gx;
    this->gy = grid_id.gy;
    return *this;
  }
};

class distance_transform {
 public:
  distance_transform();//double gp = 0.1, double gw = 4.0, double gf = 8.0, double gb = 1.0);
  ~distance_transform();
  void GenerateGridMap(SortedTree& nearestobs);
  double ReadDistanceFromMap(const Site& point);

 private:
  bool LoadBaseMap(const double precision, const double f_length,
                   const double b_length, const double half_width, cv::Mat& basemap);
  bool ModifyObject(SortedTree& nearestobs, cv::Mat& obj_map);
  bool DistanceTransform(const cv::Mat& grid_image, cv::Mat& dis_image);
  GridID GridIDIndex(const double xg, const double yg, const float precision);
  std::array<GridID, 8> GridIDIndex8(const GridID& center_grid);

 private:
  float BASEMAP_GRID_PRECISION = 0.05;
  float GRID_PRECISION;
  float GRID_WIDTH_HALF;
  float GRID_FRONT;
  float GRID_BACK;
  double move_origin_x_;
  double move_origin_y_;


  std::pair<int, int> rows_bound_;//yg
  std::pair<int, int> cols_bound_;//xg
  int map_rows_;
  int map_cols_;
  int colli_start_;
  int colli_end_;
  cv::Mat dismap_;

};

#endif
