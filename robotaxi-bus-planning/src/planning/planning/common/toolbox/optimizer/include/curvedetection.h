#ifndef COMMON_TOOLBOX_OPTIMIZER_INCLUDE_CURVEDETECTION_H_
#define COMMON_TOOLBOX_OPTIMIZER_INCLUDE_CURVEDETECTION_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <limits>

#include "common/toolbox/geometry/include/geoheader.h"
#include <gtest/gtest.h>
namespace optimizer {

class CurveDetection {
 public:
  CurveDetection();
  ~CurveDetection() = default;
  double RunDetection(geometry::SiteVec& c_path, int pathdirection = 0);
  double GlobalCurveFitting(geometry::SiteVec& c_path, int pathdirection);
 private:
  double GetFitLength(int direction);
  bool GetTotalLength(geometry::SiteVec& path, std::vector<double> &length_vec, bool use_global = false);

  double PointCheck(geometry::SiteVec& path);
 private:
  int fit_curve_smoothlen_;
  int current_path_direction_;
  double fit_curve_len;
  double fit_curve_reverse_len;
  double curve_end_;
  double fit_curve_minlen_;
  double total_length_;
  double curve_limit_;
  double curve_limit_max_;
private:
  FRIEND_TEST(curvedetectionTest, MeanFilterNullPath);
};

}  // optimizer
#endif