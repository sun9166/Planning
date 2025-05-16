
#ifndef COMMON_TOOLBOX_OPTIMIZER_INCLUDE_MEANFILT_H_
#define COMMON_TOOLBOX_OPTIMIZER_INCLUDE_MEANFILT_H_
#include "common/toolbox/geometry/include/geoheader.h"
#include <utility>
#include <iostream>
#include <gtest/gtest.h>
// #include <cmath>
// #include "adapter.h"
// #include <fstream>

using geometry::Site;
using geometry::SiteVec;
using geometry::SiteList;

namespace toolbox {
namespace optimizer {
#define M_PII 3.141592653 
struct tempPath{
    Site Point;
    std::pair<double, double> dis_l_r;
  };
class MeanFilt {
 public:
  MeanFilt() {
    iner_path_.clear();
    update_path_.clear();
  }
  ~MeanFilt() = default;

 public:
  void Interpolation(SiteVec &to_smooth_path, double density = 0.10);
  void Filt(SiteVec &anchors, double density = 0.10);
  void FiltList(SiteList &path, double density = 0.05);
  // SiteVec FiltSmooth(std::vector<tempPath> &smooth_path, double density = 0.05);
  SiteVec FiltSmooth(std::vector<tempPath> &smooth_path, SiteVec smoothPoints, double density = 0.05);
  std::pair<double, double> xy_limit(tempPath &pointInf);
  bool nodeAfterSmooth(tempPath &pointInf, const Site &beforeResult, Site &result);
  void gccs2vcs(const Site &beforeResultGCCS, const Site &resultFinalGCCS, Site &vcs);
  void vcs2gccs(const Site &beforeResultGCCS, const Site &resultFinalVCS, Site &gccs);
  bool angleFilter(SiteVec &outPoint);
  double Normalize(double angle);

 private:
  SiteVec iner_path_;
  SiteVec update_path_;
 private:
  FRIEND_TEST(meanfiltTest, NullData);
  FRIEND_TEST(meanfiltTest, DensePoints);
  FRIEND_TEST(meanfiltTest, SparsePoints);
  FRIEND_TEST(meanfiltTest, ZerolDensity);
  FRIEND_TEST(meanfiltTest, NormalPointsLargeDensity);
  FRIEND_TEST(meanfiltTest, DensePointsAndSparsePoints);
  FRIEND_TEST(meanfiltTest, SparsePointsAndDensePoints);
  FRIEND_TEST(meanfiltTest, FiltPathShort);
  FRIEND_TEST(meanfiltTest, FiltPathNormal40);
  FRIEND_TEST(meanfiltTest, FiltPathNormal30);

};
}  // optimizer
}  // toolbox

#endif  // __TOOLBOX_OPTIMIZER_MEANFILT_H__
