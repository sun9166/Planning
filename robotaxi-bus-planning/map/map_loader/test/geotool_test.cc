/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: geotool_test.cc
*
* Description: test ACU geotool API

*
* History:
* lbh         2018/05/21    1.0.0    build this module.
******************************************************************************/
#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <boost/concept_check.hpp>

#include "map/vectormap/include/geotool.h"


using namespace std;
using namespace acu::vectormap;

constexpr double kMIN = 1e-5;
double size=10.0;
GeoTool geo(50, size);

class GeoToolTest : public testing::Test {
 protected:
  PointGCCS pgccs, target_pgccs;
  PointGPS pgps;
  PointVCS pvcs; PointGCCS ppgccs;PointGICS ppgics;
  PointGICS pgics, target_pgics;

 protected:
  virtual void SetUp() {
    pgccs={443528.941802, 4436812.24313, 120};
    pgps= {116.337666164, 40.0796899483, 330};
    pgics= {44352, 443681};
    pvcs={100.0,173.205080756,60};
    ppgccs={443328.941802,4436812.24313, 180};
    ppgics={44332,443681};
  }
  virtual void TearDown() {}
};

TEST_F(GeoToolTest, GCCS2GPS) {
  PointGPS gps;
  EXPECT_NE(geo.GCCS2GPS(pgccs,gps), -1);
  EXPECT_NEAR(gps.lon, pgps.lon, kMIN);
  EXPECT_NEAR(gps.lat, pgps.lat, kMIN);
  EXPECT_NEAR(gps.heading, pgps.heading, kMIN);
}
TEST_F(GeoToolTest, GPS2GCCS) {
  PointGCCS gccs;
  EXPECT_NE(geo.GPS2GCCS(pgps,gccs), -1);
  EXPECT_NEAR(gccs.xg, pgccs.xg, kMIN);
  EXPECT_NEAR(gccs.yg, pgccs.yg, kMIN);
  EXPECT_NEAR(gccs.angle, pgccs.angle, kMIN);
}

TEST_F(GeoToolTest, GCCS2GICS) {
  PointGICS gics;
  EXPECT_NE(geo.GCCS2GICS(pgccs,gics), -1);
  EXPECT_NEAR(gics.ug, pgics.ug, size);
  EXPECT_NEAR(gics.vg, pgics.vg, size);
}
TEST_F(GeoToolTest, GICS2GCCS) {
  PointGCCS gccs;
  EXPECT_NE(geo.GICS2GCCS(pgics,gccs), -1);
  EXPECT_NEAR(gccs.xg, pgccs.xg, size);
  EXPECT_NEAR(gccs.yg, pgccs.yg, size);
}

TEST_F(GeoToolTest, GCCS2VCS) {
  PointVCS vcs;
  EXPECT_NE(geo.GCCS2VCS(pgccs,ppgccs,vcs), -1);
  EXPECT_NEAR(vcs.x, pvcs.x, kMIN);
  EXPECT_NEAR(vcs.y, pvcs.y, kMIN);
  EXPECT_NEAR(vcs.angle, vcs.angle, kMIN);
}
TEST_F(GeoToolTest, VCS2GCCS) {
  PointGCCS gccs;
  EXPECT_NE(geo.VCS2GCCS(pgccs,pvcs,gccs), -1);
  EXPECT_NEAR(gccs.xg, ppgccs.xg, kMIN);
  EXPECT_NEAR(gccs.yg, ppgccs.yg, kMIN);
  EXPECT_NEAR(gccs.angle, ppgccs.angle, kMIN);
}

TEST_F(GeoToolTest, GICS2VCS) {
  PointVCS vcs;
  EXPECT_NE(geo.GICS2VCS(pgccs,ppgics,vcs), -1);
  EXPECT_NEAR(vcs.x, pvcs.x, size);
  EXPECT_NEAR(vcs.y, pvcs.y, size);
}
TEST_F(GeoToolTest, VCS2GICS) {
  PointGICS gics;
  EXPECT_NE(geo.VCS2GICS(pgccs,pvcs,gics), -1);
  EXPECT_NEAR(gics.ug, ppgics.ug, kMIN);
  EXPECT_NEAR(gics.vg, ppgics.vg, kMIN);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}