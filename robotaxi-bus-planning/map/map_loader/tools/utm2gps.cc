/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: geotool_test.cc
*
* Description: test ACU geotool API

*
* History:
* xiayang         2021/11/02    1.0.0    build this module.
******************************************************************************/
#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>
#include <vector>
#include <boost/concept_check.hpp>
#include "common/base/log/include/acu_node.h"
#include "map/vectormap/include/geotool.h"


using namespace std;
using namespace acu::vectormap;

int main(int argc, char **argv) {
  std::string node_name = "utm2gps";
  ros::init(argc, argv, node_name);
  ros::NodeHandle private_nh("~");
  bool Enable_GLOG_Screen = true;
  acu::common::AcuNode::Init(node_name, Enable_GLOG_Screen);
  PointGPS pgps;
  PointGCCS gccs;
  GeoTool geo;
  int utm_zone = 50;
  private_nh.param("utm_xg", gccs.xg, 443528.941802);
  private_nh.param("utm_yg", gccs.yg, 4436812.24313);
  private_nh.param("utm_zone", utm_zone, 50);
  gccs.angle = 330;
  geo.SetUtmZone(utm_zone);
  AINFO << std::setprecision(9) 
      << "(utm_xg,utm_yg)-->" <<"("<< gccs.xg << "," << gccs.yg<<")";
  AINFO<<"utm_zone-->"<<utm_zone;
  geo.GCCS2GPS(gccs, pgps);
  AINFO << std::setprecision(10) 
  		<< "(gps_lon,gps_lat)-->" <<"("<< pgps.lon << "," << pgps.lat<<")";
  return 0;
}