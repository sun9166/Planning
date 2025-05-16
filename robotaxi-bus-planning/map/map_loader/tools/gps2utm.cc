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
  std::string node_name = "gps2utm";
  ros::init(argc, argv, node_name);
  ros::NodeHandle private_nh("~");
  bool Enable_GLOG_Screen = true;
  acu::common::AcuNode::Init(node_name, Enable_GLOG_Screen);
  PointGPS pgps;
  PointGCCS gccs;
  GeoTool geo;
  private_nh.param("gps_lon", pgps.lon, 116.337666164);
  private_nh.param("gps_lat", pgps.lat, 40.0796899483);
  pgps.heading =  330;
  geo.GPS2GCCS(pgps, gccs);
  AINFO << std::setprecision(9) 
  		<< "(xg,yg)-->" <<"("<< gccs.xg << "," << gccs.yg<<")";
  return 0;
}