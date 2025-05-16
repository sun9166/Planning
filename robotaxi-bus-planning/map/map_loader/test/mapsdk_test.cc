#include<iostream>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <unistd.h>
#include "map/map_loader/include/map_loader.h"

using namespace acu::map;
using namespace acu::hdmap;
using namespace acu::vectormap;
using acu::common::PointENU;

static acu::vectormap::VectorMap* vcmap_ = acu::map::MapLoader::GetVectorMapPtr(); //init once
static acu::vectormap::HDMapImpl* hdmap_ = acu::map::MapLoader::GetHDMapImplPtr(); //init once
static acu::map::AppMap* appmap_ = acu::map::MapLoader::GetAppmapPtr(); //init once
static acu::map::Mapinfo* mapinfo_ = acu::map::MapLoader::GetMapinfoPtr(); //init once
static acu::map::BaseMap* basemap_ = MapLoader::GetBasemapPtr();

class TestMapSDK : public testing::Test
{
public:
    virtual void SetUp()
    {
        // std::cout<<"SetUp"<<std::endl;
    }
    virtual void TearDown()
    {
        // std::cout<<"TearDown"<<std::endl;
    }
};


double GetPointHeading(double xg, double yg) {

    acu::common::PointENU obj_pose;
    obj_pose.set_x(xg);//gccs1.xg);
    obj_pose.set_y(yg);//gccs1.yg);
    // std::cout << "target:" << std::setprecision(15) << xg << "," << yg << std::endl;

    const acu::vectormap::VectorMap* vcmap_ptr =
        acu::map::MapLoader::GetVectorMapPtr();
    if (vcmap_ptr == nullptr) {
        std::cout << "vcmap_ptr is nulltpr." << std::endl;
        return -1;
    }
    // double search_radius = 0.5;
    // std::vector<LaneInfoConstPtr> lanes;
    // vcmap_ptr->GetLanes(obj_pose, search_radius, lanes);
    // if (lanes.size() == 0) {
    //  std::cout << "Point Not in the lane." << std::endl;
    //  return 1000;
    // }
    double s_len = 0.0;
    double l_len = 0.0;
    LaneInfoConstPtr lane_ptr;
    vcmap_ptr->GetNearestLane(obj_pose, lane_ptr, s_len, l_len);
    if (lane_ptr == nullptr) {
        std::cout << "GetNearestLane get lane nullptr" << std::endl;
        return 1000;
    }
    double heading = 0.0;
    heading = lane_ptr->Heading(s_len);


    return heading * 180 / 3.1415926;
}





TEST_F(TestMapSDK, Appmap_GetFunctionPoints)  
{
    auto points = appmap_->GetFunctionPoints();
    for(int i = 0; i < points.size(); i++){
        std::cout << "GetPointHeading:"<< points[i].type<<"|"<<points[i].name<< "|"<<GetPointHeading(points[i].point.x, points[i].point.y)<<  std::endl;
    }
}

/*
./mapsdk_test --gtest_list_tests
TestMapSDK.
  Crosswalk
  Roadboundary
  GetCellInfo

./mapsdk_test --gtest_filter=TestMapSDK.Basemap_GetCellInfo
*/

int main(int argc,char *argv[])
{
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
