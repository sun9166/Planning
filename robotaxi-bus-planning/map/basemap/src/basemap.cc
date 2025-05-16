/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName: basemap.cc
*
* Description: basemap API

*
* History:
* lbh         2018/05/22    1.0.0    build this module.
******************************************************************************/

#include "map/basemap/include/basemap.h"
#include "common/util/file.h"
#include "config_schema.pb.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace acu {
namespace map {

BaseMap::BaseMap(std::string map_dir, acu::map::mpHeader &config) {
  png_manager_.reset(new basemap_shm_util::ShmParser(
                       config.image_matrix_size, config.basemap_image_size,
                       config.basemap_resolution_ratio, map_dir, config.image_pixel_chars,
                       config.basemap_filename_level));
}

int BaseMap::CreatSharedMemory() { return png_manager_->CreatSharedMemory(); }
int BaseMap::FreeSharedMemory() { return png_manager_->FreeSharedMemory(); }

int BaseMap::NewSharedMemoryPtr() {
  if (png_manager_->ShmPtr() != NULL) {
    std::cout << "return existed SharedMemoryPtr" << std::endl;
    return 0;
  }
  int t = 1000;
  while (--t) {
    if (png_manager_->ShmPtr() == NULL) {
      png_manager_->NewSharedMemoryPtr();
      usleep(1000 * 1000);
    } else
      break;
  }
  if (t <= 0) {
    std::cout << "no shm to use, so new BaseMap() failed" << std::endl;
    return -1;
  } else {
    std::cout << "new BaseMap() succeed." << std::endl;
    return 0;
  }
}

int BaseMap::DeleteSharedMemoryPtr() {
  return png_manager_->DeleteSharedMemoryPtr();
}

void BaseMap::Init(double x, double y) { png_manager_->Init(x, y); }

int BaseMap::UpdateShm(double x, double y) {
  int ret = png_manager_->UpdateShm(x, y);
  //  png_manager_->DTGUpdateShm(x, y);  // just used in unstructed map, disabled in XingJi project
  return  ret;
}

eCellInfo BaseMap::GetCellInfo(const PointGCCS &car_pgccs,
                               const PointVCS &target_pvcs) {
  bool flag = false;
  eCellInfo cell_info = png_parser_.GetObject(
                          png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag));

  if (!flag) return eCellInfo::UNKNOWN;

  return cell_info;
}

bool BaseMap::IsInFreespace(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs) const {
  uint img_pixel = 0;
  bool flag = false;

  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);

  //  ROS_ERROR_STREAM("*********************" << flag << "  " << uint (*((unsigned char *)&img_pixel + 0)) << "   " <<
  //    uint (*((unsigned char *)&img_pixel + 1)) << "   " << uint (*((unsigned char *)&img_pixel + 2)));


  if (!flag || (0 == img_pixel) || (1 == img_pixel)) return false;

  // std::cout << "basemap:" << (int)(*((unsigned char *)&img_pixel + 1)) << std::endl;
  if (*((unsigned char *)&img_pixel + 1) < 50) // for distance map 0/out; /255/in.
    return false;
  else
    return true;
}

bool BaseMap::IsInFreespace(const acu::common::math::Box2d &box2d, const acu::vectormap::PointVCS &target_pvcs) {
  std::vector<acu::common::math::Vec2d> corners;
  box2d.GetAllCorners(&corners);
  std::vector<bool> isIn;
  // std::cout << std::endl;
  for (size_t i = 0; i < corners.size(); i++) {
    PointGCCS pgccs;
    pgccs.xg = corners[i].x();
    pgccs.yg = corners[i].y();
    // std::cout << "xg: " << pgccs.xg << ", yg: " << pgccs.yg  << "\t";
    if(!IsInFreespace(pgccs, target_pvcs)) {
      return false;
    }
  }
  return true;
}

int BaseMap::GetLeftRightDistanceToFreespace(
    const acu::vectormap::PointGCCS &car_gccs, double &heading,  double &left, double &right) const {
  acu::vectormap::PointVCS pvcs = {0, 0, 0};
  if (!IsInFreespace(car_gccs, pvcs)) {
    left = 0.0;
    right = 0.0;
    return -1;
  }
  acu::common::math::Vec2d tmp_point(car_gccs.xg, car_gccs.yg);
  double res = 0.05;
  acu::common::math::Vec2d right_in;
  acu::common::math::Vec2d right_out;
  bool first_out_r = false;
  acu::common::math::Vec2d left_in;
  acu::common::math::Vec2d left_out;
  bool first_out_l = false;
  for(size_t i = 0; i < 1400; i++)  //1400 -> 70m
  {
    double dis = i * res;
    acu::common::math::Box2d car_box(tmp_point, heading, 0, 2 * dis);// length=0,width=140
    std::vector<acu::common::math::Vec2d> corners;
    car_box.GetAllCorners(&corners);
    PointGCCS pgccs;
    pgccs.xg = corners[0].x();// 0 = 3
    pgccs.yg = corners[0].y();
    if (first_out_r == false) {
      if ( IsInFreespace(pgccs, pvcs)) {
        right_in = corners[0];
      }
      else{
        right_out = corners[0];
        first_out_r = true;
      }
    }
    
    pgccs.xg = corners[1].x();// 1 = 2
    pgccs.yg = corners[1].y();
    if (first_out_l == false) {
      if (IsInFreespace(pgccs, pvcs)) {
        left_in = corners[1];
      }
      else {
        left_out = corners[1];
        first_out_l = true;
      }
    }
    if (first_out_r && first_out_l) {
      break;
    }
  }
  if (first_out_r)
  {
    acu::common::math::LineSegment2d right_line(right_in, right_out);
    acu::common::math::LineSegment2d right_result(right_line.center(), tmp_point);
    right = right_result.length();
  }
  else {
    right = 70.0;
  }
  
  if (first_out_l)
  {
    acu::common::math::LineSegment2d left_line(left_in, left_out);
    acu::common::math::LineSegment2d left_result(left_line.center(), tmp_point);
    left = left_result.length();
  }
  else {
    left = 70.0;
  }
  // std::cout << "left: " << left << ", right: " << right << std::endl;

  return 0;
}

double  BaseMap::DistanceToFreeSpace(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs) const {
  // positive in freespace ; negative out freespace
  uint img_pixel = 0;
  bool flag = false;
  double distance = 0.0;
  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);
  bool positive_flag = (*((unsigned char *)&img_pixel + 1) == 0) ? false : true; // 0: out ; 255:in.
  // std::cout << (int)(*((unsigned char *)&img_pixel + 1)) << std::endl;
  if (!flag || (0 == img_pixel) || (1 == img_pixel)) {

    return (positive_flag == true) ? 100000 : -1000000;
  }


  distance = (double)(*((unsigned char *)&img_pixel + 2)) * 0.05;
  return (positive_flag == true) ? distance : -distance;
}

FunctionType BaseMap::GetFunctionType(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs) {
  uint img_pixel = 0;
  FunctionType result;
  uint X = 0;
  uint Y = 0;
  bool flag = false;
  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);
  if (!flag || (0 == img_pixel) || (1 == img_pixel)) {
    result.FunctionRegionType = 0;
    result.AttentionRegionType = 0;
    result.IsValid = false;
    return result;
  }
  result.IsValid = true;
  X = (uint (*((unsigned char *)&img_pixel + 1))) / 50;
  //X = uint (*((unsigned char *)&img_pixel + 1));
  Y = (uint (*((unsigned char *)&img_pixel + 1))) % 50;
  // if (X < 50) result.FunctionRegionType = 0;
  //  else if (X >= 200) result.FunctionRegionType = 1;
  //       else result.FunctionRegionType = 2;
  switch (X) {
  case 0  : result.FunctionRegionType = 0; break;
  case 2  : result.FunctionRegionType = 2; break;
  case 4  : result.FunctionRegionType = 1; break;
  default : result.FunctionRegionType = 0;
    result.IsValid = false;
    break;
  }

  switch (Y) {
  case 0 : result.AttentionRegionType = 0; break;
  case 10 : result.AttentionRegionType = 1; break;
  case 20 : result.AttentionRegionType = 2; break;
  case 30 : result.AttentionRegionType = 3; break;
  case 40 : result.AttentionRegionType = 4; break;
  default : result.AttentionRegionType = 0;
    result.IsValid = false;
    break;
  }
  return result;
}

LocValue BaseMap::GetLocValue(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs) {
  uint img_pixel = 0;
  LocValue result;
  uint U = 0;
  uint V = 0;
  bool flag = false;
  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);
  if (!flag || (0 == img_pixel) || (1 == img_pixel)) {
    result.UsingGPS = false;
    result.Pro = 0;
    result.IsValid = false;
    return result;
  }
  U = uint (*((unsigned char *)&img_pixel + 0));
  V = uint ((*((unsigned char *)&img_pixel + 0)) % 50);
  if (U < 50) result.UsingGPS = false;
  else if (U >= 100 && U < 150) result.UsingGPS = true;
  else result.UsingGPS = false;
  result.Pro = V / 10.0;
  result.IsValid = true;
  return result;
}

DTGProto BaseMap::GetDistanceAndNearest(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs)
{
  uint img_pixel = 0;
  bool flag = true;
  DTGProto result;
  result.Distance = 0;
  result.IsInFreespace = false;
  result.NearestGCCS.x = 0;
  result.NearestGCCS.y = 0;
  result.NearestVCS.x = 0;
  result.NearestVCS.y = 0;
  result.IsValid = false;

  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);
  if (!flag) return result;

  if (!flag || (0 == img_pixel) || (1 == img_pixel) ||
      (*((unsigned char *)&img_pixel + 1) < 50)) {
    // ROS_ERROR_STREAM("GetCellBGRA failed");
    result.IsValid = true;
    return result;
  }

  result.IsInFreespace = img_pixel;

  if (!png_manager_->GetDTGValue(car_pgccs, target_pvcs,
                                 result.Distance, result.NearestGCCS.x, result.NearestGCCS.y)) {
    printf("GetDTGValue failed\n");
    return result;
  }
  // AINFO << std::fixed << "result: " << result.Distance << "=(" << result.NearestGCCS.x  << "," << result.NearestGCCS.y << ") ";

  acu::vectormap::PointVCS vcs;
  acu::vectormap::PointGCCS re_gccs;
  re_gccs.xg = result.NearestGCCS.x;
  re_gccs.yg = result.NearestGCCS.y;
  re_gccs.angle = 0.0;
  if (GeoTransform(car_pgccs, re_gccs, vcs) == -1) {  //debug
    result.NearestVCS.x = 0;
    result.NearestVCS.y = 0;
    result.IsValid = false;
    return result;
  }
  result.NearestVCS.x = vcs.x;
  result.NearestVCS.y = vcs.y;
  // result.NearestVCS.angle = 0.0;
  result.IsValid = true;
  // ROS_ERROR_STREAM("********  " << result.NearestVCS.x << " ***   " << result.NearestVCS.y);
  return result;
}

DTGProto BaseMap::GetDistanceTransformGraphProto(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs)
{
  uint img_pixel = 0;
  bool flag = false;
  DTGProto result;
  img_pixel = png_manager_->GetCellBGRA(car_pgccs, target_pvcs, flag);
  if (!flag || (0 == img_pixel) || (1 == img_pixel) ||
      (*((unsigned char *)&img_pixel + 1) < 50)) {
    result.Distance = 0;
    result.IsInFreespace = false;
    result.NearestGCCS.x = 0;
    result.NearestGCCS.y = 0;
    result.NearestVCS.x = 0;
    result.NearestVCS.y = 0;
    result.IsValid = false;
    return result;
  }

  result.IsInFreespace = img_pixel;

  // unsigned char r = (unsigned char)(img_pixel & 0x000000ff);
  // unsigned char g = (unsigned char)((img_pixel & 0x0000ff00)>>8);
  // unsigned char b = (unsigned char)((img_pixel & 0x00ff0000)>>16);

  // result.NearestGCCS.x = ((signed char)g)*0.05;
  // result.NearestGCCS.y = ((signed char)b)*0.05;
  // result.Distance = r*0.05;

  result.Distance = (*((unsigned char *)&img_pixel + 2)) * 0.05;

  acu::vectormap::PointGCCS gccs;
  acu::vectormap::PointVCS vcs;
  if (GeoTransform(car_pgccs, target_pvcs, gccs) == -1) {
    result.NearestGCCS.x = 0;
    result.NearestGCCS.y = 0;
    result.NearestVCS.x = 0;
    result.NearestVCS.y = 0;
    result.IsValid = false;
    return result;
  }
  result.NearestGCCS.x = gccs.xg + (*((signed char *)&img_pixel + 1)) * 0.05;
  result.NearestGCCS.y = gccs.yg + (*((signed char *)&img_pixel + 0)) * 0.05;
  // ROS_ERROR_STREAM("--------  " << result.Distance << " ---   " << result.NearestGCCS.x
  //               << " --- " << result.NearestGCCS.y << " ---   " << result.IsInFreespace);
  // ROS_ERROR_STREAM("********  " << target_pvcs.x << " ***   " << target_pvcs.y);
  // ROS_ERROR_STREAM("........  " << (int)(*((unsigned char *)&img_pixel + 2)) * 0.05
  //               << " ...   " << (int)(*((signed char *)&img_pixel + 3)) * 0.05);
  acu::vectormap::PointGCCS re_gccs;
  re_gccs.xg = result.NearestGCCS.x;
  re_gccs.yg = result.NearestGCCS.y;
  re_gccs.angle = 0.9;
  if (GeoTransform(car_pgccs, re_gccs, vcs) == -1) {  //debug
    result.NearestVCS.x = 0;
    result.NearestVCS.y = 0;
    result.IsValid = false;
    return result;
  }
  result.NearestVCS.x = vcs.x;
  result.NearestVCS.y = vcs.y;
  result.IsValid = true;
  // ROS_ERROR_STREAM("********  " << result.NearestVCS.x << " ***   " << result.NearestVCS.y);
  return result;
}

}  // namespace map
}  // namespace acu
