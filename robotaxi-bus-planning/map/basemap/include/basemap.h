/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName: basemap.h
*
* Description: basemap API

*
* History:
* lbh         2018/05/22    1.0.0    build this module.
******************************************************************************/

#ifndef BASEMAP_API_H_
#define BASEMAP_API_H_

#include <memory>
#include "map/basemap/protocol_parser/png_parser.h"
#include "map/basemap/shm_dynamic/shm_parser.h"
#include "map/map_loader/include/map_common.h"

#include "common/math/box2d.h"

namespace acu {
namespace map {

struct FunctionType
{
    int FunctionRegionType; 
    int AttentionRegionType;
    bool IsValid;
};

struct LocValue
{
    bool UsingGPS;
    double Pro;
    bool IsValid;
};

struct DTGPoint
{
  double x;
  double y;
  DTGPoint() {
    this->x = 0;
    this->y = 0;
  }
};

struct DTGProto
{
  double Distance;
  bool IsInFreespace;
  bool IsValid;
  DTGPoint NearestGCCS;
  DTGPoint NearestVCS;
  DTGProto () {
    this->Distance = 0;;
    this->IsInFreespace = false;
    this->IsValid = false;
  }
};

class BaseMap {
 public:
  BaseMap(std::string map_dir, acu::map::mpHeader &config);
  ///////////////////////////////////////////
  //   users should use this api only    ////
  //////////////////////////////////////////
  /**
   * @brief get the object property of target point in basemap.
   * @param car_pgccs gccs point of the car
   * @param target_pvcs the vcs point of the object
   * @return enum eCellInfo defined in map/basemap/src/png_parser.h
   */
  eCellInfo GetCellInfo(const PointGCCS &car_pgccs,
                        const PointVCS &target_pvcs);

  bool IsInFreespace(const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs) const;
  bool IsInFreespace(const acu::common::math::Box2d &box2d, 
        const acu::vectormap::PointVCS &target_pvcs);
  int GetLeftRightDistanceToFreespace(const acu::vectormap::PointGCCS &car_gccs, 
        double &heading,  double &left, double &right) const;
  double  DistanceToFreeSpace(const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs) const;
  FunctionType GetFunctionType(const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs);
  LocValue GetLocValue(const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs);

  /**
    * @brief get the object property of target point in basemap.
    * @param car_pgccs gccs point of the car
    * @param target_pvcs the vcs point of the object
    * @return distance transform graph proto of the target point(pro and using gps status)      
    */
  DTGProto GetDistanceTransformGraphProto(
      const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs = {0, 0, 0});

  DTGProto GetDistanceAndNearest(
      const acu::vectormap::PointGCCS &car_pgccs, 
      const acu::vectormap::PointVCS &target_pvcs = {0, 0, 0});

  /////////////////////////////////////////////////////////////////
  ////////   used by root to manage shm    ///////////////////////
  ////////////////////////////////////////////////////////////////
 public:
  // preload basemap by init
  void Init(double x, double y);
  int UpdateShm(double x, double y);
  // 4 functions about shared memory. you neednt to manager shared memory and
  // ptr,
  // but you should call the 4 functions in order.

  // creat and free the shared memory
  int CreatSharedMemory();
  int FreeSharedMemory();

  // new and delete the ptr to shared memory in current process
  int NewSharedMemoryPtr();
  int DeleteSharedMemoryPtr();

 private:
  std::unique_ptr<MemoryManagerBase> png_manager_;
  PNGParser png_parser_;
};
}  // namespace map
}  // namespace acu

#endif  // BASEMAP_API_H_
