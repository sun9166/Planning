/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName:
*
* Description:

*
* History:
* lbh         2018/05/22    1.0.0    build this module.
******************************************************************************/

#ifndef MEMORY_MANAGER_BASE_H_
#define MEMORY_MANAGER_BASE_H_

#include <opencv2/core/core.hpp>
#include "map/vectormap/include/geotool.h"

namespace acu {
namespace map {
using namespace acu::vectormap;
using namespace acu::map;
class MemoryManagerBase {
 public:
  MemoryManagerBase() {}
  virtual ~MemoryManagerBase(){}

  virtual int CreatSharedMemory() = 0;
  virtual int FreeSharedMemory() = 0;
  virtual int NewSharedMemoryPtr() = 0;
  virtual int DeleteSharedMemoryPtr() = 0;
  virtual void *ShmPtr() = 0;
  virtual int UpdateShm(double x, double y){return 0;}
  virtual int DTGUpdateShm(double x, double y){return 0;}

  virtual int Init(double x, double y){return 0;}

  virtual uint GetCellBGRA(const PointGCCS &car_pgccs,
                              const PointVCS &target_pvcs, bool &flag) = 0;
  virtual bool GetDTGValue(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs, 
    double& distance, double& point_x, double& point_y) = 0;
};
}  // namespace map
}  // namespace acu

#endif
