/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * locer          2019/06/11    1.0.0        build
 *****************************************************************************/
#ifndef ACU_UTIL_ODOM_TABLE_H_
#define ACU_UTIL_ODOM_TABLE_H_

#include <algorithm>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <mutex>

typedef unsigned short int u16;
typedef unsigned int u32;

namespace acu {
namespace util {

struct FusionPose {
  double t, v;
  double x,y,z;
  double xdr, ydr, zdr, yawdr;
  double angle;
  int loc_status;
  FusionPose()
  {
    t = v=0;
    x=y=z=0;
    xdr=ydr=zdr=yawdr=0;
    angle=0;
    loc_status=0;
  }
};

class OdomTable {
 public:
  OdomTable(int buffer_size);
  ~OdomTable() {}

  void Reset() { deque_.clear(); }
  void PushPose(FusionPose& pose);
  int GetPose(double t, FusionPose& pose);
  int GetDRPose(const double& t, FusionPose& pose);
  void ClearDeque();
  int GetSize();

 protected:
  int FindDataIndex(const double time, const std::deque<FusionPose>& deque);
  int FindDataIndex2(const double time, const std::deque<FusionPose>& deque);

 protected:
  std::deque<FusionPose> deque_;
  std::mutex odom_deque_mutex_;
  int max_size_;
};
}  // namespace util
}  // namespace acu
#endif
