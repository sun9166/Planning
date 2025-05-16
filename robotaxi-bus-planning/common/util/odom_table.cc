/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * locer          2019/06/11    1.0.0        build
 *****************************************************************************/

#include "common/util/odom_table.h"
#include "common/base/log/include/log.h"

namespace acu {
namespace util {

OdomTable::OdomTable(int buffse_size) : max_size_(buffse_size) {
  if (max_size_ < 50) max_size_ = 50;
}

void OdomTable::PushPose(FusionPose& pose) {
  std::unique_lock<std::mutex> lock(odom_deque_mutex_);
  if (deque_.size() > 0) {
    double dt = pose.t - deque_.back().t;
    if (dt < 0 || dt > 0.3) {
      Reset();
    }
  }
  deque_.push_back(pose);
  int size = deque_.size();
  // AINFO << "fusion deque : " << size << " " << std::setprecision(16) << deque_.back().t;
  while (size > max_size_) {
    deque_.pop_front();
    --size;
  }
}

int OdomTable::GetPose(double t, FusionPose& pose) {
  std::unique_lock<std::mutex> lock(odom_deque_mutex_);
  int index = FindDataIndex(t, deque_);
  if (index >= 0) {
    pose = deque_[index];
    if (pose.loc_status == 2) {
      return -1;
    }
  }
  return index;
}

int OdomTable::GetDRPose(const double& t, FusionPose& pose) {
  std::unique_lock<std::mutex> lock(odom_deque_mutex_);
  int index = FindDataIndex(t, deque_);
  if (index < 0) return index;
  pose = deque_[index];
  return 0;
}

void OdomTable::ClearDeque() {
  std::unique_lock<std::mutex> lock(odom_deque_mutex_);
  while (!deque_.empty()) {
    deque_.pop_front();
  }
}
int OdomTable::GetSize() { return deque_.size(); }

int OdomTable::FindDataIndex(const double time, const std::deque<FusionPose>& deque) {
  if (deque.empty()) {
    AWARN << " odom table deque is empty.";
    return -1;
  }
  if (time <= deque.front().t) {
    AWARN << std::setprecision(16) << "query t:" << time << " deque front:" << deque.front().t;
    return -1;
  }
  if (time > deque.back().t) {
    if (time > deque.back().t + 0.1) {
      AWARN << std::setprecision(16) << "query t:" << time << " deque back:" << deque.back().t;
      return -2;
    } else {
      AWARN << std::setprecision(16) << "query t:" << time << " deque back:" << deque.back().t << " greater less 0.1";
      return deque.size() - 1;
    }
  }
  const auto end = std::lower_bound(deque.begin(), deque.end(), time,
                                    [](const FusionPose& pose, const double time) { return pose.t < time; });
  if (end != deque.end()) {
    int index = std::distance(deque.begin(), end);
    if (end->t == time) {
      return index;
    }
    if (end == deque.begin()) {
      return 0;
    } else {
      const auto start = std::prev(end);
      if (start != deque.end()) {
        if (fabs(time - start->t) > fabs(time - end->t)) {
          return index;
        } else {
          return index - 1;
        }
      }
    }
  } else {
    return deque.size() - 1;
  }
  return -1;
}

int OdomTable::FindDataIndex2(const double time, const std::deque<FusionPose>& deque) {
  int size = deque.size();
  if (size == 0) return -1;
  int index = -1;
  for (int k = size - 1; k >= 0; --k)
    if (time - deque[k].t >= 0) {
      index = k;
      break;
    }
  if (index == -1) return -1;
  if (time - deque[index].t > 0.1) return -2;
  return index;
}
}  // namespace util
}  // namespace acu
