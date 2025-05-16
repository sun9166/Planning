/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MODULES_PERCEPTION_LIB_BASE_TIMER_H_
#define MODULES_PERCEPTION_LIB_BASE_TIMER_H_

#include <stdint.h>
#include <chrono>
#include <string>
#include <sys/time.h>

#include "common/base/macros.h"

using TimePoint = std::chrono::system_clock::time_point;
namespace  acu {
namespace common {
namespace util {
#define GLOG_TIMESTAMP(timestamp) \
  std::fixed << std::setprecision(9) << timestamp
class Timer {
public:
  Timer() = default;

  // no-thread safe.
  void start();

  // return the elapsed time,
  // also output msg and time in glog.
  // automatically start a new timer.
  // no-thread safe.
  uint64_t end(const std::string& msg);

private:
  // in ms.
  TimePoint start_time_;
  TimePoint end_time_;

  DISALLOW_COPY_AND_ASSIGN(Timer);
};

class TimerWrapper {
public:
  explicit TimerWrapper(const std::string& msg) : msg_(msg) { timer_.start(); }

  ~TimerWrapper() { timer_.end(msg_); }

  static double GetCurrentTime() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const double timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
    return timestamp / 1000000;
  }

private:
  Timer timer_;
  std::string msg_;

  DISALLOW_COPY_AND_ASSIGN(TimerWrapper);
};

}  // namespace util
}  // namespace common
}  // namespace  acu
#define PERF_FUNCTION(function_name) \
  acu::common::util::TimerWrapper _timer_wrapper_(function_name)

#define PERF_BLOCK_START()           \
  acu::common::util::Timer _timer_; \
  _timer_.start()

#define PERF_BLOCK_END(msg) _timer_.end(msg)

#endif  // MODULES_PERCEPTION_LIB_BASE_TIMER_H_
