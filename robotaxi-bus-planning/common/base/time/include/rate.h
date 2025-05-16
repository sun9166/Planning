/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/03/28    1.0.0        refer to apollo3.5
 *****************************************************************************/
#ifndef ACU_TIME_RATE_H_
#define ACU_TIME_RATE_H_

#include "common/base/time/include/duration.h"
#include "common/base/time/include/node_time.h"

namespace acu {
namespace common {

class Rate {
public:
  explicit Rate(double frequency);
  explicit Rate(uint64_t nanoseconds);
  explicit Rate(const Duration&);
  void Sleep();
  void Reset();
  Duration CycleTime() const;
  Duration ExpectedCycleTime() const { return expected_cycle_time_; }

private:
  NodeTime start_;
  Duration expected_cycle_time_;
  Duration actual_cycle_time_;
};

}  // namespace common
}  // namespace acu

#endif  // ACU_TIME_RATE_H_
