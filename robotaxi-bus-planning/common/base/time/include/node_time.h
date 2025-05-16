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
#ifndef ACU_TIME_TIME_H_
#define ACU_TIME_TIME_H_

#include <limits>
#include <string>

#include "duration.h"

namespace acu {
namespace common {

class NodeTime {
public:
  static const NodeTime MAX;
  static const NodeTime MIN;
  NodeTime() {}
  explicit NodeTime(uint64_t nanoseconds);
  explicit NodeTime(int nanoseconds);
  explicit NodeTime(double seconds);
  NodeTime(uint32_t seconds, uint32_t nanoseconds);
  NodeTime(const NodeTime& other);
  NodeTime& operator=(const NodeTime& other);
  ~NodeTime() {}

  static NodeTime Now();
  static NodeTime MonoTime();
  static void SleepUntil(const NodeTime& time);

  double ToSecond() const;
  uint64_t ToNanosecond() const;
  std::string ToString() const;
  bool IsZero() const;

  Duration operator-(const NodeTime& rhs) const;
  NodeTime operator+(const Duration& rhs) const;
  NodeTime operator-(const Duration& rhs) const;
  NodeTime& operator+=(const Duration& rhs);
  NodeTime& operator-=(const Duration& rhs);
  bool operator==(const NodeTime& rhs) const;
  bool operator!=(const NodeTime& rhs) const;
  bool operator>(const NodeTime& rhs) const;
  bool operator<(const NodeTime& rhs) const;
  bool operator>=(const NodeTime& rhs) const;
  bool operator<=(const NodeTime& rhs) const;

private:
  uint64_t nanoseconds_ = 0;
};

std::ostream& operator<<(std::ostream& os, const NodeTime& rhs);

}  // namespace common
}  // namespace acu

#endif  // ACU_TIME_TIME_H_
