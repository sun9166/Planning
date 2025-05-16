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

#include "common/base/time/include/node_time.h"

#include <time.h>
#include <chrono>
#include <limits>
#include <sstream>
#include <thread>

#if __GNUC__ >= 5
#include <iomanip>
#endif

namespace acu {
namespace common {

using std::chrono::high_resolution_clock;
using std::chrono::steady_clock;
using std::chrono::system_clock;

const NodeTime NodeTime::MAX = NodeTime(std::numeric_limits<uint64_t>::max());
const NodeTime NodeTime::MIN = NodeTime(1);

NodeTime::NodeTime(uint64_t nanoseconds) { nanoseconds_ = nanoseconds; }

NodeTime::NodeTime(int nanoseconds) {
  nanoseconds_ = static_cast<uint64_t>(nanoseconds);
}

NodeTime::NodeTime(double seconds) {
  nanoseconds_ = static_cast<uint64_t>(seconds * 1000000000UL);
}

NodeTime::NodeTime(uint32_t seconds, uint32_t nanoseconds) {
  nanoseconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nanoseconds;
}

NodeTime::NodeTime(const NodeTime& other) { nanoseconds_ = other.nanoseconds_; }

NodeTime& NodeTime::operator=(const NodeTime& other) {
  this->nanoseconds_ = other.nanoseconds_;
  return *this;
}

NodeTime NodeTime::Now() {
  auto now = high_resolution_clock::now();
  auto nano_time_point =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_time_point.time_since_epoch();
  uint64_t now_nano =
    std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return NodeTime(now_nano);
}

NodeTime NodeTime::MonoTime() {
  auto now = steady_clock::now();
  auto nano_time_point =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = nano_time_point.time_since_epoch();
  uint64_t now_nano =
    std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
  return NodeTime(now_nano);
}

double NodeTime::ToSecond() const {
  return static_cast<double>(nanoseconds_) / 1000000000UL;
}

bool NodeTime::IsZero() const { return nanoseconds_ == 0; }

uint64_t NodeTime::ToNanosecond() const { return nanoseconds_; }

std::string NodeTime::ToString() const {
  auto nano = std::chrono::nanoseconds(nanoseconds_);
  system_clock::time_point tp(nano);
  auto time = system_clock::to_time_t(tp);
  struct tm stm;
  auto ret = localtime_r(&time, &stm);
  if (ret == nullptr) {
    return std::to_string(static_cast<double>(nanoseconds_) / 1000000000.0);
  }
  std::stringstream ss;
#if __GNUC__ >= 5
  ss << std::put_time(ret, "%F %T");
  ss << "." << nanoseconds_ % 1000000000UL;
#else
  char date_time[128];
  strftime(date_time, sizeof(date_time), "%F %T", ret);
  ss << std::string(date_time) << "." << nanoseconds_ % 1000000000UL;
#endif
  return ss.str();
}

void NodeTime::SleepUntil(const NodeTime& time) {
  auto nano = std::chrono::nanoseconds(time.ToNanosecond());
  system_clock::time_point tp(nano);
  std::this_thread::sleep_until(tp);
}

Duration NodeTime::operator-(const NodeTime& rhs) const {
  return Duration(static_cast<int64_t>(nanoseconds_ - rhs.nanoseconds_));
}

NodeTime NodeTime::operator+(const Duration& rhs) const {
  return NodeTime(nanoseconds_ + rhs.ToNanosecond());
}

NodeTime NodeTime::operator-(const Duration& rhs) const {
  return NodeTime(nanoseconds_ - rhs.ToNanosecond());
}

NodeTime& NodeTime::operator+=(const Duration& rhs) {
  *this = *this + rhs;
  return *this;
}

NodeTime& NodeTime::operator-=(const Duration& rhs) {
  *this = *this - rhs;
  return *this;
}

bool NodeTime::operator==(const NodeTime& rhs) const {
  return nanoseconds_ == rhs.nanoseconds_;
}

bool NodeTime::operator!=(const NodeTime& rhs) const {
  return nanoseconds_ != rhs.nanoseconds_;
}

bool NodeTime::operator>(const NodeTime& rhs) const {
  return nanoseconds_ > rhs.nanoseconds_;
}

bool NodeTime::operator<(const NodeTime& rhs) const {
  return nanoseconds_ < rhs.nanoseconds_;
}

bool NodeTime::operator>=(const NodeTime& rhs) const {
  return nanoseconds_ >= rhs.nanoseconds_;
}

bool NodeTime::operator<=(const NodeTime& rhs) const {
  return nanoseconds_ <= rhs.nanoseconds_;
}

std::ostream& operator<<(std::ostream& os, const NodeTime& rhs) {
  os << rhs.ToString();
  return os;
}

}  // namespace common
}  // namespace acu
