/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

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

#include "common/base/time/include/rate.h"
#include "common/base/log/include/log.h"

namespace acu {
namespace common {

Rate::Rate(double frequency)
  : start_(NodeTime::Now()),
    expected_cycle_time_(1.0 / frequency),
    actual_cycle_time_(0.0) {}

Rate::Rate(uint64_t nanoseconds)
  : start_(NodeTime::Now()),
    expected_cycle_time_(static_cast<int64_t>(nanoseconds)),
    actual_cycle_time_(0.0) {}

Rate::Rate(const Duration& d)
  : start_(NodeTime::Now()), expected_cycle_time_(d), actual_cycle_time_(0.0) {}

void Rate::Sleep() {
  NodeTime expected_end = start_ + expected_cycle_time_;

  NodeTime actual_end = NodeTime::Now();

  // detect backward jumps in time
  if (actual_end < start_) {
    // AWARN << "Detect backward jumps in time";
    expected_end = actual_end + expected_cycle_time_;
  }

  // calculate the time we'll sleep for
  Duration sleep_time = expected_end - actual_end;

  // set the actual amount of time the loop took in case the user wants to kNow
  actual_cycle_time_ = actual_end - start_;

  // make sure to reset our start time
  start_ = expected_end;

  // if we've taken too much time we won't sleep
  if (sleep_time < Duration(0.0)) {
    // AWARN << "Detect forward jumps in time";
    // if we've jumped forward in time, or the loop has taken more than a full
    // extra
    // cycle, reset our cycle
    if (actual_end > expected_end + expected_cycle_time_) {
      start_ = actual_end;
    }
    // return false to show that the desired rate was not met
    return;
  }

  NodeTime::SleepUntil(expected_end);
}

void Rate::Reset() { start_ = NodeTime::Now(); }

Duration Rate::CycleTime() const { return actual_cycle_time_; }

}  // namespace common
}  // namespace acu
