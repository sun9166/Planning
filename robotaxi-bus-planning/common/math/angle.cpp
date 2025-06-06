/******************************************************************************
 * Copyright 2017 The acu Authors. All Rights Reserved.
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

#include "common/math/angle.h"

#include "common/math/sin_table.h"

namespace acu {
namespace common {
namespace math {

float sin(Angle16 a) {
  auto idx = a.raw();
  if (idx < -Angle16::RAW_PI_2) {
    idx += Angle16::RAW_PI;
    return -SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  if (idx < 0) {
    return -SIN_TABLE[(-idx) % SIN_TABLE_SIZE];
  }
  if (idx < Angle16::RAW_PI_2) {
    return SIN_TABLE[idx % SIN_TABLE_SIZE];
  }
  idx = Angle16::RAW_PI - idx;
  return SIN_TABLE[idx % SIN_TABLE_SIZE];
}

float cos(Angle16 a) {
  Angle16 b(Angle16::RAW_PI_2 - a.raw());
  return sin(b);
}

float tan(Angle16 a) { return sin(a) / cos(a); }

float sin(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return sin(b);
}

float cos(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return cos(b);
}

float tan(Angle8 a) {
  Angle16 b(a.raw() << 8);
  return tan(b);
}

}  // namespace math
}  // namespace common
}  // namespace acu
