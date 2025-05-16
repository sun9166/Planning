/******************************************************************************
 * Copyright 2017 The  acu Authors. All Rights Reserved.
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

/**
 * @file
 * @brief Some util functions.
 */

#ifndef MODULES_COMMON_UTIL_H_
#define MODULES_COMMON_UTIL_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "google/protobuf/util/message_differencer.h"

#include "common/math/vec2d.h"
#include "common/common_proto/pnc_point.pb.h"
#include "map/vectormap_proto/proto_obj/geometry.pb.h"
//#include "modules/perception/proto/perception_obstacle.pb.h"

/**
 * @namespace  acu::common::util
 * @brief  acu::common::util
 */

namespace  acu {
namespace common {
namespace util {


/**  //@pqg add
 * uniformly slice a segment [start, end] to num + 1 pieces
 * the result sliced will contain the n + 1 points that slices the provided
 * segment. `start` and `end` will be the first and last element in `sliced`.
 */
template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T>* sliced) {
  if (!sliced || num == 0) {
    return;
  }
  const T delta = (end - start) / num;
  sliced->resize(num + 1);
  T s = start;
  for (uint32_t i = 0; i < num; ++i, s += delta) {
    sliced->at(i) = s;
  }
  sliced->at(num) = end;
}

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  return google::protobuf::util::MessageDifferencer::Equals(a, b);
}

struct PairHash {
  template <typename T, typename U>
  size_t operator()(const std::pair<T, U>& pair) const {
    return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
  }
};

/**
 * @brief create a SL point
 * @param s the s value
 * @param l the l value
 * @return a SLPoint instance
 */
SLPoint MakeSLPoint(const double s, const double l);

PointENU MakePointENU(const double x, const double y, const double z);

PointENU MakePointENU(const math::Vec2d& xy);

/*acu::perception::Point MakePerceptionPoint(const double x, const double y,
                                             const double z);*/

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da);

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa);

template <typename Container>
typename Container::value_type MaxElement(const Container& elements) {
  return *std::max_element(elements.begin(), elements.end());
}

template <typename Container>
typename Container::value_type MinElement(const Container& elements) {
  return *std::min_element(elements.begin(), elements.end());
}

/**
 * calculate the distance beteween PathPoint a and PathPoint b
 * @param a one path point
 * @param b another path point
 * @return sqrt((a.x-b.x)^2 + (a.y-b.y)^2), i.e., the Euclid distance on XY
 * dimension
 */
double Distance2D(const PathPoint& a, const PathPoint& b);

int myexec(const char *cmd, std::vector<std::string> &resvec);
bool GetSrcPath(std::string &res);
bool getPackagePath(std::string package_name, std::string &res) ;

}  // namespace util
}  // namespace common
}  // namespace  acu

template <typename A, typename B>
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  return os << "first: " << p.first << ", second: " << p.second;
}

#endif  // MODULES_COMMON_UTIL_H_
