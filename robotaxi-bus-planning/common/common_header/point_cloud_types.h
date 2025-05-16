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

#ifndef __ACU_COMMON_LIDAR_POINT_TYPES_H__
#define __ACU_COMMON_LIDAR_POINT_TYPES_H__

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace acu {
namespace driver {


struct PointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  float angle;
  uint8_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

}  // namespace driver
}  // namespace acu

POINT_CLOUD_REGISTER_POINT_STRUCT(acu::driver::PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (float, angle, angle)
                                  (uint8_t, ring, ring)
                                  (double, timestamp, timestamp))

typedef acu::driver::PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
#endif  // __ACU_COMMON_LIDAR_POINT_TYPES_H__
