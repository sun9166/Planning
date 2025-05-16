/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=========================================================================*/

#ifndef MODULES_MAP_HDMAP_HDMAP_UTIL_H_
#define MODULES_MAP_HDMAP_HDMAP_UTIL_H_

#include <memory>
#include <mutex>
#include <string>

#include "map_id.pb.h"
#include "map_speed_control.pb.h"

#include "common/util/file.h"
#include "common/util/string_util.h"
#include "map/vectormap/src/hdmap/hdmap.h"

/**
 * @namespace acu::hdmap
 * @brief acu::hdmap
 */
namespace acu {
namespace hdmap {
/*
inline std::string EndWayPointFile() {
  if (FLAGS_use_navigation_mode) {
    return acu::common::util::StrCat(
        FLAGS_navigation_mode_end_way_point_file);
  } else {
    return acu::common::util::StrCat(FLAGS_map_dir, "/",
                                        FLAGS_end_way_point_filename);
  }
}

inline std::string SpeedControlFile() {
  return acu::common::util::StrCat(FLAGS_map_dir, "/",
                                      FLAGS_speed_control_filename);
}
*/
/**
 * @brief create a Map ID given a string.
 * @param id a string id
 * @return a Map ID instance
 */
inline acu::hdmap::Id MakeMapId(const std::string& id) {
  acu::hdmap::Id map_id;
  map_id.set_id(id);
  return map_id;
}

}  // namespace hdmap
}  // namespace acu

#endif  // MODULES_MAP_HDMAP_HDMAP_UTIL_H_
