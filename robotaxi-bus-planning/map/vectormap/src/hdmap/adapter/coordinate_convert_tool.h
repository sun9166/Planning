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
#ifndef MODULES_MAP_MAP_LOADER_ADAPTER_COORDINATE_CONVERT_TOOL_H_
#define MODULES_MAP_MAP_LOADER_ADAPTER_COORDINATE_CONVERT_TOOL_H_
#include <proj_api.h>
#include <string>
#include "map/vectormap/src/hdmap/adapter/xml_parser/status.h"
#include "map/vectormap/src/vectormap/alog.h"

namespace acu {
namespace hdmap  {
namespace adapter {

class CoordinateConvertTool {
 public:
  CoordinateConvertTool();
  ~CoordinateConvertTool();

 public:
  static CoordinateConvertTool* GetInstance();

 public:
  Status SetConvertParam(const std::string &source_param,
                        const std::string &dst_param);
  Status CoordiateConvert(const double longitude, const double latitude,
                          const double height_ellipsoid, double* utm_x,
                          double* utm_y, double* utm_z);

 private:
  std::string source_convert_param_;
  std::string dst_convert_param_;

  projPJ pj_from_;
  projPJ pj_to_;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace acu

#endif  // MODULES_MAP_MAP_LOADER_ADAPTER_COORDINATE_CONVERT_TOOL_H_
