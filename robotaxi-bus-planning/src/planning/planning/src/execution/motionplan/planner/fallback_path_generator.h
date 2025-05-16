/**
 * @file fallback_path_generator.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "pnc_point.pb.h"

#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/trajectory/publishable_trajectory.h"

namespace acu {
namespace planning {
namespace fallback_path_generator {

void GenerateFallbackPathProfile(
  const ReferenceLineInfo* reference_line_info, PathInfo* path_data);
void GenerateFallbackPathProfileUsingPrevTrajectory(
  	const ReferenceLineInfo* reference_line_info, PathInfo* path_data,const PublishableTrajectory &path);

}
}  // namespace planning
}  // namespace acu
