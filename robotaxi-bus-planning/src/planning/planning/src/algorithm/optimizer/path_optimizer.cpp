/**
 * @file path_optimizer.cpp
 **/

#include "path_optimizer.h"
#include "src/execution/motionplan/common/planning_gflags.h"

namespace acu {
namespace planning {

using acu::common::Status;

PathOptimizer::PathOptimizer(const std::string& name) : Procedure(name) {}

acu::common::Status PathOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  Procedure::Execute(frame, reference_line_info);
  auto ret = Process(
      reference_line_info->speed_data(), reference_line_info->reference_line(),
      frame->PlanningStartPoint(), reference_line_info->mutable_path_data());
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
  }
  return ret;
}

void PathOptimizer::RecordDebugInfo(const PathInfo& path_data) {
  const auto& path_points = path_data.discretized_path();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

}  // namespace planning
}  // namespace acu
