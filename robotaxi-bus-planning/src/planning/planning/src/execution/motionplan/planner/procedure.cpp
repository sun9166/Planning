/**
 * @file procedure.cpp
 **/

#include "procedure.h"

namespace acu {
namespace planning {

using acu::common::Status;

Procedure::Procedure(const std::string& name) : name_(name) {}

const std::string& Procedure::Name() const { return name_; }

bool Procedure::Init(const PlanningConfig&) { return true; }

Status Procedure::Execute(Frame* frame, ReferenceLineInfo* reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;
  return Status::OK();
}

}  // namespace planning
}  // namespace acu
