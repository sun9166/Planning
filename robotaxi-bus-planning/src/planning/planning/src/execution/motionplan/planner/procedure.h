/**
 * @file procedure.h
 **/

#pragma once
#include <string>

#include "planning_config.pb.h"

#include "common/common_header/status.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"

namespace acu {
namespace planning {

class Procedure {
 public:
  explicit Procedure(const std::string& name);
  virtual ~Procedure() = default;
  virtual const std::string& Name() const;
  virtual acu::common::Status Execute(
      Frame* frame, ReferenceLineInfo* reference_line_info);

  virtual bool Init(const PlanningConfig& config);

 protected:
  bool is_init_ = false;
  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;

 private:
  const std::string name_;
};

}  // namespace planning
}  // namespace acu

