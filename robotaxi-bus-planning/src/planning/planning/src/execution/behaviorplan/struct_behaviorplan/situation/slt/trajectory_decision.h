#ifndef TRAJECTORY_DECISION_H
#define TRAJECTORY_DECISION_H

#include "src/execution/behaviorplan/struct_behaviorplan/situation/slt/trajectory_generation.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/slt/dynamic_sl_generation.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/slt/path_generation.h"

namespace acu {
namespace planning {

class TrajectoryDecision {
 public:
  TrajectoryDecision();
  ~TrajectoryDecision();

  void MakeMeetingDecision();
  void GetSLAndBoundary();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
};

}  //  namespace planning
}  //  namespace acu

#endif