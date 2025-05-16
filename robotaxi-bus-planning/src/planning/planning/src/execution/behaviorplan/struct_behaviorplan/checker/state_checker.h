#ifndef STATE_CHECKER_H_
#define STATE_CHECKER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/behaviorplan/struct_behaviorplan/checker/path_search.h"

namespace acu {
namespace planning {

class ReadyStateCheck {
 public:
  ReadyStateCheck();
  ~ReadyStateCheck();

  int GetReadyState();
  bool IsInPathBound();
  bool IsLcDisEnough();
  bool IsGlobalLaneBlocked();
  bool ReachTargetLine();

 private:
  DecisionContext* context_ = DecisionContext::Instance();
  DataPool* DP_ = DataPool::Instance();
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
};

}  // namespace planning
}  // namespace acu

#endif  // DECIDER_BASE_H_
