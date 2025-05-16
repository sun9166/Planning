#ifndef SCENARIO_DISPATCHER_H_
#define SCENARIO_DISPATCHER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class ScenarioDispatcher {
 public:
 	ScenarioDispatcher();
 	~ScenarioDispatcher();

 	bool GetScenarioType(eScenarioEnum& type);
 	bool SatisfyParkingCondition();
 	void GetDisToSolid();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
};

}  // namespace planning
}  // namespace acu

#endif  // DECIDER_BASE_H_