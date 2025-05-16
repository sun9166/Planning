#ifndef CRUISE_DECIDER_H_
#define CRUISE_DECIDER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_base.h"

namespace acu {
namespace planning {

class CruiseDecider : public ScenarioBase {
 public:
 	CruiseDecider();
 	~CruiseDecider();

 	bool MakeUpperDecision() override;

 	DecisionContext* context_ = DecisionContext::Instance();
 	
 private:
};

}  // namespace planning
}  // namespace acu

#endif  // CRUISE_DECIDER_H_