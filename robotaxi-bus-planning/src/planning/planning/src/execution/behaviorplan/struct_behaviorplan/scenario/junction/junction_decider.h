#ifndef JUNCTION_DECIDER_H_
#define JUNCTION_DECIDER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_base.h"

namespace acu {
namespace planning {

class JunctionDecider : public ScenarioBase {
 public:
 	JunctionDecider();
 	~JunctionDecider();

 	bool MakeUpperDecision() override;
 	
 private:
 	
 	DecisionContext* context_ = DecisionContext::Instance();
};

}  // namespace planning
}  // namespace acu

#endif  // JUNCTION_DECIDER_H_