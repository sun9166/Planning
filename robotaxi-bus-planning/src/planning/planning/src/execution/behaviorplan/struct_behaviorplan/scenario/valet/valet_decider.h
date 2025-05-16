#ifndef VALET_DECIDER_H_
#define VALET_DECIDER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_base.h"

namespace acu {
namespace planning {

class ValetDecider : public ScenarioBase {
 public:
 	ValetDecider();
 	~ValetDecider();

 	bool MakeLateralDecision();
 	bool MakeObjectDecision();
 	bool MakeUpperDecision() override;
 	
 private:
};

}  // namespace planning
}  // namespace acu

#endif  // VALET_DECIDER_H_