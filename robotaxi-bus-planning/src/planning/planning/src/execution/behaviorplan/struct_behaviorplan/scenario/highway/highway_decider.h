#ifndef HIGHWAY_DECIDER_H_
#define HIGHWAY_DECIDER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_base.h"

namespace acu {
namespace planning {

class HighwayDecider : public ScenarioBase {
 public:
 	HighwayDecider();
 	~HighwayDecider();

 	bool MakeLateralDecision();
 	bool MakeObjectDecision();
 	bool MakeUpperDecision() override;
 	
 private:
};

}  // namespace planning
}  // namespace acu

#endif  // HIGHWAY_DECIDER_H_