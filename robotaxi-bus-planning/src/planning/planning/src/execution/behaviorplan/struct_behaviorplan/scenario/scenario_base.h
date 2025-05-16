#ifndef SCENARIO_BASE_H_
#define SCENARIO_BASE_H_

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/intention/behavior_intention.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/reference_line/reference_line_evaluation.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/object/object_evaluation.h"

namespace acu {
namespace planning {

class ScenarioBase {
 public:
 	ScenarioBase();
 	virtual ~ScenarioBase();

 	virtual bool MakeIntentionDecision();
 	virtual bool MakeLateralDecision();
 	bool MakeObjectDecision();
 	virtual bool MakeUpperDecision();

 	void GenerateSentence();
 	void GenerateSpecialSentence();
 	void GenerateBoundary();
 	void ModifyBoundary();
 	void FindTargetGap();
  void FitGapSpeed();
 	StructReferenceLineInfo* reference_line_;

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
};

}  // namespace planning
}  // namespace acu

#endif  // SCENARIO_BASE_H_