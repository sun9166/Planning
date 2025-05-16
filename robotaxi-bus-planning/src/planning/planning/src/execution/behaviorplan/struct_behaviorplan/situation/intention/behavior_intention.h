#ifndef BEHAVIOR_INTENTION_H_
#define BEHAVIOR_INTENTION_H_

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/object/object_evaluation.h"

namespace acu {
namespace planning {

class BehaviorIntention {
 public:
  BehaviorIntention();
  ~BehaviorIntention();

  bool Init();
  bool HavePullOver();
  bool HaveDeparture();
 	bool HaveAbandonLC();
 	bool HaveCommandLC();
  bool HaveMissionLC();
  bool HaveObstacleAvoidance();
  bool HaveSpeedLC();
  bool HaveOffsetCenter();
  bool HaveMeeting();
  bool CheckStartCondition();
  void CheckRearCar();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_ = nullptr;
};

}  //  namespace planning
}  //  namespace acu

#endif
