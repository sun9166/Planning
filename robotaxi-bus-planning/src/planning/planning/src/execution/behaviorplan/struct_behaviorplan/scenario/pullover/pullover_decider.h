#ifndef PULLOVER_DECIDER_H_
#define PULLOVER_DECIDER_H_

#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_base.h"
#include "src/execution/behaviorplan/struct_behaviorplan/checker/path_search.h"

namespace acu {
namespace planning {

class PulloverDecider : public ScenarioBase {
 public:
 	PulloverDecider();
 	~PulloverDecider();

 	bool MakeIntentionDecision();
 	bool MakeLateralDecision();
 	
 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_ = nullptr;
 	std::set<int> block_set_;
  std::set<int> clear_area_;
 	std::vector<std::vector<MapNodeStruct>> points_;
  std::map<int, ReferenceLineFrame*> lines_;

 	void BuildBlockSet();
 	bool CheckDestClear(const double dest_s, const double min_s, const double max_s);
 	bool IsEgoStable();
  void GetOptionLine();
  bool FindTargetLine();
  void BuildBlockSet(const ReferenceLineFrame* line);
};

}  // namespace planning
}  // namespace acu

#endif  // PULLOVER_DECIDER_H_
