#ifndef DYNAMIC_MAP_SEARCH_H
#define DYNAMIC_MAP_SEARCH_H

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class DynamicMapSearch {
 public:
  DynamicMapSearch();
  ~DynamicMapSearch();

  bool GetSearchResult();
  void FiltSpecialObjects();
  void FiltCrossObjects(const bool use_local);
  void FiltRearObjects(LineObject& object, const double ttc);
  void FiltParaObjects(LineObject& object);
  void FindParaObjects();
  void ExpandST();
  void RecoverST();
  bool ModifyST(const bool is_target, LineObject& object);
  bool GenerateTrajectory(const bool is_comfort);
  bool SatisfyConstrain(const int t, const int s, const int last_s);
  void GetFinalDecision();
  void GetFinalGoal();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	ReferenceLineFrame* target_line_;
 	std::vector<std::map<int, SpeedNodeStruct>> points_;
 	std::vector<LineObject> prior_objects_;
 	std::vector<LineObject> objects_;
  std::set<int> special_objects_;
  std::set<int> follow_objects_;
  std::set<int> para_objects_;
  std::set<int> conflict_objects_;
 	double max_dec_;
  double max_acc_;
  double path_length_;
  std::vector<int> s_sequence_;
};

}  //  namespace planning
}  //  namespace acu

#endif