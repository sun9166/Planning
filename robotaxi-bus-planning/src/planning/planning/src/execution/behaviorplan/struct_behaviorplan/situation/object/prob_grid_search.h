#ifndef PROB_GRID_SEARCH_H
#define PROB_GRID_SEARCH_H

#define MAX_COST 1000000.0

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/object/interaction/object_manager.h"

namespace acu {
namespace planning {

class ProbGridSearch {
 public:
  ProbGridSearch();
  ~ProbGridSearch();

  bool GetSearchResult();
  void AddSpeedLimit();
  bool GenerateTrajectory();
  void CalculateCost(const STNodeStruct& parent, STNodeStruct& child);
  bool SatisfyConstrain(const STNodeStruct& parent, const STNodeStruct& child);
  float SafetyCost(const STNodeStruct& parent, const STNodeStruct& child);
  float ObjectCost(const STNodeStruct& parent, const STNodeStruct& child);
  float ComfortCost(const STNodeStruct& parent, const STNodeStruct& child);
  void GetFinalDecision();
  void GetObjectDecision();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  ReferenceLineFrame* target_line_;
  ReferenceLineFrame* search_line_;
  std::vector<std::map<float, STNodeStruct>> points_;
  std::set<int> special_objects_;
  double path_length_;
  std::vector<float> s_sequence_;
  std::map<double, double> speed_limit_;
};

}  //  namespace planning
}  //  namespace acu

#endif
