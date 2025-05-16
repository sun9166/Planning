#ifndef ACC_ANALYSIS_H
#define ACC_ANALYSIS_H

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/cognition/struct_cognition/referenceline_frame/st_map/st_map.h"

namespace acu {
namespace planning {

class AccAnalysis {
 public:
  AccAnalysis();
  ~AccAnalysis();

  void UpdateProbGrid(std::set<int>& special_objects);
  void UpdateExpectedAcc(std::set<int>& special_objects);
  void UpdatePushedPoints();
  bool AllowChangeAcc(const int& id);
  bool AllowChangeAcc(const int& id, const int& direction);
  bool AllowChangeAcc(const int& id, double& final_v);

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  ReferenceLineFrame* target_line_;
  ReferenceLineFrame* search_line_;
};

}  //  namespace planning
}  //  namespace acu

#endif
