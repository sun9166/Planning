#ifndef JUNCTION_ANALYSIS_H
#define JUNCTION_ANALYSIS_H

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "src/execution/cognition/struct_cognition/referenceline_frame/st_map/st_map.h"

namespace acu {
namespace planning {

class JunctionAnalysis {
 public:
  JunctionAnalysis();
  ~JunctionAnalysis();

  void AddInteractionInfo();
  void GetCrossWalkPriority(const LineObject& object);
  void GetStraightPriority(const LineObject& object);
  bool NearCrosswalk(const LineObject& object);
  bool IsStraight(const LineObject& object);

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  int junction_type_;
  double dec_, dec_s_;
};

}  //  namespace planning
}  //  namespace acu

#endif
