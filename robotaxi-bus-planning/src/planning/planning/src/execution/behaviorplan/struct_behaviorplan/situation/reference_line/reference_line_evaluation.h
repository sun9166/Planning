#ifndef REFERENCE_LINE_EVALUATION_H__
#define REFERENCE_LINE_EVALUATION_H__

#include "static_map_search.h"

namespace acu {
namespace planning {

class ReferenceLineEvaluation {
 public:
  ReferenceLineEvaluation();
  ~ReferenceLineEvaluation();

  void Init();
  void GetLineCost();
  void GetGlobalCost();
  void GetExcuteCost();
  void GetStableCost();
  void GetExtraCost();
  float GetPrepareCost();
  float GetAbandonCost();
  bool IsRoadCongestion();
  bool OffGlobalPath(ReferenceLineFrame* line);
  static bool CompareCost(OptionStruct s1, OptionStruct s2);
  OptionStruct best_option() { return options_.front(); }

 private:
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
 	std::vector<OptionStruct> options_;
 	int left_block_status_, right_block_status_;
  DecisionContext* context_ = DecisionContext::Instance();
};

}  //  namespace planning
}  //  namespace acu

#endif