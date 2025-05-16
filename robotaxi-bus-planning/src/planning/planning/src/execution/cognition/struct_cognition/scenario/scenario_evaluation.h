#ifndef SCENARIO_EVALUATION_H_
#define SCENARIO_EVALUATION_H_

#include "datapool/include/data_pool.h"

namespace acu{
namespace planning {

class ScenarioEvaluation {
public:

  ScenarioEvaluation();
  ~ScenarioEvaluation();
  void EvaluateScenario(); 

private:
  void EvaluateRoad();
  void EvaluateRoadSide();
  void EvaluateJunction();
  void EvaluateNearJunction();
  void EvaluateMultiLanes();
  void EvaluateObject();
  void EvaluateInvader();
  void EvaluateWaiting();


private:
  PlanningMainData* DP_;
  StructScenarioInfo* scenario_info_;
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;

  int lane_num_;
  int current_index_;
  float dis_to_junction_;
  bool have_side_lane_;
  std::vector<ReferenceLineFrame*> lines_;
  std::map<double, int> side_parking_;
  std::map<double, int> on_road_;
    
};

} // namespace planning
} // namespace acu


#endif
