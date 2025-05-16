#ifndef STRUCT_DECISION_H_
#define STRUCT_DECISION_H_

#include "src/execution/behaviorplan/struct_behaviorplan/dispatcher/scenario_dispatcher.h"
#include "src/execution/behaviorplan/struct_behaviorplan/scenario/scenario_factory.h"
#include "src/execution/behaviorplan/struct_behaviorplan/checker/state_checker.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/slt/trajectory_decision.h"
#include "src/execution/behaviorplan/struct_behaviorplan/checker/passable_path_search.h"

namespace acu {
namespace planning {

class StructDecision {
 public:
 	StructDecision();
 	~StructDecision();

  void Init();
  void ResetData();
  void PullData();
  void Process();
  void PushData();
  void AddDebugInfo();
  bool ReferenceLineInit();
  void ReplanStateManager();
  void UpdateCloudCommand();
  void UpdatePedalCommand();
  void CheckEmergencyLevel();
  void GetCongestionState();
  int GetTurningDirection();
  int GetPathDirection();
  void GetTurnDirection();
  eLaneType GetLaneType();
  void CheckPassableForReplan();
  bool CheckMissionLcPassableForReplan();
  bool CheckCruisePassableForReplan();
  void AddExtraDebugInfo();
  void CheckNeedAddStopPointOrNot(std::map<int,double>& search_s, 
                        const double min_dis_to_end,
                        const double dis_to_junction_thres,
                        const double delta_for_reach_global);
  void AddBlackRoad(int &front_lane_size,
                  const double min_dis_to_end,
                  const double dis_to_junction_thres,
                  const double delta_for_reach_global);
  string LaneToRoad(string lane_id);
 

 private:
  DataPool* DP_ = DataPool::Instance();
  DecisionContext* context_ = DecisionContext::Instance();
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  std::pair<double, int> turn_;
  std::pair<double, int> merge_;
  
};

}  // namespace planning
}  // namespace acu

#endif  // STRUCT_DECISION_H_
