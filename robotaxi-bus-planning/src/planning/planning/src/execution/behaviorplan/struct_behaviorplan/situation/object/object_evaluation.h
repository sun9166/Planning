#ifndef OBJECT_EVALUATION_H
#define OBJECT_EVALUATION_H

#include "dynamic_map_search.h"
#include "prob_grid_search.h"

namespace acu {
namespace planning {

class ObjectEvaluation {
 public:
  ObjectEvaluation();
  ~ObjectEvaluation();

  bool DepartureDecision();
  void SetLastLCPoint();
  void SetFollowLCPoint();
  void CrosswalkDecision();
  void SpeedLimitDecision();
  void VirtualObjectDecision();
  void ConflictZoneDecision();
  void CrossObjectDecision();
  void CrosswalkTempDecision(bool& is_slowdown, double& stop_dis);
  void TrafficLightDecision(const StructTrafficlightInfo& light);
  void AddStopPoint(const Site& point);
  bool GetConflictPoint(double& stop_s);
  void SetGuaranteeReplanPoint(const int ref_lane_id,const int left_or_right,const double block_s);



 private:
  DecisionContext* context_ = DecisionContext::Instance();
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  double lc_speed_limit_ = 100.0;
  double fixed_s = 1001.0;
  //
  double road_max_speed = 0.0;
  double road_min_speed = 2 / 3.6; //m/s
  double v2x_expected_speed = 16.6; //m/s
};
}  //  namespace planning
}  //  namespace acu

#endif
