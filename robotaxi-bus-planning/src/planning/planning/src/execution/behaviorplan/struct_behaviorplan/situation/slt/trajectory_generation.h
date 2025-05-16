#ifndef TRAJECTORY_GENERATION_H
#define TRAJECTORY_GENERATION_H

#include "src/execution/behaviorplan/struct_behaviorplan/situation/slt/grid_generation.h"

namespace acu {
namespace planning {

class TrajectoryGeneration {
 public:
  TrajectoryGeneration();
  ~TrajectoryGeneration();

  bool GenerateSLTTrajectory();
  bool SearchTrajectory();
  void AddSpeedLimit();
  void GetFinalTrajectory();
  void CalculateCost(const GridInfoStruct& pre_point, GridInfoStruct& point);
  bool GetSafetyCost(const GridInfoStruct& pre_point, GridInfoStruct& point, double& cost);
  bool GetComfortCost(const GridInfoStruct& pre_point, GridInfoStruct& point, double& cost);

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
  double length_, width_, front_over_hang_, back_over_hang_;
  std::map<int, int> line_index_;
  std::map<double, double> speed_limit_;

};

}  //  namespace planning
}  //  namespace acu

#endif