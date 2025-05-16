#ifndef GRID_GENERATION_H
#define GRID_GENERATION_H

#define STEP_S 1.0
#define MAX_S 100.0
#define MAX_T 6.0
#define VALID_COST 1000000.0

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class GridGeneration {
 public:
  GridGeneration();
  ~GridGeneration();

  void InitProbGrid();
  void AddGridInfo();
  void AddMultiLaneGrid(std::vector<std::vector<GridInfoStruct>>& sl_grids);
  void AddSingleLaneGrid(std::vector<std::vector<GridInfoStruct>>& sl_grids);
  void AddGridLineIds(const int index, GridInfoStruct& grid);
  void AddObjectInfo();
  bool SLInBoundary(const StructSLBoundary& sl_boundary);

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
  std::vector<ReferenceLineFrame*> lines_;
  double half_width_;

};

}  //  namespace planning
}  //  namespace acu

#endif
