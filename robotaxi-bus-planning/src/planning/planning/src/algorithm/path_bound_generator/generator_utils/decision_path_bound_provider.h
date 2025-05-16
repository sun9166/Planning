/**
 * @file decision_path_bound_provider.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "datapool/include/data_pool.h"
#include "datapool/include/cognition_typedef.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"

namespace acu {
namespace planning {

class DecisionPathBoundProvider {
 public:
  DecisionPathBoundProvider() = default;
  ~DecisionPathBoundProvider() = default;
  
  bool GetDecisionPathBound(double &left_bound, double &right_bound,const double s,
                            const int left_lane_id, const int right_lane_id, const ReferenceLineInfo* target_reference_line_info);
  

 private:
   const ReferenceLineFrame* FindTargetRefLine(const int id, LanePosition& position);
 
 private:
   DataPool* DP_ = DataPool::Instance();
   
};

}  // namespace planning
}  // namespace acu
