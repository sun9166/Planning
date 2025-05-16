/**
 * @file path_bound_generator.h
 **/

#pragma once
#include "../base_generator/path_bound_generator.h"

namespace acu {
namespace planning {
class LiteCouplingBoundGenerator : public PathBoundGenerator {
 public:
	LiteCouplingBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info, 
		const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state);
	~LiteCouplingBoundGenerator() = default;

	common::Status Generate(std::vector<PathBoundary>& candidate_path_boundaries) override;

	bool GetBoundaryFromDecisionDynamicBoxObstacles(LaneBound* const path_boundaries,
													std::string* const blocking_obstacle_id);

 protected:

Status GenerateRegularLiteCouplingPathBound(
    const LaneBorrowInfo& lane_borrow_info, LaneBound* const path_bound,
    std::string* const blocking_obstacle_id,
    std::string* const borrow_lane_type);

	std::vector<ObstacleEdge> SortBehaviorDynamicBoxes();


 private:
	bool enable_lite_coupling_ = false;
}; 

}  // namespace planning
}  // namespace acu