/**
 * @file path_bound_generator.h
 **/

#pragma once
#include "../base_generator/path_bound_generator.h"

namespace acu {
namespace planning {
class PullOverBoundGenerator : public PathBoundGenerator {
 public:
  PullOverBoundGenerator(Frame* frame, ReferenceLineInfo* reference_line_info,
  	 const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state);
  ~PullOverBoundGenerator() = default;
  
  const std::string& Name() const override{
    return "PullOverBoundGenerator";
  }
  
  common::Status Generate(std::vector<PathBoundary>& candidate_path_boundaries) override;

  bool InitPathBoundary(LaneBound* const path_bound,
	    std::vector<std::pair<int, int>>* const boundary_type, LaneBound* const dynamic_bound,
	    std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary) override;

 protected:
  std::vector<ObstacleEdge> SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles, 
    const double& obstacle_lat_buffer, const LaneBound* path_boundaries) override;

 private:
  bool IsWithinPullOverPathDeciderScopeObstacle(const Obstacle& obstacle);
 	bool GetPullOverTailPathBound(LaneBound* const path_bound, const double dis_to_curb, const PathDecision* path_decision);
 	bool SearchPullOverPosition(const PathDecision* path_decision,
       const std::vector<std::tuple<double, double, double>>& path_bound,const double pull_over_start_s,
       size_t &position_index);
  
}; 

}  // namespace planning
}  // namespace acu