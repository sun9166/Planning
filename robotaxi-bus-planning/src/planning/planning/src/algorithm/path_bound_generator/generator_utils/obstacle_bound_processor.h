/**
 * @file obstacle_bound_processor.h
 **/

#pragma once
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/datatype.h"
#include "bound_adjustment.h"

namespace acu {
namespace planning {
class ObstacleBoundProcessor{
public:
	ObstacleBoundProcessor(Frame* frame, ReferenceLineInfo* reference_line_info,
  							const double& adc_frenet_s, const double& adc_frenet_l);

	~ObstacleBoundProcessor() = default;

	std::vector<ObstacleEdge> 
	SortObstaclesForSweepLine(const IndexedList<std::string, Obstacle>& indexed_obstacles, 
      						  const double& obstacle_lat_buffer, const LaneBound* path_boundaries);

	bool 
	GetSoftBoundaryFromUncertainObstacles(
      const PathDecision* path_decision,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      const std::unordered_map<std::string, bool>& obs_id_to_direction,
      std::vector<std::pair<int, int>>* const boundary_type,
      std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary);

	bool 
	GetDynamicObstacleBoxes(const uint32_t& num_of_time_stamps,
                const double& eval_time_interval, const double& total_time, 
                std::vector<std::vector<common::math::Box2d>>& dynamic_obstacle_boxes);

	bool 
	GetDynamicObstacleSLBoundaries(const uint32_t& num_of_time_stamps,
                const double& eval_time_interval, const double& total_time, 
                std::vector<std::vector<SLBoundary>>& dynamic_obstacle_sl_boundaries);

	bool 
	CheckStaticObstacleNudgeAvailable(const Obstacle& obstacle,
      			const LaneBound* path_boundaries, double& nudge_dis_available) const;
protected:


private:
	bool 
	IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle);

	bool 
	GetBoundaryMin(const LaneBound* path_boundaries,const double& start_s, 
   				const double& end_s, const int& obstacle_pos, std::pair<double, double>& boundary_min) const;

	Frame* frame_ = nullptr;
	ReferenceLineInfo* reference_line_info_ = nullptr;
	double adc_frenet_s_ = 0.0;
	double adc_frenet_l_ = 0.0;
  	std::shared_ptr<BoundAdjustment> bound_adjustment_ptr_;
}; 

}  // namespace planning
}  // namespace acu