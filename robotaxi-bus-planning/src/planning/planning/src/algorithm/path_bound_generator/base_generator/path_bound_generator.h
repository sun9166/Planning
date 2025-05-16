/**
 * @file path_bound_generator.h
 **/

#pragma once
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/datatype.h"
#include "../generator_utils/bound_adjustment.h"
#include "../generator_utils/obstacle_bound_processor.h"

namespace acu {
namespace planning {
namespace {
  constexpr double kPathBoundsDeciderHorizon = 100.0;//150.0;
  constexpr double kDefaultLaneWidth = 5.0;
  constexpr double kDefaultRoadWidth = 20.0;
  constexpr int kNumExtraTailBoundPoint = 1;
  constexpr double kPulloverLonSearchCoeff = 1.5;
  constexpr double kPulloverLatSearchCoeff = 1.25;
  constexpr double kDottedBoundBuffer = 0.5;
  constexpr double kPulloverForceConstrantBuffer = 1;

}

  
class PathBoundGenerator {
 public:
  explicit PathBoundGenerator(const std::string& name, Frame*, ReferenceLineInfo* reference_line_info, 
                              const std::pair<std::array<double, 3>, std::array<double, 3>>& start_frenet_state);
  virtual ~PathBoundGenerator() = default;
  virtual const std::string& Name() const;
  
  virtual bool 
  InitPathBoundary(LaneBound* const path_bound,
            std::vector<std::pair<int, int>>* const boundary_type, LaneBound* const dynamic_bound,
            std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary);
  
  common::Status 
  GetBoundary(const double& ADC_buffer, LaneBound* const path_bound, std::vector<std::pair<int, int>>* const boundary_type,
              std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary);
  
  bool 
  GetBoundaryFromStaticObstacles(
    const PathDecision* path_decision, LaneBound* const path_boundaries,
    std::string* const blocking_obstacle_id, const double& obstacle_lat_buffer,
    std::vector<std::pair<int, int>>* const boundary_type,
    std::vector<std::tuple<double, double, double, double, std::string>>* const soft_boundary); 
  
  bool 
  CalculateDynamicObstacleBound(LaneBound* const dynamic_bound);

  bool 
  CalculateBehaviorDynamicObstacleBound();
  
  virtual common::Status Generate(std::vector<PathBoundary>& candidate_path_boundaries) = 0;

 protected:

  bool 
  GetSolidLineStartS(const int& search_direction, double& solid_line_start_s);
  
  bool 
  IsInRoadBoundary(const double& adc_left, const double& adc_right, 
                   const double& start_s, double& boundary_error) const;
  
  bool 
  IsInDecisionBound(const double& adc_left, const double& adc_right, const double& start_s) const;
  
  bool 
  GetBoundaryFromDecision(const double& curr_s, double& left_bound, double& right_bound, 
                          int& left_bound_id, int& right_bound_id);
  
  bool 
  GetBoundaryFromLanesAndADC(const LaneBorrowInfo& lane_borrow_info, 
                             const double& ADC_buffer,
                             LaneBound* const path_bound, 
                             std::string* const borrow_lane_type, int& path_blocked_idx);
  
  bool 
  GetFreeSpaceBoundary(const std::vector<std::pair<int, int>>& boundary_type, 
                       LaneBound* const path_boundaries, int& path_blocked_idx);
  
  bool 
  ReCalculatePathBoundBaesdOnLaneType(
            const std::vector<std::pair<int, int>>& boundary_type, 
            LaneBound* const path_boundaries, 
            const size_t& idx, const double& fs_width_l, const double& fs_width_r) const;
  
  bool 
  GetBoundaryFromRoads(LaneBound* const path_bound, const int check_lane_type, int& path_blocked_idx);
  
  virtual std::vector<ObstacleEdge> 
  SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles, 
      const double& obstacle_lat_buffer, const LaneBound* path_boundaries);

  Frame* frame_ = nullptr;
  ReferenceLineInfo* reference_line_info_ = nullptr;
  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_ld_ = 0.0;
  std::shared_ptr<BoundAdjustment> bound_adjustment_ptr_;
  std::shared_ptr<ObstacleBoundProcessor> obstacle_bound_process_ptr_;
  std::vector<std::pair<int, int>> boundary_type_;
  std::vector<std::tuple<double, double, double, double, std::string>> soft_boundary_;
  LaneBound dynamic_bound_;
  BlockedInfo blocked_infos_;

 private:
  common::Status 
  CheckAbandonLCValid(double& k_length);
  
  common::Status 
  CalculateSingleBoundWithDecision(const double& curr_s, const bool& is_in_decision_bound, 
            double& curr_left_bound_lane, double& curr_right_bound_lane, 
            int& left_bound_id, int& right_bound_id);

  common::Status 
  CalculateSingleBoundWithoutDecision(const double& curr_s, const size_t& index,
            const double& curr_right_bound_adc, const double& curr_left_bound_adc, 
            double& curr_left_bound_lane, double& curr_right_bound_lane);

  common::Status 
  LimitSingleBoundByRoadWidth(
      const double& curr_s, const bool& is_in_road_boundary, const size_t& index,
      const double& s_max_consider_adc_l, const int& left_bound_id,
      const double& curr_left_bound_lane, const double& curr_right_bound_lane, 
      const double& curr_right_bound_adc, const double& curr_left_bound_adc, 
      double& curr_left_bound, double& curr_right_bound);


  const std::string name_;
}; 


}  // namespace planning
}  // namespace acu