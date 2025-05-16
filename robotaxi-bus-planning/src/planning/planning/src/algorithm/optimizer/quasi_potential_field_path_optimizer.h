/**
 * @file piecewise_jerk_path_optimizer.h
 **/

#pragma once

#include <utility>
#include <vector>
#include "planning_config.pb.h"
#include "path_optimizer.h"

namespace acu {
namespace planning {

class QuasiPotentialFieldPathOptimizer : public PathOptimizer {
 public:
  QuasiPotentialFieldPathOptimizer();
  bool Init(const PlanningConfig &config) override;

  virtual ~QuasiPotentialFieldPathOptimizer() = default;

 private:
  common::Status Process(const SpeedInfo& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         PathInfo* const path_data) override;

  bool OptimizePath(
      const std::array<double, 5>& init_state,
      const std::array<double, 5>& end_state, const double delta_s,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::tuple<double, double, double, double, std::string>>& soft_boundaries,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::vector<std::pair<int, int>>& boundary_types,
      const std::array<double, 8>& w, std::vector<double>* ptr_x,
      std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
      std::vector<double>* left_soft_delt_l, std::vector<double>* right_soft_delt_l,
      const int max_iter);

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s,
                                      double &property_length) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;
  bool GenerateReferencePathData(double start_s,PathInfo *const path_data);
  void AddPathPlanStithingTrajectory(FrenetFramePath& frenet_frame_path); 
  bool IsIdeaLaneChangeDistanceInSolidLine(const double& distance) const;
  double GetPreferredLaneChangeDistance() const;
  private:
   QuasiPotentialFieldPathConfig config_;
};

}  // namespace planning
}  // namespace acu
