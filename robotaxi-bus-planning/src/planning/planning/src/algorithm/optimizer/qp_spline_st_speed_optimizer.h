/**
 * @file qp_spline_st_speed_optimizer.h
 **/

#pragma once

#include <memory>
#include <string>

#include "planning_config.pb.h"
#include "qp_st_speed_config.pb.h"
#include "st_boundary_config.pb.h"

#include "src/algorithm/optimizer/speed_optimizer.h"
#include "src/algorithm/st_boundary_mapper/st_boundary_mapper.h"
#include "src/algorithm/speed_limit_decider/speed_limit_decider.h"
#include "src/algorithm/optimizer/smoothing_spline/active_set_spline_1d_solver.h"
#include "src/algorithm/optimizer/smoothing_spline/osqp_spline_1d_solver.h"
#include "src/algorithm/curve/constant_deceleration_trajectory1d.h"
#include "src/algorithm/curve/constant_acceleration_trajectory1d.h"

namespace acu {
namespace planning {

class QpSplineStSpeedOptimizer : public SpeedOptimizer {
 public:
  QpSplineStSpeedOptimizer();
  bool Init(const PlanningConfig& config) override;

 private:
  common::Status Process(const SLBoundary& adc_sl_boundary,
                         const PathInfo& path_data,
                         const acu::common::TrajectoryPoint& init_point,
                         const ReferenceLine& reference_line,
                         const SpeedInfo& reference_speed_data,
                         PathDecision* const path_decision,
                         SpeedInfo* const speed_data) override;
  void GenerateStopSpeedProfile(
          const double stop_point_s,const double init_speed,
          const double init_acc,SpeedInfo* const speed_data);
  bool GenerateFallbackSpeedData(
          const SpeedLimit speed_limit, const CarModel car_model, 
          const  std::vector<std::vector<std::pair<double, double>> > yield_st_map, 
          const  std::vector<std::vector<std::pair<double, double>> > yield_vt_map, 
          std::vector<double> t_evaluated, const PathDecision* path_decision, 
          SpeedInfo* const speed_data,
          const PathInfo& path_data) ; 

  QpStSpeedConfig qp_st_speed_config_;
  StBoundaryConfig st_boundary_config_;
  std::unique_ptr<math::Spline1dSolver> spline_solver_;
};

}  // namespace planning
}  // namespace acu
