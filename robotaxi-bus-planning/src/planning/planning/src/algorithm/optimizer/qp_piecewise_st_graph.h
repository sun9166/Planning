/**
 * @file qp_spline_st_graph.h
 **/

#pragma once

#include <memory>
#include <utility>
#include <vector>
#include "pnc_point.pb.h"
#include "qp_st_speed_config.pb.h"
#include "common/util/string_util.h"
#include "planning_internal_info.pb.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"
#include "src/execution/motionplan/common/st_graph/st_graph_data.h"
#include "src/algorithm/optimizer/smoothing_spline/piecewise_linear_generator.h"

namespace acu {
namespace planning {

class QpPiecewiseStGraph {
 public:
  explicit QpPiecewiseStGraph(const QpStSpeedConfig& qp_st_speed_config,
                              const StGraphData& st_graph_data);

  void SetDebugLogger(planning_internal::STGraphDebug* st_graph_debug);

  common::Status Search(//const StGraphData& st_graph_data,
                        SpeedInfo* const speed_data,
                        const std::pair<double, double>& accel_bound);

 private:
  void Init();

  // Add st graph constraint
  common::Status AddConstraint(const common::TrajectoryPoint& init_point,
                               const SpeedLimit& speed_limit,
                               const std::vector<const StBoundary*>& boundaries,
                               const std::pair<double, double>& accel_bound);

  // Add objective function
  common::Status AddKernel(const std::vector<const StBoundary*>& boundaries,
                           const SpeedLimit& speed_limit);

  // solve
  common::Status Solve();

  // extract upper lower bound for constraint;
  common::Status GetSConstraintByTime(
      const std::vector<const StBoundary*>& boundaries, const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound) const;

  // generate reference speed profile
  // common::Status ApplyReferenceSpeedProfile();
  common::Status AddCruiseReferenceLineKernel(const SpeedLimit& speed_limit,
                                              const double weight);

  common::Status AddFollowReferenceLineKernel(
      const std::vector<const StBoundary*>& boundaries, const double weight);

  common::Status EstimateSpeedUpperBound(
      const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
      std::vector<double>* speed_upper_bound) const;

 private:
  // qp st configuration
  const QpStSpeedConfig qp_st_speed_config_;

  // initial status
  common::TrajectoryPoint init_point_;

  // solver
  std::unique_ptr<math::PiecewiseLinearGenerator> generator_ = nullptr;

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;

  // evaluated points
  std::vector<double> t_evaluated_;

  // reference line kernel
  std::vector<double> cruise_;

  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;

  const StGraphData& st_graph_data_;
};

}  // namespace planning
}  // namespace acu
