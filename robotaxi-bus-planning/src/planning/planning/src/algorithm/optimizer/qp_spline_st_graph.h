/**
 * @file qp_spline_st_graph.h
 **/

#pragma once

#include <utility>
#include <vector>

#include "pnc_point.pb.h"
#include "qp_st_speed_config.pb.h"
#include "planning_internal_info.pb.h"

#include "common/util/string_util.h"
#include "common/common_header/status.h"
#include "datapool/include/common_config.h"
#include "src/execution/motionplan/common/path/path_info.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"
#include "src/execution/motionplan/common/path_decision.h"
#include "src/execution/motionplan/common/st_graph/st_graph_data.h"
#include "src/algorithm/optimizer/smoothing_spline/spline_1d_solver.h"
#include "src/algorithm/optimizer/smoothing_spline/active_set_spline_1d_solver.h"
#include "src/algorithm/curve/piecewise_jerk_trajectory1d.h"


namespace acu {
namespace planning {
enum class eAccelAndDecelMode {
  DEFAULT = 0,
  NORMAL_DECEL_MODE,
  EMERGENCY_DECEL_MODE
};

class QpSplineStGraph {
 public:
  QpSplineStGraph(math::Spline1dSolver* spline_solver,
                  const QpStSpeedConfig& qp_st_speed_config,
                  const CarModel& veh_param,
                  const bool is_change_lane,
                  const bool has_overtake,
                  const StGraphData& st_graph_data,
                  PathDecision* const path_decision);

  void SetDebugLogger(planning_internal::STGraphDebug* st_graph_debug, LatencyStats* latency_ptr);

  common::Status Search(const std::pair<double, double>& accel_bound,
                        const SpeedInfo& reference_speed_data,
                        SpeedInfo* const speed_data);

  std::vector<std::vector<std::pair<double, double>> > GetYeildStMap(){
    return yield_st_map_;
  };

  std::vector<std::vector<std::pair<double, double>> > GetYeildVtMap(){
    return yield_vt_map_;
  };

  std::vector<double> GetTEvaluated() {
    return t_evaluated_;
  }
 
 private:
  void Init(PathDecision* const path_decision);

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

  common::Status GetSConstraintByTime(const double time,
      const double total_path_s, double* const s_upper_bound,
      double* const s_lower_bound, const std::vector<const StBoundary*>& boundaries) const;

  // reference line kernel is a constant s line at s = 250m
  common::Status AddCruiseReferenceLineKernel(const double weight);

  // follow line kernel
  // common::Status AddFollowReferenceLineKernel(
      // const std::vector<const StBoundary*>& boundaries, const double s_weight,const double v_weight);

  // common::Status AddFollowStMapKernel(const double s_weight,const double v_weight);

  // 合并follow和yield的stmap计算结果；
  common::Status AddFollowAndYieldStMapKernel(const double s_weight,const double v_weight);
  common::Status AddFollowAndYieldReferenceLineKernel(
          const std::vector<const StBoundary*>& boundaries,const double s_weight,const double v_weight);

  common::Status AddPiecewiseJerkVtReferenceKernel(const double weight);
  // yield line kernel
  // common::Status AddYieldReferenceLineKernel(
      // const std::vector<const StBoundary*>& boundaries, const double weight);

  // yield line kernel
  // common::Status AddYieldStMapKernel(const std::vector<const StBoundary*>& boundaries);

  // common::Status AddYieldStMapKernel(
      // const std::vector<const StBoundary*>& boundaries,const std::array<double, 3>& w);
  
  // overtake line kernel
  common::Status AddOvertakeReferenceLineKernel(
      const std::vector<const StBoundary*>& boundaries, const double weight);

  const SpeedInfo GetHistorySpeed() const;
  common::Status EstimateSpeedUpperBound(
      const common::TrajectoryPoint& init_point, 
      const std::vector<std::pair<double, double>>& speed_limit_points,
      std::vector<double>* speed_upper_bound) const;

  bool AddDpStReferenceKernel(const double weight) const;

  // common::Status SmoothSpeedLimit( std::vector<std::pair<double, double>>& speed_limit_points);
  void ModifySpeedLimit(std::vector<double>& speed_upper_bound);
  void GetStMap(PathDecision* const path_decision);
  void GetYieldFollowAndCruiseWeightFactor(double& yield_weight_factor, 
        double& follow_weight_factor, double& cruise_weight_factor) const;

 private:

  struct compare_func {
    bool operator() (const std::pair<double, double>& a, const std::pair<double, double>& b) {
      return a.second < b.second;
    }
  
  };
  // solver
  math::Spline1dSolver* spline_solver_ = nullptr;

  // qp st configuration
  const QpStSpeedConfig qp_st_speed_config_;

  // initial status
  common::TrajectoryPoint init_point_;

  // is change lane
  bool is_change_lane_ = false;

  bool has_overtake_ = false;

  // t knots resolution
  double t_knots_resolution_ = 0.0;

  // knots
  std::vector<double> t_knots_;

  // evaluated t resolution
  double t_evaluated_resolution_ = 0.0;

  // evaluated points
  std::vector<double> t_evaluated_;

  // reference line kernel
  std::vector<double> cruise_;
  // reference speed kernel
  std::vector<double> reference_speed_points_;
  double speed_limit_max_ = 0.0;

  // reference st points from dp optimizer
  SpeedInfo reference_dp_speed_points_;

  planning_internal::STGraphDebug* st_graph_debug_ = nullptr;

  LatencyStats* latency_infos_ = nullptr; 

  const StGraphData& st_graph_data_;

  const CarModel vehicle_param_;

  std::vector<std::vector<std::pair<double, double>> > yield_st_map_;
  std::vector<std::vector<std::pair<double, double>> > yield_vt_map_;
  std::vector<std::vector<std::pair<double, double>> > follow_st_map_;
  std::vector<std::vector<std::pair<double, double>> > follow_vt_map_;
  std::vector<std::vector<std::pair<double, double>> > overtake_st_map_;
  std::vector<std::vector<std::pair<double, double>> > speed_limit_vt_map_;
  std::vector<std::vector<std::pair<double, double>> > speed_limit_st_map_;
  //size == t_evaluated_.size, 包含的元素为一个vector，这个vector中的元素为一个pair,pair的第一个元素为st_point的s，第二个元素为概率
  PathDecision* path_object_decision_;

};

}  // namespace planning
}  // namespace acu
