/**
 * @file piecewise_jerk_path_optimizer.h
 **/

#pragma once

#include <utility>
#include <vector>
#include "planning_config.pb.h"
#include "path_optimizer.h"
//#include "map/map_loader/include/publish_pointcloud.h"
//#include "map/map_loader/include/map_loader.h"
//#include "map/vectormap/src/hdmap/hdmap_util.h"
//#include "map/map_loader/include/show_vectormap.h"


namespace acu {
namespace planning {

typedef struct sPoint
{
  double x;
  double y;
  double xg;
  double yg;
  double angle;
  double head_err;
} sPoint;

class PiecewiseJerkPathOptimizer : public PathOptimizer {
 public:
  PiecewiseJerkPathOptimizer();
  bool Init(const PlanningConfig &config) override;

  virtual ~PiecewiseJerkPathOptimizer() = default;

 private:
  common::Status Process(const SpeedInfo& speed_data,
                         const ReferenceLine& reference_line,
                         const common::TrajectoryPoint& init_point,
                         PathInfo* const path_data) override;

  bool OptimizePath(
      const std::array<double, 3>& init_state,
      const std::array<double, 3>& end_state, 
      const double delta_s,
      const std::vector<std::pair<double, double>>& lat_boundaries,
      const std::vector<std::pair<double, double>>& dynamic_obstacle_boundaries,
      const std::vector<std::pair<double, double>>& dl_bounds,
      const std::vector<std::pair<double, double>>& ddl_bounds,
      const std::vector<std::pair<int, int>>& boundary_types,
      const std::array<double, 9>& w, std::vector<double>* ptr_x,
      std::vector<double>* ptr_dx, std::vector<double>* ptr_ddx,
      const int max_iter);

  FrenetFramePath ToPiecewiseJerkPath(const std::vector<double>& l,
                                      const std::vector<double>& dl,
                                      const std::vector<double>& ddl,
                                      const double delta_s,
                                      const double start_s,
                                      double &property_length) const;
  FrenetFramePath SLToFrenet(const std::vector<double>& l,
                              const std::vector<double>& dl,
                              const std::vector<double>& ddl,
                              const double delta_s,
                              const double start_s) const;

  double EstimateJerkBoundary(const double vehicle_speed,
                              const double axis_distance,
                              const double max_steering_rate) const;
  bool GenerateReferencePathData(double start_s,PathInfo *const path_data);
  void AddPathPlanStithingTrajectory(FrenetFramePath& frenet_frame_path); 
  bool IsIdeaLaneChangeDistanceInSolidLine(const double& distance) const;
  double GetPreferredLaneChangeDistance() const;
  private:
   PiecewiseJerkPathConfig config_;
   int id_rep_end;
   bool is_parking_ = false;
   bool is_set_meeting_x_ref_ = false;//是否根据决策的会车平移结果，设置优化目标
};

}  // namespace planning
}  // namespace acu
