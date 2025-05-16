/**
 * @file path_bounds_decider.h
 **/

#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "planning_config.pb.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/planner/procedure.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"
#include "src/execution/motionplan/common/path/path_boundary.h"
#include "map/basemap/include/basemap.h"
#include "src/execution/motionplan/common/datatype.h"

namespace acu {
namespace planning {

class PathBoundsDecider : public Procedure {
 public:
  // explicit PathBoundsDecider(const TaskConfig& config);
  PathBoundsDecider();
  bool Init(const PlanningConfig &config) override;
  acu::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  /** @brief Every time when Process function is called, it will:
   *   1. Initialize.
   *   2. Generate Fallback Path Bound.
   *   3. Generate Regular Path Bound(s).
   */
  common::Status Process(Frame* frame,
                         ReferenceLineInfo* reference_line_info);

  /////////////////////////////////////////////////////////////////////////////
  // Below are functions called every frame when executing PathBoundsDecider.

  /** @brief The initialization function.
   */
  void InitPathBoundsDecider(const Frame& frame,
                             const ReferenceLineInfo& reference_line_info);
 private:
  std::pair<std::array<double, 3>, std::array<double, 3>> start_frenet_state_;
  double adc_frenet_s_ = 0.0;
  double adc_lane_width_ = 0.0;
  PathBoundsDeciderConfig config_;
  
  // uint32_t num_of_time_stamps_ = 0;
  // double eval_time_interval_ = 0.1;
  // double total_time_ = 7.0;
  // std::vector<std::pair<int, int>> boundary_type_;
  // SpeedInfo heuristic_speed_data_;
  // std::vector<common::SpeedPoint> heuristic_speed_;
  // std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
  // std::vector<std::vector<SLBoundary>> dynamic_obstacle_sl_boundaries_;
  // //soft_lmin, soft_lmax, probability, obj_occupy_delt_l,obj_id
  // std::vector<std::tuple<double, double, double, double, std::string>> soft_boundary_;
  // LaneBound dynamic_bound_;
  // acu::map::BaseMap* bmp_ = acu::map::MapLoader::GetBasemapPtr();

};

}  // namespace planning
}  // namespace acu
