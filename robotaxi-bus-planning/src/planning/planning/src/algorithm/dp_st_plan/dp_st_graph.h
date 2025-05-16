/**
 * @file dp_st_graph.h
 **/

#pragma once

#include <memory>
#include <vector>
#include <future>

#include "dp_st_speed_config.pb.h"
#include "planning_config.pb.h"

#include "dp_st_cost.h"
#include "src/execution/motionplan/common/st_graph/st_graph_point.h"
#include "common/common_header/status.h"
#include "src/execution/motionplan/common/frame/frame.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"
#include "src/execution/motionplan/common/speed/st_point.h"
#include "src/execution/motionplan/common/speed/speed_info.h"
#include "src/execution/motionplan/common/st_graph/st_graph_data.h"

namespace acu {
namespace planning {

class DpStGraph {
 public:
  DpStGraph(const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,
            const std::vector<const Obstacle*>& obstacles,
            const common::TrajectoryPoint& init_point,
            const SLBoundary& adc_sl_boundary);

  acu::common::Status Search(SpeedInfo* const speed_data);

 private:
  acu::common::Status InitCostTable();

  acu::common::Status RetrieveSpeedProfile(SpeedInfo* const speed_data);

  acu::common::Status CalculateTotalCost();

  // defined for cyber task
  struct StGraphMessage {
    StGraphMessage(const uint32_t c_, const int32_t r_) : c(c_), r(r_) {}
    uint32_t c;
    uint32_t r;
  };
  void CalculateCostAt(const std::shared_ptr<StGraphMessage>& msg);

  double CalculateEdgeCost(const STPoint& first, const STPoint& second,
                          const STPoint& third, const STPoint& forth,
                          const double speed_limit);
  double CalculateEdgeCostForSecondCol(const uint32_t row,
                                      const double speed_limit);
  double CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                     const uint32_t pre_r,
                                     const double speed_limit);

  void GetRowRange(const StGraphPoint& point, size_t* highest_row,
                   size_t* lowest_row);

 private:
  const StGraphData& st_graph_data_;

  // dp st configuration
  DpStSpeedConfig dp_st_speed_config_;

  // obstacles based on the current reference line
  const std::vector<const Obstacle*>& obstacles_;

  //@pqg 设置临时的最大加速度限制；
  double max_acceleration_ = 2.5;
  double max_deceleration_ = -6.0;

  // initial status
  common::TrajectoryPoint init_point_;

  // cost utility with configuration;
  DpStCost dp_st_cost_;

  const SLBoundary& adc_sl_boundary_;

  double unit_s_ = 0.0;
  double unit_t_ = 0.0;

  // cost_table_[t][s]
  // row: s, col: t --- NOTICE: Please do NOT change.
  std::vector<std::vector<StGraphPoint>> cost_table_;
};

}  // namespace planning
}  // namespace acu
