/*
 *  @file: st_graph_point.h
 */

#pragma once

#include <limits>
#include "../speed/st_point.h"

namespace acu {
namespace planning {

class StGraphPoint {
 public:
  std::uint32_t index_s() const;
  std::uint32_t index_t() const;

  const STPoint& point() const;
  const StGraphPoint* pre_point() const;

  double reference_cost() const;
  double obstacle_cost() const;
  double total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const STPoint& st_point);

  // given reference speed profile, reach the cost, including position
  void SetReferenceCost(const double reference_cost);

  // given obstacle info, get the cost;
  void SetObstacleCost(const double obs_cost);

  // total cost
  void SetTotalCost(const double total_cost);

  void SetPrePoint(const StGraphPoint& pre_point);

 private:
  STPoint point_;
  const StGraphPoint* pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;

  double reference_cost_ = 0.0;
  double obstacle_cost_ = 0.0;
  double total_cost_ = std::numeric_limits<double>::infinity();
};

}  // namespace planning
}  // namespace acu
