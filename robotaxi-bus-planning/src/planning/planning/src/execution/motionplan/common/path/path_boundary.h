/**
 * @file path_boundary.h
 **/
#pragma once

#include <string>
#include <utility>
#include <vector>
#include <tuple>

//#include "planning_debug_msgs/BoundaryType.h"
//#include "planning_debug_msgs/LBounds.h"
//#include "planning_debug_msgs/SoftBoundary.h"
//#include "planning_debug_msgs/DebugSLBoundary.h"
#include "planning_debug_msgs.pb.h"

namespace acu {
namespace planning {

class PathBoundary {
 public:
  PathBoundary(const double start_s, const double delta_s,
               std::vector<std::pair<double, double>> path_boundary);

  virtual ~PathBoundary() = default;

  double start_s() const;

  double delta_s() const;

  void set_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& boundary() const;

  void set_dynamic_obstacle_boundary(const std::vector<std::pair<double, double>>& boundary);

  const std::vector<std::pair<double, double>>& dynamic_obstacle_boundary() const;

  void set_soft_boundary(const std::vector<std::tuple<double, double, double, double, std::string>>& boundary);

  const std::vector<std::tuple<double, double, double, double, std::string>>& soft_boundary() const;

  void set_boundary_type(const std::vector<std::pair<int, int>>& types);
  const std::vector<std::pair<int, int>>& boundary_type() const;

  void set_label(const std::string& label);
  const std::string& label() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;

  void set_is_blocked(const bool is_blocked);
  const bool& is_blocked() const;

  planning_debug_msgs::DebugSLBoundary TransformBoundaryToMsg() const;
  planning_debug_msgs::DebugSLBoundary TransformBoundaryToMsg(
    const std::vector<std::pair<double, double>>& boundary) const;

 private:
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::vector<std::pair<double, double>> boundary_;
  std::vector<std::pair<double, double>> dynamic_obstacle_boundary_;
  //soft_lmin, soft_lmax, probability, obj_occupy_delt_l,obj_id
  std::vector<std::tuple<double, double, double, double, std::string>> soft_boundary_;
  std::vector<std::pair<int, int>> boundary_type_;
  std::string label_ = "regular";
  std::string blocking_obstacle_id_ = "";
  bool is_blocked_ = false;
};

}  // namespace planning
}  // namespace acu
