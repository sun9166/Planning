/**
 * @file path_boundary.cpp
 **/

#include "path_boundary.h"

namespace acu {
namespace planning {

PathBoundary::PathBoundary(const double start_s, const double delta_s,
                           std::vector<std::pair<double, double>> path_boundary)
    : start_s_(start_s),
      delta_s_(delta_s),
      boundary_(std::move(path_boundary)) {}

double PathBoundary::start_s() const { return start_s_; }

double PathBoundary::delta_s() const { return delta_s_; }

void PathBoundary::set_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& PathBoundary::boundary() const {
  return boundary_;
}

void PathBoundary::set_dynamic_obstacle_boundary(const std::vector<std::pair<double, double>>& boundary) {
  dynamic_obstacle_boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& PathBoundary::dynamic_obstacle_boundary() const {
  return dynamic_obstacle_boundary_;
}

void PathBoundary::set_soft_boundary(const std::vector<std::tuple<double, double, double, double, std::string>>& boundary){
  soft_boundary_ = boundary;
}

const std::vector<std::tuple<double, double, double, double, std::string>>& PathBoundary::soft_boundary() const{
  return soft_boundary_;
}


void PathBoundary::set_boundary_type(const std::vector<std::pair<int, int>>& types) {
  boundary_type_ = types;
}
const std::vector<std::pair<int, int>>& PathBoundary::boundary_type() const {
  return boundary_type_;
}


void PathBoundary::set_label(const std::string& label) { label_ = label; }

const std::string& PathBoundary::label() const { return label_; }

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
  blocking_obstacle_id_ = obs_id;
}

const std::string& PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

void PathBoundary::set_is_blocked(const bool is_blocked) {
  is_blocked_ = is_blocked;

}

const bool& PathBoundary::is_blocked() const{
  return is_blocked_;
}

planning_debug_msgs::DebugSLBoundary PathBoundary::TransformBoundaryToMsg() const{
  planning_debug_msgs::DebugSLBoundary output;
   output.set_start_s(start_s());
  output.set_delta_s(delta_s());
  for(const auto& b : boundary()){
    planning_debug_msgs::LBounds l_bounds;
    l_bounds.set_right_boundary(b.first);
    l_bounds.set_left_boundary(b.second);
    output.add_boundary()->CopyFrom(l_bounds);
  }
  for(const auto& b : dynamic_obstacle_boundary()){
    planning_debug_msgs::LBounds l_bounds;
    l_bounds.set_right_boundary(b.first);
    l_bounds.set_left_boundary(b.second);
    output.add_dynamic_obstacle_boundary()->CopyFrom(l_bounds);
  }
  for(const auto& b : soft_boundary()){
    planning_debug_msgs::SoftBoundary soft_boundary;
    soft_boundary.mutable_l_bounds()->set_right_boundary(std::get<0>(b));
    soft_boundary.mutable_l_bounds()->set_left_boundary(std::get<1>(b));
    soft_boundary.set_probability(std::get<2>(b));
    soft_boundary.set_obj_occupy_delt_l(std::get<3>(b));
    soft_boundary.set_obj_id(std::get<4>(b));
    output.add_soft_boundary()->CopyFrom(soft_boundary);
  }
  for(const auto& b : boundary_type()){
    planning_debug_msgs::BoundaryType  boundary_type;
    boundary_type.set_right_bound_type(b.first);
    boundary_type.set_left_bound_type(b.second);
    output.add_boundary_type()->CopyFrom(boundary_type);
  }
  output.set_label(label());
  output.set_blocking_obstacle_id(blocking_obstacle_id());
  output.set_is_blocked(is_blocked());
  return output;
}

planning_debug_msgs::DebugSLBoundary PathBoundary::TransformBoundaryToMsg(
      const std::vector<std::pair<double, double>>& boundary) const{
  planning_debug_msgs::DebugSLBoundary output = TransformBoundaryToMsg();
  for(const auto& b : boundary){
    planning_debug_msgs::LBounds l_bounds;
    l_bounds.set_right_boundary(b.first);
    l_bounds.set_left_boundary(b.second);
    output.add_box_modified_boundary()->CopyFrom(l_bounds);
  }
  return output;
}


}  // namespace planning
}  // namespace acu
