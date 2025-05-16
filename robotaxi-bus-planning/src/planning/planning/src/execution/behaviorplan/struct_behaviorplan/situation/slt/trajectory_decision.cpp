#include "trajectory_decision.h"

namespace acu {
namespace planning {

TrajectoryDecision::TrajectoryDecision() {
  context_->decision_result_.meeting_ids.clear();
  context_->decision_result_.dynamic_sl.clear();
  context_->decision_result_.expand_l = -1.0;
  context_->final_path_.clear();
}

TrajectoryDecision::~TrajectoryDecision() {}

void TrajectoryDecision::MakeMeetingDecision() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  AERROR << "meeting_size = " << current_line_->meeting_objects_.size();
  if (context_->decision_result_.reference_line_id >= 20 || 
      current_line_->meeting_objects_.empty()) {
    return;
  }
  for (auto& object : current_line_->meeting_objects_) {
    context_->decision_result_.meeting_ids.insert(object);
  }
  context_->decision_result_.expand_l = 0.0;
  TrajectoryGeneration generator;
  if (generator.GenerateSLTTrajectory()) {
    GetSLAndBoundary();
    return;
  }
  context_->decision_result_.dynamic_sl.clear();
  DynamicSLGeneration sl_generator;
  sl_generator.CalculateDynamicBoxesWithCollision();
  context_->decision_result_.dynamic_sl = sl_generator.GetResultDynamicObstacleBoxes();
  context_->decision_result_.meeting_ids.clear();
  // PathGeneration path_generator;
  // path_generator.GeneratePath();
}

void TrajectoryDecision::GetSLAndBoundary() {
  std::map<double, double> time_s;
  double t = 0.0;
  auto& car_model = context_->planning_config_.car_model;
  const double delta_t = 0.2;
  for (int i = 1; i < context_->final_path_.size(); i++) {
    auto& point = context_->final_path_[i];
    while (t < point.t) {
      time_s[t] = point.s + point.v * (t - point.t);
      t += delta_t;
    }
  }
  for (auto& t_s : time_s) {
    for (auto& dynamic_object : context_->dynamic_objects_) {
      if (dynamic_object.second.lower_bound(t_s.first) == 
          dynamic_object.second.end()) {
        continue;
      }
      auto& box_sl = dynamic_object.second.lower_bound(t_s.first)->second;
      if (box_sl.second.min_s <= t_s.second + car_model.front_over_hang && 
          box_sl.second.max_s >= t_s.second - car_model.back_over_hang) {
        context_->decision_result_.dynamic_sl.push_back(box_sl.first);
      }
    }
  }
  double left_w, right_w;
  for (auto& point : context_->final_path_) {
    current_line_->GetWidthToLaneBoundary(left_w, right_w, point.s);
    right_w += FLAGS_boundary_width;
    double expand_l = std::max(-(double)point.l, right_w - 0.5 * car_model.car_width);
    if (expand_l > context_->decision_result_.expand_l) {
      context_->decision_result_.expand_l = expand_l;
    }
  }
  if (context_->decision_result_.dynamic_sl.empty()) {
    for (auto& dynamic_object : context_->dynamic_objects_) {
      for (auto& box_sl : dynamic_object.second) {
        context_->decision_result_.dynamic_sl.push_back(box_sl.second.first);
      }
    }
    // double left_w, right_w;
    // current_line_->GetWidthToRoadBoundary(left_w, right_w);
    // context_->decision_result_.expand_l = right_w;
    context_->decision_result_.expand_l = 11.0;
  }
  AERROR << "expand_l = " << context_->decision_result_.expand_l << 
      " dynamic_sl_size = " << context_->decision_result_.dynamic_sl.size();
}

}  //  namespace planning
}  //  namespace acu
