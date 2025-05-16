/**
 * @file path_boundary.cpp
 **/

#include "path_decision.h"
#include "common/util/util.h"

namespace acu {
namespace planning {

Obstacle *PathDecision::AddObstacle(const Obstacle &obstacle) {
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  return obstacles_.Add(obstacle.Id(), obstacle);
}

Obstacle *PathDecision::AddWholeObstacle(const Obstacle &obstacle) {
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  return whole_obstacles_.Add(obstacle.Id(), obstacle);
}

const IndexedObstacles &PathDecision::obstacles() const { return obstacles_; }

const IndexedObstacles &PathDecision::whole_obstacles() const { return whole_obstacles_; }

Obstacle *PathDecision::Find(const std::string &object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *PathDecision::Find(const std::string &object_id) const {
  return obstacles_.Find(object_id);
}

void PathDecision::SetStBoundary(const std::string &id,
                                 const StBoundary &boundary) {
  auto *obstacle = obstacles_.Find(id);

  if (!obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->SetStBoundary(boundary);
  }
}
void PathDecision::SetNudgeBuffer(const std::string &id, const double &buffer) {
  auto *obstacle = obstacles_.Find(id);
  if (!obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to find obstacle : " << id;
    return;
  } else {
    obstacle->SetNudgeLatBufferMin(buffer);
  }

}

bool PathDecision::AddLateralDecision(const std::string &tag,
                                      const std::string &object_id,
                                      const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion) << "[PathDecision/AddLateralDecision]failed to find obstacle";
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}

void PathDecision::EraseStBoundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
    obstacle_ptr->EraseStBoundary();
  }
}

bool PathDecision::AddLongitudinalDecision(const std::string &tag,
                                           const std::string &object_id,
                                           const ObjectDecisionType &decision) {
  auto *obstacle = obstacles_.Find(object_id);
  if (!obstacle) {
    AWARN_IF(FLAGS_enable_debug_motion) << "failed to find obstacle";
    return false;
  }
  obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool PathDecision::MergeWithMainStop(const ObjectStop &obj_stop,
                                     const std::string &obj_id,
                                     const ReferenceLine &reference_line,
                                     const SLBoundary &adc_sl_boundary) {
  PointENU stop_point = obj_stop.stop_point();
  common::SLPoint stop_line_sl;
  reference_line.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);

  double stop_line_s = stop_line_sl.s();
  if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Ignore object:" << obj_id << " fence route_s[" << stop_line_s
           << "] not in range[0, " << reference_line.Length() << "]";
    return false;
  }

  // check stop_line_s vs adc_s, ignore if it is further way than main stop
  const double kStopBuff = 1.0;
  stop_line_s = std::fmax(stop_line_s, adc_sl_boundary.end_s() - kStopBuff);

  if (stop_line_s >= stop_reference_line_s_) {
    return false;
  }

  main_stop_.Clear();
  main_stop_.set_reason_code(obj_stop.reason_code());
  main_stop_.set_reason("stop by " + obj_id);
  main_stop_.mutable_stop_point()->set_x(obj_stop.stop_point().x());
  main_stop_.mutable_stop_point()->set_y(obj_stop.stop_point().y());
  main_stop_.set_stop_heading(obj_stop.stop_heading());
  stop_reference_line_s_ = stop_line_s;
  return true;
}

bool PathDecision::GetMainFocusObstacle() {
  double distance = std::numeric_limits<double>::max();
  for (auto& obstacle : obstacles_.Items()) {
    if (obstacle->HasLongitudinalDecision()) {
      const auto& decision = obstacle->LongitudinalDecision();
      if ((decision.has_stop() || decision.has_follow()) && 
        obstacle->PerceptionSLBoundary().end_s() < distance) {
        distance = obstacle->PerceptionSLBoundary().start_s();
        main_focus_obstacle_ = Find(obstacle->Id());
      }
    }
  }
  if (distance < std::numeric_limits<double>::max()) {
    return true;
  } else {
    main_focus_obstacle_ = nullptr;
    return false;
  }
}

bool PathDecision::GetMainYieldObstacle() {
  double distance = std::numeric_limits<double>::max();
  for (auto& obstacle : obstacles_.Items()) {
    if (obstacle->HasLongitudinalDecision() && !obstacle->IsVirtual()) {
      const auto& decision = obstacle->LongitudinalDecision();
      if (decision.has_yield() && 
        obstacle->PerceptionSLBoundary().start_s() < distance) {
        distance = obstacle->PerceptionSLBoundary().start_s();
        main_yield_obstacle_ = Find(obstacle->Id());
      }
    }
  }
  if (distance < std::numeric_limits<double>::max()) {
    return true;
  } else {
    main_yield_obstacle_ = nullptr;
    return false;
  }
}

bool PathDecision::has_overtake_obstacle() const {
  for (auto& obstacle : obstacles_.Items()) {
    if (obstacle->HasLongitudinalDecision() && !obstacle->IsVirtual()) {
      const auto& decision = obstacle->LongitudinalDecision();
      if (decision.has_overtake()) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace planning
}  // namespace acu
