#include "behavior_intention.h"

namespace acu {
namespace planning {

BehaviorIntention::BehaviorIntention() {}

BehaviorIntention::~BehaviorIntention() {}

bool BehaviorIntention::Init() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  AWARN_IF(FLAGS_log_enable) << "cur = " << reference_line_->current_line_id << 
      " target = " << reference_line_->target_line_id << " ego_l = " << 
      current_line_->mapinfo.dis2line << " v = " << context_->ego_speed_;
  bool road_side = current_line_->IsRoadSide();
  AWARN_IF(FLAGS_log_enable) << "road_side = " << road_side;
  if (context_->cognition_info_->vehicle_info.chassis.drive_state < 1) {
    if (context_->cognition_info_->vehicle_info.chassis.velocity < 0.1 && road_side) {
      AWARN_IF(FLAGS_log_enable) << "Roadside Parking!";
      context_->decision_result_.reference_line_id = 0;
      context_->departure_counter_ = 0;
    } else {
      context_->decision_result_.reference_line_id = reference_line_->current_line_id;
    }
    context_->replan_state_ = eReplanStateEnum::CRUISE;
    context_->stable_counter_ = 0;
    AWARN_IF(FLAGS_log_enable) << "In standby mode!";
    return false;
  } else if (road_side == false) {
    context_->departure_counter_ = FLAGS_start_yield_frame + 1;
  } else if (context_->departure_counter_ < FLAGS_start_yield_frame) {
    if (CheckStartCondition()) {
      context_->departure_counter_++;
    } else {
      context_->departure_counter_ = 0;
    }
    context_->decision_result_.reference_line_id = 0;
    AINFO_IF(FLAGS_log_enable) << "departure_counter = " << context_->departure_counter_;
  } else if (context_->departure_counter_ == FLAGS_start_yield_frame) {
    CheckRearCar();
  }
  return true;
}

bool BehaviorIntention::CheckStartCondition() {
  double left_w, right_w;
  current_line_->GetWidthToLaneBoundary(left_w, right_w);
  
  auto& car_model = context_->planning_config_.car_model;
  for (auto& obj : current_line_->objects_) {
    auto& object = obj.second;
    if (object.type == 2 && object.sl_boundary.min_l < left_w && 
        object.sl_boundary.min_s < car_model.front_over_hang + 3.0 && 
        object.sl_boundary.max_s > -car_model.back_over_hang - 3.0) {
      AWARN_IF(FLAGS_log_enable) << "Can't start for " << object.id << " nearby";
      return false;
    }
    if (object.type == 3 && object.sl_boundary.min_l < left_w && 
        object.sl_boundary.min_s < car_model.front_over_hang && 
        object.vsabs > 0.0 && fabs(object.sl_boundary.min_s) / object.vsabs < 6.0) {
      AWARN_IF(FLAGS_log_enable) << "Can't start for " << object.id << " rear";
      return false;
    }
    double min_t = 10.0;
    for (auto& trajectory : object.prediction.trajectories) {
      if (trajectory.st_boundary.size()) {
        min_t = std::min(trajectory.st_boundary.front().first.y(), min_t);
        AERROR << object.id << " min_t = " << trajectory.st_boundary.front().first.y();
      }
    }
    if (object.sl_boundary.min_s < car_model.front_over_hang && min_t < 6.0) {
      AWARN_IF(FLAGS_log_enable) << "Can't start for " << object.id << " st";
      context_->departure_counter_ = 0;
      return false;
    }
  }
  return true;
}

void BehaviorIntention::CheckRearCar() {
  const double ego_acc = 0.7;
  const double object_dec = -2.0;
  auto& local_line = reference_line_->local_reference_line;
  auto& car_model = context_->planning_config_.car_model;
  for (auto& obj : current_line_->objects_) {
    auto& object = obj.second;
    double min_t = 10.0;
    for (auto& trajectory : object.prediction.trajectories) {
      if (trajectory.st_boundary.size()) {
        min_t = std::min(trajectory.st_boundary.front().first.y(), min_t);
        AERROR << object.id << " min_t = " << trajectory.st_boundary.front().first.y();
      }
    }
    if (local_line.objects_.count(object.id)) {
      for (auto& trajectory : local_line.objects_[object.id].prediction.trajectories) {
        if (trajectory.st_boundary.size()) {
          min_t = std::min(trajectory.st_boundary.front().first.y(), min_t);
          AERROR << object.id << " min_t = " << trajectory.st_boundary.front().first.y();
        }
      }
    }
    if (object.is_static || object.speed < context_->ego_speed_ || 
        object.sl_boundary.min_s > car_model.front_over_hang || 
        object.type == 2 || min_t > 6.0) {
      continue;
    }
    double t = (object.speed - context_->ego_speed_) / (ego_acc - object_dec);
    double ego_s = context_->ego_speed_ * t + 0.5 * ego_acc * t * t;
    double object_s = object.speed * t + 0.5 * object_dec * t * t;
    double safe_dis = std::min(FLAGS_side_risk_range, 
        context_->ego_speed_ + fabs(current_line_->mapinfo.dis2line));
    AERROR << "t = " << t << " ego_s = " << ego_s << " object_s = " << object_s;
    if (fabs(object.sl_boundary.max_s) / (object.speed - context_->ego_speed_) < 2.0 || 
        ego_s < object_s + object.sl_boundary.max_s + safe_dis) {
      AWARN_IF(FLAGS_log_enable) << "start collision " << object.id << " at " << t;
      double stop_s = std::min(current_line_->mapinfo.dis_to_end, 
          pow(context_->ego_speed_, 2) / 2.0 + car_model.front_over_hang);
      context_->stop_s_ = std::min(context_->stop_s_, stop_s);
      int index;
      Site stop_point;
      current_line_->GetNearestPoint(stop_s, stop_point, index);
      ObjectEvaluation object;
      object.AddStopPoint(stop_point);
      return;
    }
  }
}

bool BehaviorIntention::HavePullOver() {
  if (current_line_->mapinfo.first_lc_time == 0 && 
      context_->dis_to_mission_ < FLAGS_pull_over_dis && 
      context_->dis_to_mission_ > FLAGS_min_lc_dis / 2.0) {
    return true;
  }
  context_->pull_over_ = false;
  return false;
}

bool BehaviorIntention::HaveDeparture() {
  if (context_->departure_counter_ == FLAGS_start_yield_frame) {
    return fabs(current_line_->mapinfo.dis2line) > 1.0;
  }
  return false;
}

bool BehaviorIntention::HaveAbandonLC() {
  int target_id = reference_line_->target_line_id;
  if (target_id < 20) {
    return false;
  } else if (!context_->reference_line_map_.count(target_id)) {
    AWARN_IF(FLAGS_log_enable) << "Can't find target line!";
    return false;
  } else if(context_->pedal_command_) {
    AWARN_IF(FLAGS_log_enable) << "Ignore safety for now!";
    return false;
  } else if (current_line_->dis_to_last_lc < FLAGS_min_lc_dis) {
    AWARN_IF(FLAGS_log_enable) << "There's no enough space to abandon lc!";
    return false;
  } else {
    for (auto& block :current_line_->block_l_) {
      if (current_line_->objects_.count(block.first)) {
        auto& object = current_line_->objects_[block.first];
        if (object.is_static && block.second < 0.2 && 
            object.sl_boundary.min_s < FLAGS_min_lc_dis) {
          AWARN_IF(FLAGS_log_enable) << "There's no enough space to abandon lc!";
          return false;
        }
      }
    }
  }
  double left_width, right_width, width;
  auto target_line = context_->reference_line_map_[target_id];
  target_line->GetWidthToLaneBoundary(left_width, right_width);
  width = target_id < 30 ? right_width : left_width;
  width += 0.5 * context_->planning_config_.car_model.car_width;
  if (fabs(target_line->mapinfo.dis2line) < width) {
    return false;
  }
  for (auto it = target_line->objects_.begin(); it != target_line->objects_.end(); it++) {
    auto &object = it->second;
    if (object.conflict_type != 1 || object.sl_boundary.max_s > 0.0 || 
        object.is_static || object.speed <= context_->ego_speed_) {
      continue;
    }
    // Collision before ego and object have same speed
    double t = (object.speed - context_->ego_speed_) / 
        (FLAGS_recommended_planning_acc - FLAGS_recommended_planning_dec);
    double ego_s = context_->ego_speed_ * t + 0.5 * FLAGS_recommended_planning_acc * t * t;
    double object_s = object.speed * t + 0.5 * FLAGS_recommended_planning_dec * t * t;
    double safe_dis = std::min(context_->ego_speed_, FLAGS_side_risk_range);
    AERROR << "t = " << t << " ego_s = " << ego_s << " object_s = " << object_s;
    if (fabs(object.sl_boundary.max_s) / (object.speed - context_->ego_speed_) < 2.0 || 
        ego_s < object_s + object.sl_boundary.max_s + safe_dis) {
      AWARN_IF(FLAGS_log_enable) << "collision_id = " << object.id << " at " << t; 
      return true;
    }
  }
  return false;
}

bool BehaviorIntention::HaveCommandLC() {
  if (context_->lc_command > 0) {
    if (reference_line_->target_line_id >= 20) {
      DataPool::Instance()->GetMainDataRef().debug_planning_msg.decision.set_cmd_feedback( 
        "Abandon lc command while lane changing!");
      DataPool::Instance()->GetMainDataRef().debug_planning_msg.decision.set_lc_status(-1);
      return false;
    }
    return true;
  }
  return false;
}

bool BehaviorIntention::HaveMissionLC() {
  if (reference_line_->target_line_id >= 20 || 
      context_->replan_state_ == eReplanStateEnum::EXCUTE) {
    return false;
  } 
  if (current_line_->mapinfo.all_lc_time == 1000) {
    return true;
  }
  if (context_->scenario_type == eScenarioEnum::JUNCTION) {
    return false;
  }
  int lc_time = fabs(current_line_->mapinfo.first_lc_time);
  double dis_to_end = current_line_->mapinfo.dis_to_end;
  dis_to_end = std::min(dis_to_end, context_->dis_to_mission_);
  if (lc_time) {
    AWARN_IF(FLAGS_log_enable) << "average_dis = " << dis_to_end / lc_time;
    return dis_to_end / lc_time < FLAGS_expected_lc_dis;
  }  
  return false;
}

bool BehaviorIntention::HaveObstacleAvoidance() {
  return reference_line_->local_reference_line.line_blocked;
}

bool BehaviorIntention::HaveSpeedLC() {
  if (reference_line_->target_line_id >= 20 || 
      reference_line_->path_in_current == false) {
    return false;
  }
  //  else if (current_line_->mapinfo.rightside_length > 0.0 && 
  //     context_->dis_to_junction_ > 100.0 && 
  //     reference_line_->left_reference_line.size() && 
  //     reference_line_->left_reference_line.back().mapinfo.dis_to_end > 200.0) {
  //   return true;
  // }
  // return current_line_->line_slow;
  return current_line_->speed_cost > 1.0;
}

bool BehaviorIntention::HaveOffsetCenter() {
  if (current_line_->meeting_car) {
    return false;
  }
  if (context_->trajectory_info_.path.size() && 
      context_->trajectory_info_.path.front().offset_property == 2) {
    return true;
  }
  return false;
}

bool BehaviorIntention::HaveMeeting() {
  if (current_line_->meeting_car) {
    auto id = current_line_->meeting_id;
    if (id > 0 && id != context_->decision_result_.giveway_id && 
        current_line_->meeting_state > 1) {
      context_->decision_result_.giveway_id = id;
      AWARN << "giveway_id = " << id;
      return true;
    }
  } else if (context_->decision_result_.giveway_id > 0) {
    context_->decision_result_.giveway_id = -1;
    return true;
  }
  return false;
}

}  //  namespace planning
}  //  namespace acu
