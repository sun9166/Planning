#include "scenario_evaluation.h"

using geometry::Site;
using namespace acu::common;

namespace acu{
namespace planning {

ScenarioEvaluation::ScenarioEvaluation() {
  DP_ = DataPool::Instance()->GetMainDataPtr();
  reference_line_ = &DP_->cognition_info.struct_env_info.reference_line_info;
  scenario_info_ = &DP_->cognition_info.struct_env_info.scenario_info;
}

ScenarioEvaluation::~ScenarioEvaluation() {}

void ScenarioEvaluation::EvaluateScenario() {
  if (DP_->mapengine_data.map_info_data.alllinelists.empty() || 
      reference_line_->current_reference_line.empty()) {
    return;
  }
  for (auto& line : reference_line_->left_reference_line) {
    lines_.push_back(&line);
  }
  for (auto& line : reference_line_->current_reference_line) {
    if (line.reference_lane_id == reference_line_->current_line_id) {
      current_index_ = lines_.size();
      current_line_ = &line;
    }
    lines_.push_back(&line);
  }
  for (auto& line : reference_line_->right_reference_line) {
    lines_.push_back(&line);
  }
  EvaluateRoad();
  EvaluateObject();
}

void ScenarioEvaluation::EvaluateRoad() {
  lane_num_ = DP_->mapengine_data.map_info_data.alllinelists.size();
  dis_to_junction_ = current_line_->mapinfo.distance_to_junctions.size() ? 
      current_line_->mapinfo.distance_to_junctions.front().first : 1000.0;
  for (auto& map_line : DP_->mapengine_data.map_info_data.alllinelists) {
    if (dis_to_junction_ == 1000.0 && map_line.distance_to_junctions.size() && 
        map_line.distance_to_junctions.front().first < dis_to_junction_) {
      dis_to_junction_ = map_line.distance_to_junctions.front().first;
    }
  }
  for (auto& object : current_line_->objects_) {
    double left_w, right_w, s = object.second.sl_boundary.min_s;
    if (object.second.sl_boundary.min_s > current_line_->mapinfo.dis_to_end) {
      continue;
    }
    if (object.second.sl_boundary.min_s > 0.0) {
      current_line_->GetWidthToRoadBoundary(left_w, right_w, s);
      if (object.second.type < 2 && object.second.sl_boundary.min_l < left_w && 
          object.second.sl_boundary.max_l > -right_w) {
        on_road_[object.second.sl_boundary.min_s] = object.first;
      }
    }    
  }
  EvaluateRoadSide();
  if (dis_to_junction_ < 0.0) {
    EvaluateJunction();
  } else if (dis_to_junction_ < 150.0) {
    EvaluateNearJunction();
  } else {
    EvaluateMultiLanes();
  }
}

void ScenarioEvaluation::EvaluateRoadSide() {
  for (auto& object : current_line_->objects_) {
    double left_w, right_w, s = object.second.sl_boundary.min_s;
    if (object.second.sl_boundary.min_s > current_line_->mapinfo.dis_to_end) {
      continue;
    }
    if (object.second.sl_boundary.min_l < 0.0 && object.second.sl_boundary.min_s > 0.0) {
      current_line_->GetWidthToRoadBoundary(left_w, right_w, s);
      if (object.second.is_static && object.second.type < 2 &&
          object.second.sl_boundary.min_l - 0.3 < -right_w) {
        side_parking_[object.second.sl_boundary.min_s] = object.first;
      }
    } 
  }
  double left_w, right_road, right_bound;
  current_line_->GetWidthToRoadBoundary(left_w, right_road);
  current_line_->GetWidthToRoadBoundary(left_w, right_bound, 0.0, false);
  have_side_lane_ = right_bound > right_road + 1.0;
  if (side_parking_.empty() || have_side_lane_) {
    AWARN_IF(FLAGS_log_enable) << "side_parking size = " << side_parking_.size();
    return;
  } 
  bool side_park = true;
  if (lines_.back()->mapinfo.rightside_length > 0.0 && lines_.back()->line_queue) {
    double dis = side_parking_.rbegin()->first;
    for (auto it = side_parking_.rbegin(); it != side_parking_.rend(); ++it) {
      auto& object = current_line_->objects_[it->second];
      if (object.sl_boundary.max_s > dis_to_junction_ - 2.0 || object.was_dynamic) {
        side_park = false;
        break;
      } else if (dis - object.sl_boundary.max_s > 5.0) {
        break;
      }
      dis = object.sl_boundary.min_s;
    }
  }
  if (!side_park) {
    side_parking_.clear();
  }
  AWARN_IF(FLAGS_log_enable) << "side_park = " << side_park << " size = " << 
      side_parking_.size();
}

void ScenarioEvaluation::EvaluateJunction() {
  // reset flag
  scenario_info_->Reset();
}

void ScenarioEvaluation::EvaluateNearJunction() {
  if (scenario_info_->waiting_status > 1) {
    return;
  }
  int global_num = 0, queue_num = 0;
  if (current_line_->car_info_.size()) {
    auto& object = current_line_->objects_[current_line_->car_info_.begin()->second];
    if (object.was_dynamic && !side_parking_.count(object.sl_boundary.min_s)) {
      scenario_info_->waiting_status = 1;
      if (current_line_->line_queue) {
        scenario_info_->waiting_status = 2;
      }
    }      
  }
  for (auto line: lines_) {
    if (line->mapinfo.distance_to_junctions.size() && 
        line->mapinfo.distance_to_junctions.front().first < 1000.0) {
      global_num++;
      if (line->line_queue || line->car_info_.size() > 2) {
        queue_num++;
      } 
    }
  }
  if (queue_num > 1 || queue_num == global_num) {
    scenario_info_->waiting_status = 2;
  } 
  AWARN_IF(FLAGS_log_enable) << "queue_num = " << queue_num << " global_num = " << 
      global_num << " waiting_status = " << scenario_info_->waiting_status;
}

void ScenarioEvaluation::EvaluateMultiLanes() {
  double aver_num = 0.0, aver_speed = FLAGS_free_speed, sum_speed = 0.0;
  int lane_num = lane_num_;
  if (side_parking_.size() && !have_side_lane_) {
    lane_num = std::max(lane_num_ - 1, 1);
  }
  if (on_road_.size()) {
    aver_num = on_road_.size() / lane_num;
    for (auto it = on_road_.begin(); it != on_road_.end(); it++) {
      sum_speed += current_line_->objects_[it->second].speed;
    }
    aver_speed = sum_speed / on_road_.size();
  } 
  scenario_info_->aver_num = 0.5 * (scenario_info_->aver_num + aver_num);
  scenario_info_->aver_speed = 0.5 * (scenario_info_->aver_speed + aver_speed);
  if (aver_num < 1) {
    scenario_info_->congestion_level = 0;
  } else if (aver_num < 2) {
    scenario_info_->congestion_level = 1;
  } else {
    scenario_info_->congestion_level = 2;
  }
  AWARN_IF(FLAGS_log_enable) << "aver_num = " << aver_num << " speed = " << aver_speed;
}

void ScenarioEvaluation::EvaluateObject() {
  EvaluateInvader();
  EvaluateWaiting();
  for (auto it = on_road_.begin(); it != on_road_.end(); it++) {
    auto& object = current_line_->objects_[it->second];
    object.driving_status = 0;
    if (side_parking_.count(it->first) || object.type == 4) {
      object.driving_status = 1;
    } else if (object.dis_to_junction < 1.0) {
      object.driving_status = 2;
    } else if (scenario_info_->waiting_status > 1) {
      if (object.sl_boundary.max_s < dis_to_junction_) {
        object.driving_status = 3;
      }
    } else if (scenario_info_->congestion_level > 1) {
      object.driving_status = 4;
    } else if (scenario_info_->invader_.count(object.id)) {
      object.driving_status = 5;
    }
  }
}

void ScenarioEvaluation::EvaluateInvader() {
  scenario_info_->invader_.clear();
  if (dis_to_junction_ < FLAGS_front_perception_range || 
      current_line_->invader_on_line_.empty()) {
    AWARN_IF(FLAGS_log_enable) << "There's no invader!";
    return;
  }
  int id = current_line_->invader_on_line_.begin()->first;
  bool is_left = current_line_->objects_[id].pass_l > 0.0;
  double min_s = FLAGS_front_perception_range, max_s = 0.0;
  for (auto invader : current_line_->invader_on_line_) {
    auto& object = current_line_->objects_[invader.first];
    if (object.pass_l > 0.0 != is_left) {
      AWARN_IF(FLAGS_log_enable) << "Invader direction is different!";
      return;
    }
    if (invader.second > 20) {
      min_s = std::min(min_s, object.sl_boundary.min_s);
      max_s = std::max(max_s, object.sl_boundary.min_s);
      scenario_info_->invader_[object.id] = current_line_->objects_[id].pass_l;
      AWARN_IF(FLAGS_log_enable) << "invader_id = " << object.id << " l = " << 
          current_line_->objects_[id].pass_l;
    }
  }
  if (scenario_info_->invader_.empty() || min_s > 40.0) {
    scenario_info_->invader_.clear();
    AWARN_IF(FLAGS_log_enable) << "Invader is far or unstable!";
    return;
  }
  for (auto block : current_line_->block_l_) {
    auto& object = current_line_->objects_[block.first]; 
    if (current_line_->invader_on_line_.count(object.id) || 
        object.sl_boundary.min_s > max_s + 10.0 && 
        object.speed > FLAGS_speed_threshold) {
      continue;
    }
    AWARN_IF(FLAGS_log_enable) << "There's object in front of invader!";
    return;
  }
  scenario_info_->enable_offset_ = false;
  int line_index = is_left ? current_index_ - 1 : current_index_ + 1;
  AERROR_IF(FLAGS_log_enable) << "current_index_ = " << current_index_ << " target = " << line_index;
  if (line_index >= 0 && line_index < lines_.size()) {
    scenario_info_->enable_offset_ = true;
    for (auto& car : lines_[line_index]->car_info_) {
      if (!lines_[line_index]->objects_[car.second].is_static) {
        scenario_info_->enable_offset_ = false;
        break;
      }
    }
  }
  AWARN_IF(FLAGS_log_enable) << "enable_offset_ = " << scenario_info_->enable_offset_;
}

void ScenarioEvaluation::EvaluateWaiting() {
  scenario_info_->waiting_objects_.clear();
  if (dis_to_junction_ > 120.0 || dis_to_junction_ < 1e-3) {
    AWARN_IF(FLAGS_log_enable) << "dis_to_junction = " << dis_to_junction_;
    return;
  }
  for (auto& obj : current_line_->objects_) {
    auto& object = obj.second;
    if (object.is_static && object.type < 2 && object.dis_to_junction < 80.0 && 
        object.sl_boundary.min_s < dis_to_junction_) {
      if (side_parking_.count(object.id)) {
        AWARN_IF(FLAGS_log_enable) << "parking_object = " << object.id;
      } else {
        scenario_info_->waiting_objects_.insert(object.id);
        AWARN_IF(FLAGS_log_enable) << "waiting_object = " << object.id;
      }
    }
  }
}

} // namespace planning
} // namespace acu
