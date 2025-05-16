#include "prob_grid_search.h"

namespace acu {
namespace planning {

ProbGridSearch::ProbGridSearch() {}

ProbGridSearch::~ProbGridSearch() {}

bool ProbGridSearch::GetSearchResult() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = context_->reference_line_map_[context_->decision_result_.reference_line_id];
  search_line_ = reference_line_->path_in_current ? 
      current_line_ : &reference_line_->local_reference_line;
  path_length_ = std::min(search_line_->mapinfo.dis_to_end, context_->stop_s_);
  AINFO_IF(FLAGS_log_enable) << "path_in_current = " << reference_line_->path_in_current;
  AddSpeedLimit();
  ObjectManager manager;
  manager.AddObjectInfo();
  if (GenerateTrajectory()) {
    GetFinalDecision();
    GetObjectDecision();
    context_->decision_result_.object_decision_success = 1;
    return true;
  } 
  context_->decision_result_.object_decision_success = -1;
  return false;
}

void ProbGridSearch::AddSpeedLimit() {
  if (search_line_->mapinfo.expected_speeds.empty()) {
    return;
  }
  std::map<double, double> speed_limit;
  for (auto& speed : search_line_->mapinfo.expected_speeds) {
    speed_limit[speed.first] = speed.second;
    // AERROR << "s = " << speed.first <<  " speed_limit = " << speed.second;
  }
  int step = 0;
  for (auto it = context_->trajectory_info_.path.begin(); it != context_->trajectory_info_.path.end(); it++) {
    if (it->length >= path_length_ || 
        speed_limit.lower_bound(it->length) == speed_limit.end()) {
      break;
    } else if (it->length < double(step)) {
      continue;
    }
    step++;
    double lane_limit = speed_limit.lower_bound(it->length)->second;
    double curv_limit = sqrt(1.2 / std::min(fabs(it->curvature), 1.0));
    speed_limit_[it->length] = (std::min(lane_limit, curv_limit));
    // if (curv_limit < lane_limit) {
    //   AERROR << " s = " << it->length <<  " curv_limit = " << 
    //       curv_limit << " lane_limit = " << lane_limit;
    // }
  }
}

bool ProbGridSearch::GenerateTrajectory() {
  std::map<float, STNodeStruct> parent;
  STNodeStruct node;
  node.v = context_->ego_speed_;
  node.a = context_->cognition_info_->vehicle_info.chassis.vehicle_accel;
  for (auto& point : context_->trajectory_info_.path) {
    if (point.t > 0.2) {
      node.v = point.velocity;
      node.a = point.a;
      if( fabs(node.a + 1.2) < 0.00001) {
        AERROR<< "-----------GenerateTrajectory GenerateTrajectory--------------";
        //sleep(1000000000000000);
      }      
      break;
    }
  }
  node.total_cost = 0.0;
  parent[0.0] = node;
  points_.push_back(parent);
  const int step_t = (int)(1.0 / FLAGS_scale_t);

  const int range_t = search_line_->GetSTMapRangeT();
  const int range_s = search_line_->GetSTMapRangeS();
  auto st_map_ptr = search_line_->GetSTMapPtr();
  for (int t = step_t; t < range_t; t += step_t) {
    std::map<float, STNodeStruct> child;
    float min_s = parent.begin()->first + parent.begin()->second.v + FLAGS_max_dec;
    float max_s = parent.rbegin()->first + parent.rbegin()->second.v + FLAGS_max_acc;
    if (t > 20) {
      t += step_t;
      min_s += parent.begin()->second.v + 2.0 * FLAGS_max_dec;
      max_s += parent.rbegin()->second.v + 2.0 * FLAGS_max_acc;
    }
    for (int s = 0; s < range_s; s++) {
      if (t > 20 && s > 0) {
        s++;
      }
      node.s = FLAGS_scale_s * s;
      if (node.s > max_s || node.s > path_length_) {
        break;
      } else if (node.s < min_s || st_map_ptr[t][s].p > FLAGS_p_threshold) {
        continue;
      } 
      node.t = FLAGS_scale_t * t;
      node.s_id = s;
      node.t_id = t;
      node.total_cost = MAX_COST;
      for (auto it = parent.begin(); it != parent.end(); it++) {
        CalculateCost(it->second, node);
      }
      if (node.total_cost < MAX_COST) {
        child[node.s] = node;
      }
    }
    points_.push_back(child);
    parent = child;
    // AERROR_IF(FLAGS_log_enable) << "t = " << t << " size = " << child.size();
    if (child.empty()) {
      return false;
    }
  }
  return true;
}

void ProbGridSearch::CalculateCost(const STNodeStruct& parent, STNodeStruct& child) {
  if (SatisfyConstrain(parent, child) == false) {
    return;
  }
  float safety_cost = SafetyCost(parent, child);
  if (safety_cost >= MAX_COST) {
    return;
  }
  float object_cost = ObjectCost(parent, child);
  float comfort_cost = ComfortCost(parent, child);
  float speed_cost = 0.3 * (parent.s - child.s);
  float edge_cost = safety_cost + object_cost + comfort_cost + speed_cost;
  if (edge_cost + parent.total_cost < child.total_cost) {
    child.total_cost = edge_cost + parent.total_cost;
    child.safety_cost = safety_cost;
    child.object_cost = object_cost;
    child.comfort_cost = comfort_cost;
    child.last_s = parent.s;
    child.v = (child.s - parent.s) / (child.t - parent.t);
    child.a = (child.v - parent.v) / (child.t - parent.t);
  }
}

bool ProbGridSearch::SatisfyConstrain(const STNodeStruct& parent, 
                                      const STNodeStruct& child) {
  if (child.s < parent.s) {
    return false;
  }
  double a = ((child.s - parent.s) / (child.t - parent.t) - parent.v) / (child.t - parent.t);
  if (a < FLAGS_max_dec || a > FLAGS_max_acc) {
    return false;
  }  
  double speed_limit = 
      context_->planning_config_.speedplan_config.maximum_cruising_speed / 3.6;
  if (speed_limit_.lower_bound(child.s) != speed_limit_.end()) {
    speed_limit = speed_limit_.lower_bound(child.s)->second;
  }
  auto st_map_ptr = search_line_->GetSTMapPtr();
  auto& point = st_map_ptr[child.t_id][child.s_id];
  if(point.p > 0.2) {
    double delta_v = 5.0 - 5.0 * point.p;
    for (auto& obj : point.objs) {
      if (search_line_->objects_.count(obj.first) && 
          context_->cognition_info_->scenario_info.invader_.count(obj.first)) {
        double v = std::max(search_line_->objects_[obj.first].vsabs + delta_v, 0.0);
        speed_limit = std::min(speed_limit, v);
      }
    }
  }
  if ((child.s - parent.s) / (child.t - parent.t) > speed_limit) {
    return false;
  }
  return true;
}

float ProbGridSearch::SafetyCost(const STNodeStruct& parent, 
                                 const STNodeStruct& child) {
  float cost = 0.0;
  double v = (child.s - parent.s) / (child.t - parent.t);
  int start_s = parent.s_id;
  for (int t = parent.t_id + 1; t <= child.t_id; t++) {
    float temp_s = parent.s + v * (t - parent.t_id) * FLAGS_scale_t;
    int temp_s_id = (int)(temp_s / FLAGS_scale_s);

    const int range_t = search_line_->GetSTMapRangeT();
    const int range_s = search_line_->GetSTMapRangeS();
    auto st_map_ptr = search_line_->GetSTMapPtr();

    auto& points = st_map_ptr[t];
    if (temp_s_id < range_s) {
      if (points[temp_s_id].p >= FLAGS_p_threshold) {
        return MAX_COST;
      }
      double temp_cost;
      search_line_->st_map.GetCost(points[temp_s_id].index, v, temp_cost);
      cost += temp_cost;
    }
  }
  return FLAGS_scale_t * cost;
}

float ProbGridSearch::ObjectCost(const STNodeStruct& parent, 
                                 const STNodeStruct& child) {
  if (context_->st_data_.empty()) {
    return 0.0;
  }
  std::map<int, int> decision;
  float v = (child.s - parent.s) / (child.t - parent.t);
  double cost = 0.0, right_cost = 0.0;
  for (auto& object : context_->st_data_) {
    if (object.second.key_points.empty()) {
      continue;
    }
    auto& key_points = object.second.key_points;
    for (int t = parent.t_id + 1; t <= child.t_id; t++) {
      float end_s = parent.s + v * (t - parent.t_id) * FLAGS_scale_t;
      if (key_points.count(t)) {
        int temp_decision = -1;
        if (end_s > FLAGS_scale_s * *key_points.find(t)->second.rbegin()) {
          temp_decision = 1;
        } else if (end_s < FLAGS_scale_s * *key_points.find(t)->second.begin()) {
          temp_decision = 2;
        }
        if (!decision.count(object.first)) {
          decision[object.first] = temp_decision;
        } else if (decision[object.first] != temp_decision || temp_decision < 0) {
          decision[object.first] = 3;
        }
      }
    }
    if (object.second.right_of_way > 0 && decision.count(object.first) && 
        decision[object.first] == 1) {
      right_cost += 1.0;
      if (object.second.priority > 0 || right_cost > 0.1 * MAX_COST) {
        right_cost = 0.1 * MAX_COST;
        continue;
      }
      if (current_line_->objects_.count(object.first) && 
          current_line_->objects_[object.first].type == 2) {
        right_cost += 1.0;
      }
    }
  }
  if (context_->dis_to_junction_ < 1e-3 && current_line_->mapinfo.lane_turns.size() && 
      current_line_->mapinfo.lane_turns.front().second == 2 || context_->ego_speed_ < 1.0) {
    right_cost *= 2.0;
  }
  for (auto& object : decision) {
    if (context_->st_data_[object.first].pre_decision == 0) {
      cost += 0.0;
    } else if (object.second > 2) {
      cost += 1.0;
    } else if (object.second != context_->st_data_[object.first].pre_decision) {
      cost += 1.0 + context_->st_data_[object.first].decision_time;

      // 在路口内，对距离较近的障碍物决策结果频繁变化会有危险，尤其从让行变为超车，因此加大代价
      if(context_->dis_to_junction_ < 10.00 && context_->st_data_[object.first].pre_decision == 2){
        if(current_line_->objects_[object.first].sl_boundary.min_s < 15.0){
          cost += context_->st_data_[object.first].decision_time*10;
        } else if(current_line_->objects_[object.first].sl_boundary.min_s < 30.0){
          cost += context_->st_data_[object.first].decision_time*5;
        }
      }
    } 
  } 
  return cost + right_cost;
}

float ProbGridSearch::ComfortCost(const STNodeStruct& parent, 
                                  const STNodeStruct& child) {
  float a = ((child.s - parent.s) / (child.t - parent.t) - parent.v) / (child.t - parent.t);
  // return (child.t - parent.t) * (a * a + 5.0 * (a - parent.a) * (a - parent.a));
  return 300.0 * fabs(a) + 10000.0 * fabs(a - parent.a);
}

void ProbGridSearch::GetFinalDecision() {
  float max_cost = MAX_COST;
  STNodeStruct goal;
  for (auto& point : points_.back()) {
    if (point.second.total_cost < max_cost) {
      max_cost = point.second.total_cost;
      goal = point.second;
    }
  }
  float last_s = goal.last_s;
  s_sequence_.push_back(goal.s);
  AWARN_IF(FLAGS_log_enable) << 
      "s = " << goal.s << " v = " << goal.v << " a = " << goal.a << 
      " comfort = " << goal.comfort_cost << " safety = " << goal.safety_cost << 
      " object = " << goal.object_cost << " cost = " << goal.total_cost;
  for (int i = points_.size() - 2; i >= 0; i--) {
    for (auto& point : points_.at(i)) {
      if (point.second.s == last_s) {
        last_s = point.second.last_s;
        if (point.second.t >= 4.0) {
          float s = (s_sequence_.back() + point.second.s) / 2.0;
          s_sequence_.push_back(s);
        }
        s_sequence_.push_back(point.second.s);
        AWARN_IF(FLAGS_log_enable) << "s = " << point.second.s << " v = " << 
            point.second.v << " a = " << point.second.a << " comfort = " << 
            point.second.comfort_cost << " safety = " << 
            point.second.safety_cost << " object = " << 
            point.second.object_cost << " cost = " << point.second.total_cost;
        if (fabs(point.second.a) > 1.0) {
          AERROR_IF(FLAGS_log_enable) << "uncomfort_a = " << point.second.a;
        }
        break;
      }
    }
  }
  reverse(s_sequence_.begin(), s_sequence_.end());
  for (auto s : s_sequence_) {
    planning_debug_msgs::DecisionPoint point;
    point.set_s(s);
    point.set_t(context_->decision_result_.st_graph.decision_points().size());
    context_->decision_result_.st_graph.add_decision_points()->CopyFrom(point);
  }  
}

void ProbGridSearch::GetObjectDecision() {
  auto& object_decision = context_->decision_result_.object_decision;
  auto& last_object_decision = context_->decision_result_.last_object_decision;
  for (auto& object : context_->st_data_) {
    if (object.second.max_p < FLAGS_p_thd_cost) {
      continue;
    } else {
      int t = (int)object.second.center_t;
      AWARN_IF(FLAGS_log_enable) << "id = " << object.first << " center_t = " << 
          object.second.center_t << " center_s = " << 
          object.second.center_s << " p = " << object.second.max_p;
      float s = s_sequence_.back();
      if (t + 1 < s_sequence_.size()) {
        float v = s_sequence_.at(t + 1) - s_sequence_.at(t);
        s = s_sequence_.at(t) + v * (object.second.center_t - t);
      }    
      int pre_decision = object.second.pre_decision;  
      object.second.pre_decision = s > object.second.center_s ? 1 : 2;
      if (pre_decision > 0 && object.second.pre_decision != pre_decision) {
        object.second.decision_time = 0.1;
        AERROR_IF(FLAGS_log_enable) << object.first << " decision changed!";
      } else {
        object.second.decision_time += 0.1;
      }
      if (object.second.pre_decision == 1) {
        //让->超:如果上帧目标决策是让，当前决策变为超，连续决策超1秒，决策超，有可能给让但没解，导致规划失败
        if (last_object_decision.count(object.first) && 
            last_object_decision[object.first] != eObjectDecisionEnum::TAKEWAY &&
            object.second.keep_decision_time < 1.0) {
            object.second.keep_decision_time += 0.1;
            object_decision[object.first] =  last_object_decision[object.first];
            AWARN_IF(FLAGS_log_enable) << "Not takeway decision keep！"<< object.first ;
            continue;
        }
        object.second.keep_decision_time = 0.0;
        //end
        object_decision[object.first] = eObjectDecisionEnum::TAKEWAY;
      } else if (object.second.conflict_type > 0 && object.second.conflict_type < 5 && 
          search_line_->objects_.at(object.first).sl_boundary.max_s > 
          context_->planning_config_.car_model.front_over_hang) {
        // if (object.second.conflict_type > 1 && current_line_->objects_.count(object.first) 
        //     && reference_line_->path_in_current && 
        //     current_line_->objects_.at(object.first).type < 2) {
        //   auto& sl = current_line_->objects_.at(object.first).sl_boundary;
        //   double s = 0.5 * (sl.min_s, sl.max_s), left_w = 1.75, right_w = 1.75;
        //   current_line_->GetWidthToLaneBoundary(left_w, right_w, s);
        //   if (sl.max_l > -right_w && sl.min_l < left_w && 
        //       (!current_line_->block_l_.count(object.first) || 
        //       current_line_->block_l_[object.first] > 1e-3)) {
        //     AWARN_IF(FLAGS_log_enable) << "id = " << object.first << " speed limit";
        //     object_decision[object.first] = eObjectDecisionEnum::SPEED_LIMIT;
        //     continue;
        //   } else {
        //     AINFO_IF(FLAGS_log_enable)<< "left_w = " << left_w << " right_w = " << right_w;
        //   }
        // }
        //让行->跟随：如果上帧目标决策是让行，当前帧决策跟随，连续决策跟随0.5秒后，决策跟随
        if (last_object_decision.count(object.first) && 
            last_object_decision[object.first] == eObjectDecisionEnum::GIVEWAY &&
            object.second.keep_decision_time < 0.5) {
            object.second.keep_decision_time += 0.1;
            object_decision[object.first] =  eObjectDecisionEnum::GIVEWAY;
            AWARN_IF(FLAGS_log_enable)  << " giveway decision keep！"<< object.first;
            continue;
        }
        object.second.keep_decision_time = 0.0;
        //end
        object_decision[object.first] = eObjectDecisionEnum::FOLLOW;
      } else {
        //跟随->让行：如果上帧目标决策是跟随，当前帧决策让行，连续决策让行0.5秒后，决策让行
        if (last_object_decision.count(object.first) && 
            last_object_decision[object.first] == eObjectDecisionEnum::FOLLOW &&
            object.second.keep_decision_time < 0.5 ) {
            object.second.keep_decision_time += 0.1;
            object_decision[object.first] =  eObjectDecisionEnum::FOLLOW;
            AWARN_IF(FLAGS_log_enable) << "follow decision keep！" << object.first ;
            continue;
        }
        object.second.keep_decision_time = 0.0;
        //end
        object_decision[object.first] = eObjectDecisionEnum::GIVEWAY;
      }      
    }
  }
  for (auto object : context_->decision_result_.object_decision) {
    AWARN_IF(FLAGS_log_enable) << "id = " << object.first << 
        " decision = " << (int)object.second << " time = " << 
        context_->st_data_[object.first].decision_time;
  }
  for (auto object : context_->decision_result_.last_object_decision) {
    AWARN_IF(FLAGS_log_enable) << "id = " << object.first << 
        " last_decision = " << (int)object.second << " time = " << 
        context_->st_data_[object.first].decision_time;
  }
  if (context_->decision_result_.reference_line_id >= 20 && FLAGS_gap_enable) {
    if (context_->decision_result_.gap_id.first > 0 && 
        context_->st_data_.count(context_->decision_result_.gap_id.first) && 
        context_->st_data_[context_->decision_result_.gap_id.first].decision_time < 1.5) {
      object_decision[context_->decision_result_.gap_id.first] = 
            eObjectDecisionEnum::TAKEWAY;
    }
    if (context_->decision_result_.gap_id.second > 0 && 
        context_->st_data_.count(context_->decision_result_.gap_id.first) && 
        context_->st_data_[context_->decision_result_.gap_id.second].decision_time < 1.5) {
      object_decision[context_->decision_result_.gap_id.second] = 
          eObjectDecisionEnum::FOLLOW;
    }
  }

  last_object_decision = object_decision;

}

}  //  namespace planning
}  //  namespace acu
