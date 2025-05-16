#include "object_evaluation.h"

namespace acu {
namespace planning {

ObjectEvaluation::ObjectEvaluation() {}

ObjectEvaluation::~ObjectEvaluation() {}

bool ObjectEvaluation::DepartureDecision() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  return context_->decision_result_.reference_line_id;
}

// Set stop point and speed limit to guarantee success and comfort
void ObjectEvaluation::SetLastLCPoint() {
  if (FLAGS_mission_lc_stop_enable == false || 
      context_->scenario_type == eScenarioEnum::PULLOVER) {
    return;
  }
  auto line_id = context_->decision_result_.reference_line_id;
  auto target_line = context_->reference_line_map_[line_id];
  if (target_line->dis_to_last_lc > 150.0 || target_line->dis_to_last_lc < 0.1) {
    return;
  } 
  if (fabs(current_line_->mapinfo.first_lc_time) > 1) {
    auto& passable_s = target_line->mapinfo.first_lc_time < 0 ? 
        target_line->mapinfo.left_passable_distances : 
        target_line->mapinfo.right_passable_distances;
    double dis_to_lc = passable_s.size() ? passable_s.back().second : 
        target_line->dis_to_last_lc;
    lc_speed_limit_ = dis_to_lc / fabs(target_line->mapinfo.first_lc_time) / 3.0;
  }
  double dis_to_end = std::max(target_line->dis_to_last_lc - 1.0, 0.0);
  AWARN_IF(FLAGS_log_enable) << "dis_to_lc_point = " << dis_to_end;
  double acc = context_->ego_speed_ > 6.0 ? 1.0 : 0.6;
  if (target_line->reference_lane_id < 20) {
    if (dis_to_end - 9.0 > pow(context_->ego_speed_, 2) / 2.0 / acc) {
      dis_to_end -= 9.0;
      AWARN_IF(FLAGS_log_enable) << "dis_to_lc_point_1 = " << dis_to_end;
    } else if (dis_to_end - 4.0 > pow(context_->ego_speed_, 2) / 2.0 / acc) {
      dis_to_end -= 4.0;
      AWARN_IF(FLAGS_log_enable) << "dis_to_lc_point_2 = " << dis_to_end;
    }
  }
  int index;
  Site point;
  dis_to_end += context_->planning_config_.car_model.front_over_hang;
  target_line->GetNearestPoint(dis_to_end, point, index);
  AddStopPoint(point);
  context_->stop_s_ = std::min(context_->stop_s_, dis_to_end);
  context_->stop_type_ += pow(2, 2);
}

void ObjectEvaluation::SetFollowLCPoint() {
  if (context_->scenario_type == eScenarioEnum::PULLOVER || 
      context_->best_option_.type < 3 || context_->dis_to_junction_ < 80.0 ||
      current_line_->mapinfo.rightside_length < 1e-3) {
    return;
  }
  double dis_to_solid = 0.0;
  for (auto& obj : current_line_->objects_) {
    if (obj.second.is_static == false || obj.second.sl_boundary.min_s < 0.0) {
      continue;
    }
    auto& object = obj.second;
    double bound_l, bound_r;
    current_line_->GetWidthToRoadBoundary(bound_l, bound_r, object.sl_boundary.min_s);
    if (current_line_->block_l_.count(object.id) && 
        current_line_->block_l_[object.id] < FLAGS_collision_buff && (object.type == 4 || 
        object.type < 2 && object.sl_boundary.min_l + bound_r < 0.3)) {
      double last_s = 0.0;
      for (auto& bd : current_line_->mapinfo.left_bd_types) {
        if (bd.first > object.sl_boundary.min_s) {
          dis_to_solid = bd.second > 2 ? last_s : dis_to_solid;
          break;
        }
        last_s = bd.first;
      }
    }
  }
  if (dis_to_solid > FLAGS_min_lc_dis + pow(context_->ego_speed_, 2) / 4.0) {
    dis_to_solid -= 10.0;
    AWARN_IF(FLAGS_log_enable) << "dis_to_solid = " << dis_to_solid;
    int index;
    Site point;
    dis_to_solid += context_->planning_config_.car_model.front_over_hang;
    current_line_->GetNearestPoint(dis_to_solid, point, index);
    AddStopPoint(point);
    context_->stop_s_ = std::min(context_->stop_s_, dis_to_solid);
    context_->stop_type_ += pow(2, 7);
  }
}

void ObjectEvaluation::CrosswalkDecision() {
  bool temp_slowdown = false;
  double stop_dis = 100.0;
  CrosswalkTempDecision(temp_slowdown, stop_dis);
  if (temp_slowdown) {
    context_->crosswalk_slowdown_ = true;
  } else if (context_->crosswalk_slowdown_counter_ == 5) {
    context_->crosswalk_slowdown_ = false;
    context_->crosswalk_slowdown_counter_ = 0;
  } else {
    context_->crosswalk_slowdown_counter_++;
  }
  if (stop_dis < 100.0) {
    if (context_->crosswalk_stop_ || context_->crosswalk_stop_counter_ == 2) { 
      int index;
      current_line_->GetNearestPoint(stop_dis, context_->crosswalk_point_, index);
      context_->crosswalk_stop_ = true;
      context_->crosswalk_stop_counter_ = 0;
    } else {
      context_->crosswalk_stop_counter_++;
    }    
  } else {
    if (!context_->crosswalk_stop_ || context_->crosswalk_stop_counter_ == 5) {
      context_->crosswalk_stop_ = false;
      context_->crosswalk_stop_counter_ = 0;
    } else {
      context_->crosswalk_stop_counter_++;
    }
  }
  if (context_->crosswalk_stop_) {
    AddStopPoint(context_->crosswalk_point_);
    double l;
    XYToSL(current_line_->mapinfo, context_->crosswalk_point_, stop_dis, l);
    stop_dis -= context_->planning_config_.car_model.front_over_hang;
    context_->stop_s_ = std::min(context_->stop_s_, stop_dis);
    context_->stop_type_ += pow(2, 4);
    AWARN_IF(FLAGS_log_enable) << "crosswalk_stop_s = " << stop_dis;
  }
}

void ObjectEvaluation::CrosswalkTempDecision(bool& is_slowdown, double& stop_dis) {
  auto front_over_hang = context_->planning_config_.car_model.front_over_hang;
  for (auto& crosswalk : current_line_->mapinfo.distance_to_crosswalks) {
    if (crosswalk.first > FLAGS_front_perception_range) {
      break;
    }
    AWARN_IF(FLAGS_log_enable) << "start_s = " << crosswalk.first << " end_s = " << crosswalk.second;
    double start_s = crosswalk.first - front_over_hang;
    double end_s = crosswalk.first - front_over_hang;
    if (start_s < 0.0) {
      continue;
    }
    for (auto it = current_line_->objects_.begin(); it != current_line_->objects_.end(); it++) {
      auto &object = it->second;
      auto& sl = object.sl_boundary;
      auto& st = object.st_boundary.s_t;
      if (sl.max_s < 0.0 || object.type < 2 || object.type > 3 || 
          sl.min_s > crosswalk.second + 0.5 || sl.max_s < crosswalk.first - 0.5 ||
          fabs(sl.min_l) > 5.0 && fabs(sl.max_l) > 5.0) {
        continue;
      }
      if (fabs(sl.min_l) < 4.0 || fabs(sl.max_l) < 4.0 || st.size()) {
        is_slowdown = true;
        AWARN_IF(FLAGS_log_enable) << "Crosswalk slowdown because of " << object.id;
      }
      if (object.conflict_type > 1 && st.size() && start_s < stop_dis) {
        double conflict_s = std::min(st.front().first.x(), st.back().first.x());
        double average_v = (context_->ego_speed_ + FLAGS_crosswalk_speed_limit) / 2.0;
        // ignore logic : collison_s is small or collison_t is big
        if (conflict_s < context_->ego_speed_ / 3.0 + FLAGS_min_yield_dis ||  
            st.front().first.y() > std::max(conflict_s, 0.0) / average_v + FLAGS_min_yield_dis) {
          AWARN_IF(FLAGS_log_enable) << "Can't yield " << object.id;
          continue;
        }
        if (start_s > context_->ego_speed_ * context_->ego_speed_ / 2.0) {
          stop_dis = crosswalk.first - 1.0;
          AWARN_IF(FLAGS_log_enable) << "Crosswalk stop because of " << object.id;
        }
      }
    }
  }
}

void ObjectEvaluation::SpeedLimitDecision() {
  auto speed = DataPool::Instance()->GetMainDataRef().task_content.speed_limit;
  if (current_line_->meeting_car) {
    if (current_line_->meeting_state > 1) {
      speed = std::min(speed, 30.0 / 3.6);
    }
  }
  if (context_->scenario_type == eScenarioEnum::PULLOVER) {
    speed = std::min(speed, context_->decider_config_->parking().cruise_speed());
  }
  if (context_->crosswalk_slowdown_) {
    speed = std::min(speed, FLAGS_crosswalk_speed_limit);
  }
  if (context_->replan_state_ == eReplanStateEnum::PREPARE && 
      context_->best_option_.type > 2 && context_->best_option_.type < 5) {
    float time = (FLAGS_decision_frame - context_->replan_counter_) / 10.0;
    speed = std::min(speed, context_->best_option_.lc_s.second / (5.0 + time));
  }
  if (context_->dis_to_mission_ > 200.0) {
    speed = std::min(speed, lc_speed_limit_);
  } else {
    speed = std::min(speed, 10.0);
  }
  speed = std::min(speed, context_->fit_gap_speed_);
  speed = std::min(speed, v2x_expected_speed);
  context_->decision_result_.speed_limit = speed;
  AWARN_IF(FLAGS_log_enable) << "final speed limit = " << context_->decision_result_.speed_limit;
}
// Generate flag to control stop and start
void ObjectEvaluation::VirtualObjectDecision() {
  int index;
  Site point;
  SentenceStruct sentence;
  sentence.action = eActionEnum::STOP;
  auto target_id = context_->decision_result_.reference_line_id;
  auto target_line = context_->reference_line_map_[target_id];
  auto light = target_line->mapinfo.trafficlight;
  AWARN_IF(FLAGS_log_enable) << "light id" << light.light_id
                             << " ,is v2x light " << light.is_v2x_traffic_light 
                             << " , g " << light.green_period
                             << " , y " << light.yellow_period
                             << " , r " << light.red_period;

  TrafficLightDecision(light);
  if (context_->trafficlight_stop_) { 
    if(light.light_s > -10.0){
      int index;
      Site point;
      //light.light_s += context_->planning_config_.car_model.front_over_hang;
      target_line->GetNearestPoint(light.light_s, point, index);
      //AddStopPoint(point);
      Vec2d stop_point(point.xg, point.yg);
      double angle = point.globalangle * acos(-1) / 180.0;
      SentenceStruct sentence;
      sentence.action = eActionEnum::STOP;    
      sentence.box = common::math::Box2d(stop_point, angle, 0.1, 4.0);
      context_->decision_result_.sentence.push_back(sentence);
    }
    //sentence.box = light.box;
    //AWARN_IF(FLAGS_log_enable)<<"light box ("<<sentence.box.center_x()<<", "<<sentence.box.center_y()<<").";
    //context_->decision_result_.sentence.push_back(sentence);
    context_->stop_s_ = std::min(context_->stop_s_, light.light_s);
    context_->stop_type_ += pow(2, 1);
    AWARN_IF(FLAGS_log_enable) << "Traffic light STOP!";
  } 
  if (FLAGS_specific_pull_over_enable && !context_->find_destination_ && 
      context_->dis_to_mission_ < FLAGS_pull_over_dis && 
      context_->planning_config_.behavior_config.enable_struct_pull_over) {
    // double end_s = context_->park_road_side_ ? 
    //     context_->dis_to_mission_ - FLAGS_min_lc_dis : context_->dis_to_mission_;
    double end_s = context_->dis_to_mission_ - FLAGS_min_lc_dis;
    target_line->GetNearestPoint(end_s, point, index);
    AddStopPoint(point);
    end_s -= context_->planning_config_.car_model.front_over_hang;
    context_->stop_s_ = std::min(context_->stop_s_, end_s);
    context_->stop_type_ += pow(2, 5);
    AWARN_IF(FLAGS_log_enable) << "dis_to_must_pullover = " << end_s;
  } else if (context_->pull_over_) {
    context_->stop_s_ = std::min(context_->stop_s_, context_->dis_to_mission_);
    context_->stop_type_ += pow(2, 5);
  }
}

void ObjectEvaluation::TrafficLightDecision(const StructTrafficlightInfo& light) {
   AWARN_IF(FLAGS_log_enable) << "light_stop = " << 
      context_->trafficlight_stop_ << " light_s = " << light.light_s;
  // Wait for green light during stopping
  if (context_->dis_to_junction_ < -10.0 && light.light_s > 50.0) {
    context_->trafficlight_stop_ = false;
    return;
  }
  if (context_->trafficlight_stop_ && light.history_light.size()) {
    if (light.history_light.back().color == 3 && 
        (light.history_light.back().left_time < 1e-3 || 
        light.history_light.back().left_time > 7.9)) {
      context_->trafficlight_stop_ = false;
    }
    return;
  }
  // Check red and yellow light during cruise
  if (light.light_s > FLAGS_front_perception_range || light.light_s < 0.5 || 
      light.history_light.empty()) {
    return;
  }
  // Calculate expected time of reaching stop line according to last trajectory
  double eta = 0.0;
  for (auto &point : context_->trajectory_info_.path) {
    eta = point.t;
    if (point.length > light.light_s) {
      break;
    }
  }
  int color = light.history_light.back().color;
  double left_time = light.history_light.back().left_time;
  AERROR_IF(FLAGS_log_enable) << "color = " << color << " left_time = " << left_time << " eta = " << eta;
  if (color == 1) {
    context_->trafficlight_stop_ = true;
  } else if (left_time < 1e-3) {
    if (color == 2 && (pow(context_->ego_speed_, 2) < 2.0 * light.light_s || 
        FLAGS_yellow_t - light.history_light.back().time < eta + 1.0)) {
      context_->trafficlight_stop_ = true;
    }
  } else if (color == 2 || color == 3) {
    if (color == 3) {
      left_time += 0.5 * std::max(2.0 * FLAGS_yellow_t - left_time, 0.0);
      AERROR_IF(FLAGS_log_enable) << "green_time = " << left_time;
    } 
    if (left_time < eta) {
      context_->trafficlight_stop_ = true;
    }
  }
}

void ObjectEvaluation::ConflictZoneDecision() {
  auto front_over_hang = context_->planning_config_.car_model.front_over_hang;
  if (current_line_->conflict_s > FLAGS_front_perception_range || 
      current_line_->conflict_s < front_over_hang + 1.0 && 
      context_->conflict_stop_ == false) {
    context_->conflict_stop_ = false;
    context_->conflict_counter_ = 0;
    return;
  }
  double stop_s;
  bool temp_conflict = GetConflictPoint(stop_s);
  if (context_->conflict_stop_ == temp_conflict) {
    context_->conflict_counter_ = 0;
  } else if (context_->conflict_counter_ < 3) {
    context_->conflict_counter_++;
  } else {
    context_->conflict_counter_ = 0;
    context_->conflict_stop_ = temp_conflict;
    if (temp_conflict) {
      int index;
      current_line_->GetNearestPoint(stop_s, context_->conflict_point_, index);
    }
  }
  if (context_->conflict_stop_) {
    AddStopPoint(context_->conflict_point_);
    double s, l;
    XYToSL(current_line_->mapinfo, context_->conflict_point_, s, l);
    context_->stop_s_ = std::min(context_->stop_s_, s - front_over_hang);
    context_->stop_type_ += pow(2, 3);
    AWARN_IF(FLAGS_log_enable) << "conflict_stop_s = " << s - front_over_hang;
  } 
  AWARN_IF(FLAGS_log_enable) << "conflict_stop = " << context_->conflict_stop_ << 
      " counter = " << context_->conflict_counter_;
}

bool ObjectEvaluation::GetConflictPoint(double& stop_s) {
  if (FLAGS_junction_conflict_enable == false) {
    return false;
  }
  auto front_over_hang = context_->planning_config_.car_model.front_over_hang;
  stop_s = current_line_->conflict_s - 1.0 - front_over_hang;
  double dec_s = context_->ego_speed_ * context_->ego_speed_ / 2.0;   
  double conflict_s = FLAGS_front_perception_range;
  bool temp_conflict = false;
  for (auto it = current_line_->objects_.begin(); it != current_line_->objects_.end(); it++) {
    auto &object = it->second;
    if (object.is_static || object.type > 1 || object.conflict_type == 1 || 
        object.st_boundary.s_t.empty()) {
      continue;
    }
    auto& st = object.st_boundary.s_t;
    double s = std::max(std::min(st.front().first.x(), st.back().first.x()), 0.0);
    double t = st.front().first.y();
    if (t < 0.5 || t > 8.0 || t > s / (context_->ego_speed_ + 0.1) + 4.0 || 
        s < pow(context_->ego_speed_, 2) / 3.0 + FLAGS_min_yield_dis || 
        current_line_->mapinfo.distance_to_junctions.size() &&  
        current_line_->mapinfo.distance_to_junctions.front().second < s) { 
      AWARN_IF(FLAGS_log_enable) << "ignore " << object.id;
      continue;
    }
    conflict_s = std::min(conflict_s, s);
    temp_conflict = true;
  }
  AWARN_IF(FLAGS_log_enable) << "stop_s = " << stop_s << " dec_s = " << dec_s;
  if (stop_s < dec_s) {
    stop_s = std::max(stop_s, conflict_s) + front_over_hang;
  }
  return temp_conflict;
}

void ObjectEvaluation::AddStopPoint(const Site& point) {
  Vec2d stop_point(point.xg, point.yg);
  double angle = point.globalangle * acos(-1) / 180.0;
  SentenceStruct sentence;
  sentence.action = eActionEnum::STOP;    
  sentence.box = common::math::Box2d(stop_point, angle, 0.1, 4.0);
  context_->decision_result_.sentence.push_back(sentence);
}

void ObjectEvaluation::CrossObjectDecision() {
  if (context_->decider_config_->object().yield_enable() == false) {
    AWARN_IF(FLAGS_log_enable) << "Shut down object decision!";
    return;
  }
  if (FLAGS_stmap_enable) {
    ProbGridSearch search;
    if (search.GetSearchResult() == false) {
      AWARN_IF(FLAGS_log_enable) << "Object decision failed!";
    }
  } else {
    DynamicMapSearch search;
    if (search.GetSearchResult() == false) {
      AWARN_IF(FLAGS_log_enable) << "Object decision failed!";
    }
  }
}


void ObjectEvaluation::SetGuaranteeReplanPoint(const int ref_lane_id,const int left_or_right, double block_s) {
  if(left_or_right <0 || left_or_right >1) return;
  // 由于静态障碍物导致的不可通行，添加一个虚拟停车点，使车尽量停在实线前的虚线lc距离，保证能够规划成功
  if(context_->cruise_blocked_counter_ < FLAGS_cruise_blocked_frame){
    return;
  }

  if(!context_->reference_line_map_.count(ref_lane_id)){
    return;
  }

  auto target_line = context_->reference_line_map_[ref_lane_id];

  double obj_min_s = 1000.0;
  for(auto& block_l: target_line->block_l_){

    auto obj = target_line->objects_[block_l.first];
    AINFO_IF(FLAGS_log_enable) <<"block_l_,first:"<<block_l.first<<",min_s:"<<obj.sl_boundary.min_s;
    if(obj.sl_boundary.min_s > block_s && (obj.sl_boundary.min_s-block_s) < (obj_min_s - block_s)){
      obj_min_s = obj.sl_boundary.min_s;
    }
  }
  AINFO_IF(FLAGS_log_enable) <<"obj_min_s:"<<obj_min_s;
  block_s = obj_min_s - context_->planning_config_.car_model.front_over_hang;

  auto bd_type = left_or_right==1 ? target_line->left_bds_:
             target_line->right_bds_;
  
  double stop_s = -1.0;
  AINFO_IF(FLAGS_log_enable) <<"SetGuaranteeReplanPoint,block_s:"<<block_s;
  if(bd_type.lower_bound(block_s) == bd_type.end()){
    return;
  }
  auto iter = bd_type.lower_bound(block_s);
  AINFO_IF(FLAGS_log_enable) <<"lower_bound:" << iter->first<<","<<int(iter->second);
  // 虚线，在虚线前lc距离设置
  if(!iter->second){
    AINFO_IF(FLAGS_log_enable) <<"[1] dotted bd, block_s:"<<block_s<<",stop_dis:"<<FLAGS_min_lc_dis + pow(context_->ego_speed_, 2) / 4.0;
    if(block_s> FLAGS_min_lc_dis + pow(context_->ego_speed_, 2) / 4.0){
      stop_s = block_s - FLAGS_min_lc_dis -2.0;
    }
    
  }else{ //实线，找实线前的虚线
    auto prev = std::prev(iter);
    AINFO_IF(FLAGS_log_enable) <<"[2] prev:"<<prev->first<<","<< int(prev->second) <<",stop_dis:"<<FLAGS_min_lc_dis + pow(context_->ego_speed_, 2) / 4.0;
    if(!prev->second){
      double dotted_s = prev->first;
      if(dotted_s> FLAGS_min_lc_dis + pow(context_->ego_speed_, 2) / 4.0){
        stop_s = dotted_s - FLAGS_min_lc_dis - 2.0;
      }
    }
  }
  AINFO_IF(FLAGS_log_enable) <<"stop_s:"<<stop_s;

  // 有效停止距离
  if(stop_s > 0.0){
    int index;
    Site point;
    stop_s += context_->planning_config_.car_model.front_over_hang;
    target_line->GetNearestPoint(stop_s, point, index);
    AddStopPoint(point);
    context_->stop_type_ += pow(2, 8);
  }
}


}  //  namespace planning
}  //  namespace acu
