#include "object_manager.h"

namespace acu {
namespace planning {

ObjectManager::ObjectManager() {}

ObjectManager::~ObjectManager() {}

void ObjectManager::AddObjectInfo() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = context_->reference_line_map_[context_->decision_result_.reference_line_id];
  search_line_ = reference_line_->path_in_current ? 
      current_line_ : &reference_line_->local_reference_line;
  std::set<int> special_objects;
  for (auto meeting_id : context_->decision_result_.meeting_ids) {
    special_objects.insert(meeting_id);
  }
  UpdateObjectInfo();
  AddOriginalMapDebug();
  if (FLAGS_decision_stmap) {
    AccAnalysis predictor;
    predictor.UpdateProbGrid(special_objects);
  }
  if (context_->dis_to_junction_ < 30.0 && reference_line_->path_in_current) {
    JunctionAnalysis analyzer;
    analyzer.AddInteractionInfo();
  }
  for (auto it = context_->st_data_.begin(); it != context_->st_data_.end();) {    
    if (search_line_->objects_.count(it->first) && 
        search_line_->objects_.at(it->first).st_area.pd_objs.size() && 
        special_objects.count(it->first) == 0) {
      it->second.interaction_time += 0.1;
      ++it;
    } else {
      it = context_->st_data_.erase(it);
    }
  }
  AddInteractionMapDebug();
}

void ObjectManager::UpdateObjectInfo() {
  for (auto it = context_->st_data_.begin(); it != context_->st_data_.end(); ++it) {
    it->second.conflict_type = 0;
    it->second.right_of_way = false;
  }
  if (context_->scenario_type == eScenarioEnum::PULLOVER) {
    AddKeyObject(search_line_);
  } else if(context_->decision_result_.reference_line_id < 20) {
    if (reference_line_->path_in_current) {
      FindInvaders();
    }
    if (reference_line_->path_in_current == false) {
      AddKeyObject(search_line_);
    }
  } else {
    AddKeyObject(target_line_);
  } 
  AddKeyObject(current_line_);
  const int range_t = search_line_->GetSTMapRangeT();
  const int range_s = search_line_->GetSTMapRangeS();
  auto st_map_ptr = search_line_->GetSTMapPtr();
  for (int t = 0; t < range_t; t++) {
    for(int s = 0; s < range_s; s++) {
      auto& point = st_map_ptr[t][s];
      for (auto& obj : point.objs) {
        if (context_->st_data_.count(obj.second.id)) {
          if (obj.second.p > context_->st_data_[obj.second.id].max_p) {
            context_->st_data_[obj.first].max_p = obj.second.p;
            context_->st_data_[obj.first].center_t = point.index.t * FLAGS_scale_t;
            context_->st_data_[obj.first].center_s = point.index.s * FLAGS_scale_s;
          }
          if (obj.second.p > FLAGS_p_threshold) {
            context_->st_data_[obj.first].key_points[point.index.t].insert(point.index.s);
          }
        }
      }
    }
  }
}

void ObjectManager::AddKeyObject(const ReferenceLineFrame* line) {
  for (const auto& object : line->objects_) {
    if (object.second.st_area.pd_objs.empty()) {
      if (line->reference_lane_id >= 20 && object.second.sl_boundary.max_s < 
          context_->planning_config_.car_model.front_over_hang && 
          search_line_->objects_.count(object.first) && 
          search_line_->objects_.at(object.first).st_area.pd_objs.size()) {
        AERROR_IF(FLAGS_log_enable) << "rear_car = " << object.first;
      } else {
        continue;
      }
    }
    InteractionInfo object_info;
    if (!context_->st_data_.count(object.first)) {
      context_->st_data_[object.first] = object_info;
    }
    auto& st_info = context_->st_data_[object.first];
    st_info.acc = object.second.acc;
    st_info.max_p = 0.0;
    st_info.key_points.clear();
    for (auto& pd : object.second.st_area.pd_objs) {
      if (pd.conflict_type > 0 && (st_info.conflict_type == 0 || 
          pd.conflict_type < st_info.conflict_type)) {
        st_info.conflict_type = pd.conflict_type;
        st_info.right_of_way = pd.right_of_way;
      }
    }
    if ((st_info.conflict_type == 1 || st_info.conflict_type == 3 || 
        st_info.conflict_type == 4) && object.second.sl_boundary.min_s > 
        context_->planning_config_.car_model.front_over_hang) {
      st_info.controllability = 0;
    } else {
      st_info.controllability = 1;
    }
    AWARN_IF(FLAGS_log_enable) << "conflict_id = " << object.first << " acc = " << 
        st_info.acc << " type = " << st_info.conflict_type << " right = " << 
        st_info.right_of_way << " controllability = " << st_info.controllability;
  }
}

void ObjectManager::FindInvaders() {
  auto& scenario_info = context_->cognition_info_->scenario_info;
  for (auto& invader : scenario_info.invader_) {
    //if(search_line_->objects_.count(invader.first)){
      //auto obj = search_line_->objects_[invader.first];
      //if(obj.type < 2) continue;
    //}
    double l = invader.second;
    bool have_overtake = false;
    if (reference_line_->local_reference_line.objects_.count(invader.first)) {
      auto& obj = reference_line_->local_reference_line.objects_[invader.first];
      double temp_l = std::min(fabs(obj.sl_boundary.min_l), fabs(obj.sl_boundary.max_l));
      if (obj.sl_boundary.min_l * obj.sl_boundary.max_l > 0.0 && FLAGS_collision_buff < 
        temp_l - 0.5 * context_->planning_config_.car_model.car_width) {
        l = temp_l - 0.5 * context_->planning_config_.car_model.car_width;
        l = fabs(obj.sl_boundary.min_l) > fabs(obj.sl_boundary.max_l) ? l : -l;
        have_overtake = true;
        AWARN_IF(FLAGS_log_enable) << "Overtake " << obj.id << " already";
      }
    }
    int side_offset = 0;
    if (scenario_info.enable_offset_) {
      if (l > 0.0) {
        side_offset = context_->decision_result_.borrow_lane_type % 2 ? 0 : 1;
      } else {
        side_offset = context_->decision_result_.borrow_lane_type > 1 ? 0 : 2;
      }
      if (!have_overtake) {
        l = fabs(l) + 0.5;
      }
    } 
    l = fabs(l);
    AWARN_IF(FLAGS_log_enable) << "invader dis = " << l << " side_offset = " << 
        side_offset << " borrow_lane_type = " << context_->decision_result_.borrow_lane_type;
    double p = 1.2 - l;
    if (p < 0.2) {
      search_line_->st_map.DecisionObjProbability(search_line_->objects_[invader.first], p);
      AWARN_IF(FLAGS_log_enable) << "Ignore " << invader.first;
      context_->decision_result_.object_decision[invader.first] = eObjectDecisionEnum::SIDEPASS;
      context_->decision_result_.borrow_lane_type += side_offset;
    } else if (p < 0.9) {
      AINFO_IF(FLAGS_log_enable)<<"behavior obj "<<search_line_->objects_[invader.first].st_area.pd_objs.size();
      search_line_->st_map.DecisionObjProbability(search_line_->objects_[invader.first], p);
      AWARN_IF(FLAGS_log_enable) << "Change st map " << invader.first << " p = " << p;
      context_->decision_result_.object_decision[invader.first] = eObjectDecisionEnum::SIDEPASS;
      context_->decision_result_.borrow_lane_type += side_offset;
    } else {
      AWARN_IF(FLAGS_log_enable) << "Can't overtake " << invader.first;
    }
  }
}

void ObjectManager::AddOriginalMapDebug() {
  for (auto it = context_->st_data_.begin(); it != context_->st_data_.end(); ++it) {
    if (it->second.key_points.empty()) {
      continue;
    }
    planning_debug_msgs::STGraph st_graph;
    st_graph.set_id(it->first);
    for (auto& point : it->second.key_points) {
      st_graph.add_t(point.first * FLAGS_scale_t);
      st_graph.add_min_s(*point.second.begin() * FLAGS_scale_s);
      st_graph.add_max_s(*point.second.rbegin() * FLAGS_scale_s);
    }
    context_->decision_result_.st_graph.add_original_objects()->CopyFrom(st_graph);
  }
}

void ObjectManager::AddInteractionMapDebug() {
  for (auto it = context_->st_data_.begin(); it != context_->st_data_.end(); ++it) {
    if (it->second.key_points.empty()) {
      continue;
    }
    AWARN_IF(FLAGS_log_enable&&(it->second.key_points.size())) << "id = " << it->first << " t = " << 
        it->second.key_points.begin()->first * FLAGS_scale_t << " min_s = " << 
        *it->second.key_points.begin()->second.begin() * FLAGS_scale_s << " max_s = " << 
        *it->second.key_points.begin()->second.rbegin() * FLAGS_scale_s;
    AWARN_IF(FLAGS_log_enable&&(it->second.key_points.size()))<< "id = " << it->first << " t = " << 
        it->second.key_points.rbegin()->first * FLAGS_scale_t << " min_s = " << 
        *it->second.key_points.rbegin()->second.begin() * FLAGS_scale_s << " max_s = " << 
        *it->second.key_points.rbegin()->second.rbegin() * FLAGS_scale_s;
    planning_debug_msgs::STGraph st_graph;
    st_graph.set_id(it->first);
    for (auto& point : it->second.key_points) {
      st_graph.add_t(point.first * FLAGS_scale_t);
      st_graph.add_min_s(*point.second.begin() * FLAGS_scale_s);
      st_graph.add_max_s(*point.second.rbegin() * FLAGS_scale_s);
    }
    context_->decision_result_.st_graph.add_cognition_objects()->CopyFrom(st_graph);
  }
}

}  //  namespace planning
}  //  namespace acu
